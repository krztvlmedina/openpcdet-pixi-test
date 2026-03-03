#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------
# batch_interpolate_kitti_split.sh
#
# For each YAML config in --configs-dir:
#   ROOT/training/velodyne  -> TARGET_ROOT/CONFIG_NAME/training/velodyne
#   ROOT/testing/velodyne   -> TARGET_ROOT/CONFIG_NAME/testing/velodyne
#
# If the target split directory exists AND is non-empty, it is SKIPPED.
# (Training and testing are checked independently.)
#
# Also copies the used config file into:
#   TARGET_ROOT/CONFIG_NAME/<original_config_filename>
# ------------------------------------------------------------

ROOT="/app/data/reduced-kitti/"
TARGET_ROOT="/app/data/output/interpolated-kitti"
CONFIGS_DIR="/app/data/config_files/interpolation/final/"
SCRIPT="/app/packages/dynamic_lidar_interpolation/scripts/interpolate_bin_batch.py"
EXTRA_ARGS=()

usage() {
  cat << 'EOF'
Usage:
  batch_interpolate_kitti_split.sh --root <kitti_root> --target-root <target_root> --configs-dir <configs_dir> --script <python_script>

Required:
  --root         Dataset root containing training/velodyne and testing/velodyne
  --target-root  Root output directory where datasets will be created
  --configs-dir  Directory containing YAML config files (*.yaml or *.yml)
  --script       Path to your python interpolation script:
                 python3 script.py <source_dir> <target_dir> --config <cfg>

Behavior:
  For each config file:
    CONFIG_NAME = filename without extension

    Outputs:
      <target-root>/<CONFIG_NAME>/training/velodyne
      <target-root>/<CONFIG_NAME>/testing/velodyne

    Also copies the config file into:
      <target-root>/<CONFIG_NAME>/<config_filename>

Skipping logic:
  - If a target split directory exists and is non-empty, that split is skipped.
  - Training/testing are processed independently.
  - If both are non-empty, interpolation is skipped for that config,
    but the config file is still copied (overwritten) to the dataset folder.

Example:
  ./batch_interpolate_kitti_split.sh \
    --root /OpenPCDet/data/reduced-kitti \
    --target-root /OpenPCDet/data/interpolated-kitti \
    --configs-dir /OpenPCDet/configs/interpolation \
    --script /OpenPCDet/src/lidar/tools/interpolate_directory.py
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

is_non_empty_dir() {
  local d="$1"
  [[ -d "$d" ]] && [[ -n "$(ls -A "$d" 2>/dev/null)" ]]
}

# -----------------------------
# Parse args
# -----------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --root)
      ROOT="${2:-}"; shift 2;;
    --target-root)
      TARGET_ROOT="${2:-}"; shift 2;;
    --configs-dir)
      CONFIGS_DIR="${2:-}"; shift 2;;
    --script)
      SCRIPT="${2:-}"; shift 2;;
    -h|--help)
      usage; exit 0;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break;;
    *)
      die "Unknown argument: $1 (use --help)";;
  esac
done

[[ -n "$ROOT" ]] || die "--root is required"
[[ -n "$TARGET_ROOT" ]] || die "--target-root is required"
[[ -n "$CONFIGS_DIR" ]] || die "--configs-dir is required"
[[ -n "$SCRIPT" ]] || die "--script is required"

[[ -d "$ROOT" ]] || die "Root directory not found: $ROOT"
[[ -d "$CONFIGS_DIR" ]] || die "Configs directory not found: $CONFIGS_DIR"
[[ -f "$SCRIPT" ]] || die "Python script not found: $SCRIPT"

TRAIN_SRC="$ROOT/training/velodyne"
TEST_SRC="$ROOT/testing/velodyne"

[[ -d "$TRAIN_SRC" ]] || die "Missing training velodyne folder: $TRAIN_SRC"
[[ -d "$TEST_SRC" ]] || die "Missing testing velodyne folder: $TEST_SRC"

mkdir -p "$TARGET_ROOT"

mapfile -t CONFIG_FILES < <(find "$CONFIGS_DIR" -maxdepth 1 -type f \( -name "*.yaml" -o -name "*.yml" \) | sort)
[[ ${#CONFIG_FILES[@]} -gt 0 ]] || die "No YAML configs found in: $CONFIGS_DIR"

echo "Root:        $ROOT"
echo "Train src:   $TRAIN_SRC"
echo "Test src:    $TEST_SRC"
echo "Target root: $TARGET_ROOT"
echo "Configs dir: $CONFIGS_DIR"
echo "Script:      $SCRIPT"
echo "Configs:     ${#CONFIG_FILES[@]}"
echo

for cfg in "${CONFIG_FILES[@]}"; do
  cfg_name="$(basename "$cfg")"
  cfg_stem="${cfg_name%.*}"

  dataset_out="$TARGET_ROOT/$cfg_stem"
  train_out="$dataset_out/training/velodyne"
  test_out="$dataset_out/testing/velodyne"

  echo "============================================================"
  echo "Config:      $cfg_name"
  echo "Dataset out: $dataset_out"
  echo "============================================================"

  # Always ensure dataset folder exists and copy config there
  mkdir -p "$dataset_out"
  cp -f "$cfg" "$dataset_out/$cfg_name"
  echo "Copied config -> $dataset_out/$cfg_name"

  train_skip=false
  test_skip=false

  if is_non_empty_dir "$train_out"; then
    echo "SKIP TRAIN: target exists and is non-empty -> $train_out"
    train_skip=true
  fi

  if is_non_empty_dir "$test_out"; then
    echo "SKIP TEST:  target exists and is non-empty -> $test_out"
    test_skip=true
  fi

  if [[ "$train_skip" == true && "$test_skip" == true ]]; then
    echo "SKIP INTERPOLATION: both splits already generated."
    echo
    continue
  fi

  mkdir -p "$dataset_out/training"
  mkdir -p "$dataset_out/testing"

  if [[ "$train_skip" == false ]]; then
    echo "[1/2] Interpolating TRAIN split..."
    python3 "$SCRIPT" "$TRAIN_SRC" "$train_out" --config "$cfg" "${EXTRA_ARGS[@]}"
    echo
  fi

  if [[ "$test_skip" == false ]]; then
    echo "[2/2] Interpolating TEST split..."
    python3 "$SCRIPT" "$TEST_SRC" "$test_out" --config "$cfg" "${EXTRA_ARGS[@]}"
    echo
  fi
done

echo "All interpolations finished."
