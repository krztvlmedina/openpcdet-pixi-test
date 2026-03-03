#!/usr/bin/env bash
# run_full_pipeline.sh — Master pipeline: downsample → interpolate → evaluate → report
#
# Executes all steps sequentially and organises outputs under:
#
#   results/<TIMESTAMP>/
#     raw_results/
#       desempeno_offline/          ← batch detection evaluations + saved bboxes
#         original/                 ← eval_all_original results
#         downsampled/              ← eval_all_downsampled results
#         interpolated/             ← eval_all_interpolated results (per config)
#         detections/               ← save_detections.py output (per model/dataset)
#       desempeno_online/           ← run_realtime_eval.sh output
#     latex/
#       desempeno_offline/          ← AP + recall tables
#       desempeno_online/           ← aggregated realtime results
#
# Prerequisites:
#   • Docker containers ${CONTAINER_VELODYNE} and ${CONTAINER_OPENPCDET} running
#   • KITTI original data mounted at ${OPC_DATA}/kitti  (images, calib, labels)
#   • Pretrained models at ${OPC_DATA}/pretrained-models
#   • Interpolation configs at ${VEL_INTERP_CONFIG_DIR}
#
# Usage:
#   ./run_full_pipeline.sh [--skip-downsample] [--skip-interp] [--skip-offline]
#                          [--skip-online] [--single-model M] [--single-config C]
#                          [--rate HZ] [--max-frames N]
set -euo pipefail

# ─── Containers ──────────────────────────────────────────────────────────────
CONTAINER_VELODYNE="${CONTAINER_VELODYNE:-velodyne_ros2}"
CONTAINER_OPENPCDET="${CONTAINER_OPENPCDET:-velodyne_openpcdet}"

# ─── Paths (inside containers) ───────────────────────────────────────────────
OPC_ROOT="/OpenPCDet"
OPC_DATA="${OPC_ROOT}/data"
OPC_TOOLS="${OPC_ROOT}/src/lidar/tools"
OPC_BATCH="${OPC_TOOLS}/batch_evaluation"

VEL_SCRIPTS="/app/packages/pointcloud_utils/scripts"
VEL_INTERP_CONFIG_DIR="/app/data/config_files/interpolation/final"

KITTI_ORIGINAL="${OPC_DATA}/kitti/training/velodyne"
KITTI_REDUCED="${OPC_DATA}/reduced-kitti"
KITTI_INTERP_BASE="${OPC_DATA}/interpolated-kitti"

# Minimum free space in GB required before starting (estimate for all datasets)
MIN_FREE_GB=50

# ─── Result root (on host, also mounted in openpcdet container) ───────────────
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RESULTS_HOST="$(pwd)/results/${TIMESTAMP}"
RESULTS_OPC="${OPC_ROOT}/results/${TIMESTAMP}"          # same path inside container

OFFLINE_RAW="${RESULTS_OPC}/raw_results/desempeno_offline"
ONLINE_RAW="${RESULTS_OPC}/raw_results/desempeno_online"
OFFLINE_LATEX="${RESULTS_OPC}/latex/desempeno_offline"
ONLINE_LATEX="${RESULTS_OPC}/latex/desempeno_online"

# ─── CLI flags ────────────────────────────────────────────────────────────────
SKIP_DOWNSAMPLE=false
SKIP_INTERP=false
SKIP_OFFLINE=false
SKIP_ONLINE=false
SINGLE_MODEL=""
SINGLE_CONFIG=""
PUBLISH_RATE=10.0
MAX_FRAMES=500
WARMUP_FRAMES=10

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-downsample) SKIP_DOWNSAMPLE=true; shift ;;
        --skip-interp)     SKIP_INTERP=true;     shift ;;
        --skip-offline)    SKIP_OFFLINE=true;    shift ;;
        --skip-online)     SKIP_ONLINE=true;     shift ;;
        --single-model)    SINGLE_MODEL="$2";    shift 2 ;;
        --single-config)   SINGLE_CONFIG="$2";   shift 2 ;;
        --rate)            PUBLISH_RATE="$2";    shift 2 ;;
        --max-frames)      MAX_FRAMES="$2";      shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ─── Models ──────────────────────────────────────────────────────────────────
declare -A MODELS
MODELS[parta2_anchor]="parta2_anchor.yaml PartA2_7940.pth"
MODELS[pointpillar]="pointpillar.yaml pointpillar_7728.pth"
MODELS[pointrcnn_iou]="pointrcnn_iou.yaml pointrcnn_iou_7875.pth"
MODELS[pv_rcnn]="pv_rcnn.yaml pv_rcnn_8369.pth"
MODELS[second]="second.yaml second_7862.pth"
MODELS[second_iou]="second_iou.yaml second_iou7909.pth"

# ─── Helpers ─────────────────────────────────────────────────────────────────
log()  { echo "[$(date +%H:%M:%S)] $*"; }
step() { echo; echo "════════════════════════════════════════════════════════"; echo " $*"; echo "════════════════════════════════════════════════════════"; }

opc()  { docker exec "${CONTAINER_OPENPCDET}" bash -c "source /opt/ros2_humble/install/setup.bash && cd ${OPC_ROOT} && $*"; }
vel()  { docker exec "${CONTAINER_VELODYNE}"  bash -c "pixi run \"$*\""; }
vel_ros() { docker exec "${CONTAINER_VELODYNE}" bash -c "source /opt/ros2_humble/install/setup.bash && $*"; }

# ─── Step 0: Storage check ───────────────────────────────────────────────────
check_storage() {
    log "Checking available storage..."
    # Check free space in the OpenPCDet container's data directory
    local free_gb
    free_gb=$(docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "df -BG '${OPC_DATA}' | awk 'NR==2{gsub(\"G\",\"\",\$4); print \$4}'")
    log "Free space in ${OPC_DATA}: ${free_gb} GB (need ${MIN_FREE_GB} GB)"
    if [[ "${free_gb}" -lt "${MIN_FREE_GB}" ]]; then
        echo "ERROR: Insufficient storage. ${free_gb} GB free, ${MIN_FREE_GB} GB required." >&2
        echo "Free up space or lower MIN_FREE_GB and retry." >&2
        exit 1
    fi
    log "Storage OK."
}

# ─── Step 1: Downsample KITTI 64→16 ─────────────────────────────────────────
run_downsample() {
    step "Step 1: Downsampling KITTI 64-beam → 16-beam (VLP-16 simulation)"
    opc "python3 ${VEL_SCRIPTS}/downsample_64_to_16.py \
            ${KITTI_ORIGINAL} ${KITTI_REDUCED} --batch"
    log "Downsampled dataset written to ${KITTI_REDUCED}"
}

# ─── Step 2: Generate interpolated datasets ───────────────────────────────────
run_interpolation() {
    step "Step 2: Generating interpolated datasets"

    local configs
    configs=$(docker exec "${CONTAINER_VELODYNE}" bash -c \
        "ls '${VEL_INTERP_CONFIG_DIR}'/*.yaml 2>/dev/null | sort")

    if [[ -z "${configs}" ]]; then
        log "WARNING: No interpolation configs found in ${VEL_INTERP_CONFIG_DIR}. Skipping."
        return
    fi

    while IFS= read -r cfg_file; do
        local cfg_name
        cfg_name=$(basename "${cfg_file}" .yaml)
        [[ -n "${SINGLE_CONFIG}" && "${cfg_name}" != "${SINGLE_CONFIG}" ]] && continue

        local out_velodyne="${KITTI_INTERP_BASE}/${cfg_name}/velodyne"
        log "Interpolating with config: ${cfg_name} → ${out_velodyne}"

        # Start interpolation node in velodyne container (background)
        docker exec "${CONTAINER_VELODYNE}" bash -c \
            "pixi run ros2 run dynamic_lidar_interpolation pointcloud_interpolation_node \
             --ros-args --params-file '${cfg_file}'" &
        local interp_pid=$!
        sleep 3

        # Start cloud saver in openpcdet container (background)
        opc "python3 ${VEL_SCRIPTS}/save_interpolated_clouds.py \
                --input-dir  ${KITTI_REDUCED} \
                --output-dir ${out_velodyne} \
                --topic      interpolated_point_cloud \
                --timeout    30" &
        local saver_pid=$!

        # Start bin publisher (plays reduced frames, no loop)
        vel "ros2 run pointcloud_utils bin_publisher_node \
             --ros-args -p bin_directory:=${KITTI_REDUCED} \
             -p publish_rate:=5.0 -p loop:=false -p topic:=velodyne_points"

        wait "${saver_pid}" 2>/dev/null || true
        kill "${interp_pid}" 2>/dev/null || true
        wait "${interp_pid}" 2>/dev/null || true

        log "Done: ${cfg_name}"
    done <<< "${configs}"
}

# ─── Step 3: Offline batch evaluations ───────────────────────────────────────
run_batch_eval() {
    # Runs setupandruntest.py for all (model, dataset) combinations in a given
    # config directory, then moves results to $dest_dir.
    # Args: cfg_subdir  data_root  image_set_root  lidar_root  dest_dir  [dataset_label]
    local cfg_subdir="$1" data_root="$2" img_root="$3" lidar_root="$4" dest_dir="$5"
    local dataset_label="${6:-$(basename "${lidar_root}")}"

    opc "mkdir -p '${dest_dir}'"

    for MODEL in $(printf '%s\n' "${!MODELS[@]}" | sort); do
        [[ -n "${SINGLE_MODEL}" && "${MODEL}" != "${SINGLE_MODEL}" ]] && continue
        read -r CFG CKPT <<< "${MODELS[$MODEL]}"
        log "  [${dataset_label}] Model: ${MODEL}"
        opc "python3 ${OPC_TOOLS}/setupandruntest.py \
                --dataset-root '${data_root}' \
                --image-set-root '${img_root}' \
                --lidar-root '${lidar_root}' \
                --cfg_file ${OPC_TOOLS}/cfgs/kitti_models/${cfg_subdir}/${CFG} \
                --batch_size 1 \
                --ckpt ${OPC_DATA}/pretrained-models/${CKPT}"
        # Move output from OpenPCDet's default output location
        local src="${OPC_ROOT}/output/${OPC_TOOLS#${OPC_ROOT}/}/cfgs/kitti_models/${cfg_subdir}/${MODEL}"
        opc "mkdir -p '${dest_dir}/${dataset_label}' && \
             mv '${src}' '${dest_dir}/${dataset_label}/${MODEL}' 2>/dev/null || true"
    done
}

run_offline_evals() {
    step "Step 3: Offline batch evaluations"
    opc "mkdir -p '${OFFLINE_RAW}/original' '${OFFLINE_RAW}/downsampled' \
                   '${OFFLINE_RAW}/interpolated'"

    # 3a. Original KITTI
    log "--- Original KITTI ---"
    run_batch_eval "original" \
        "${OPC_DATA}/kitti" "${KITTI_ORIGINAL}" "${KITTI_ORIGINAL}" \
        "${OFFLINE_RAW}/original" "kitti"

    # 3b. Downsampled (reduced-kitti)
    log "--- Downsampled (reduced-kitti) ---"
    run_batch_eval "downsampled" \
        "${KITTI_REDUCED}" "${KITTI_REDUCED}" "${KITTI_REDUCED}" \
        "${OFFLINE_RAW}/downsampled" "reduced"

    # 3c. Each interpolated dataset
    log "--- Interpolated datasets ---"
    local configs
    configs=$(opc "ls '${KITTI_INTERP_BASE}' 2>/dev/null | sort" || true)
    while IFS= read -r cfg_name; do
        [[ -z "${cfg_name}" ]] && continue
        [[ -n "${SINGLE_CONFIG}" && "${cfg_name}" != "${SINGLE_CONFIG}" ]] && continue
        local lidar_root="${KITTI_INTERP_BASE}/${cfg_name}/velodyne"
        log "  Interpolated config: ${cfg_name}"
        run_batch_eval "interpolated" \
            "${KITTI_INTERP_BASE}" "${KITTI_INTERP_BASE}" "${lidar_root}" \
            "${OFFLINE_RAW}/interpolated" "${cfg_name}"
    done <<< "${configs}"
}

# ─── Step 4: Save per-frame detection bboxes ─────────────────────────────────
run_save_detections() {
    step "Step 4: Saving per-frame detection results (kitti_inspector compatible)"
    opc "mkdir -p '${OFFLINE_RAW}/detections'"

    local datasets=("original:${KITTI_ORIGINAL}" "downsampled:${KITTI_REDUCED}")

    # Add each interpolated config
    local configs
    configs=$(opc "ls '${KITTI_INTERP_BASE}' 2>/dev/null | sort" || true)
    while IFS= read -r cfg_name; do
        [[ -z "${cfg_name}" ]] && continue
        [[ -n "${SINGLE_CONFIG}" && "${cfg_name}" != "${SINGLE_CONFIG}" ]] && continue
        datasets+=("interp_${cfg_name}:${KITTI_INTERP_BASE}/${cfg_name}/velodyne")
    done <<< "${configs}"

    for entry in "${datasets[@]}"; do
        local label="${entry%%:*}"
        local bin_dir="${entry#*:}"

        for MODEL in $(printf '%s\n' "${!MODELS[@]}" | sort); do
            [[ -n "${SINGLE_MODEL}" && "${MODEL}" != "${SINGLE_MODEL}" ]] && continue
            read -r CFG CKPT <<< "${MODELS[$MODEL]}"
            local out="${OFFLINE_RAW}/detections/${label}/${MODEL}"
            log "  save_detections: ${label} / ${MODEL}"
            opc "python3 ${OPC_BATCH}/save_detections.py \
                    --cfg_file  ${OPC_TOOLS}/cfgs/kitti_models/${CFG%.*}_det.yaml \
                    --ckpt      ${OPC_DATA}/pretrained-models/${CKPT} \
                    --bin-dir   '${bin_dir}' \
                    --output-dir '${out}'" || \
            # Fallback: try without _det suffix (use same cfg as eval)
            opc "python3 ${OPC_BATCH}/save_detections.py \
                    --cfg_file  ${OPC_TOOLS}/cfgs/kitti_models/${CFG} \
                    --ckpt      ${OPC_DATA}/pretrained-models/${CKPT} \
                    --bin-dir   '${bin_dir}' \
                    --output-dir '${out}'" || \
            log "  WARNING: save_detections failed for ${label}/${MODEL}"
        done
    done
}

# ─── Step 5: Online (real-time) evaluation ────────────────────────────────────
run_online_eval() {
    step "Step 5: Online (real-time) evaluation"
    opc "mkdir -p '${ONLINE_RAW}'"

    local args="--output-dir ${ONLINE_RAW} \
                --rate ${PUBLISH_RATE} \
                --max-frames ${MAX_FRAMES} \
                --warmup ${WARMUP_FRAMES} \
                --data-type bin \
                --data-path ${KITTI_REDUCED}"
    [[ -n "${SINGLE_MODEL}"  ]] && args+=" --model ${SINGLE_MODEL}"
    [[ -n "${SINGLE_CONFIG}" ]] && args+=" --config ${SINGLE_CONFIG}"

    bash "$(dirname "$0")/src/lidar/tools/perf_evaluation/run_realtime_eval.sh" ${args}
}

# ─── Step 6: Generate LaTeX tables ───────────────────────────────────────────
run_latex() {
    step "Step 6: Generating LaTeX tables"
    opc "mkdir -p '${OFFLINE_LATEX}' '${ONLINE_LATEX}'"

    # Offline: one table per dataset type
    for variant in original downsampled; do
        local src="${OFFLINE_RAW}/${variant}"
        opc "python3 ${OPC_BATCH}/generate_latex_single_dataset.py '${src}'" \
            && opc "mv '${src}/latex/'*.tex '${OFFLINE_LATEX}/' 2>/dev/null || true"
    done

    # Offline: interpolated (multi-dataset table)
    opc "python3 ${OPC_BATCH}/generate_latex_tables.py '${OFFLINE_RAW}/interpolated'" \
        && opc "mv '${OFFLINE_RAW}/interpolated/latex/'*.tex '${OFFLINE_LATEX}/' 2>/dev/null || true"

    # Online: aggregate and copy results
    opc "python3 ${OPC_TOOLS}/perf_evaluation/aggregate_realtime_results.py '${ONLINE_RAW}'" \
        && opc "cp '${ONLINE_RAW}/'*.csv '${ONLINE_LATEX}/' 2>/dev/null || true"
}

# ─── Main ─────────────────────────────────────────────────────────────────────
echo "════════════════════════════════════════════════════════"
echo " Full pipeline  —  results → ${RESULTS_HOST}"
echo "════════════════════════════════════════════════════════"

check_storage

opc "mkdir -p '${OFFLINE_RAW}' '${ONLINE_RAW}' '${OFFLINE_LATEX}' '${ONLINE_LATEX}'"

${SKIP_DOWNSAMPLE} || run_downsample
${SKIP_INTERP}     || run_interpolation
${SKIP_OFFLINE}    || { run_offline_evals; run_save_detections; }
${SKIP_ONLINE}     || run_online_eval
run_latex

log "Pipeline complete. Results: ${RESULTS_HOST}"
