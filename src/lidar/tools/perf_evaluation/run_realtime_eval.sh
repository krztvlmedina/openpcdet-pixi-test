#!/usr/bin/env bash
# Real-Time Performance Evaluation Pipeline
#
# Evaluates end-to-end latency of the interpolation + detection pipeline
# for all 6 detection models across interpolation configurations.
#
# Architecture:
#   velodyne_ros container: bin_publisher + interpolation_node (perf tracking)
#   openpcdet-prebuilt container: pcdet_node (perf tracking) + perf_collector_node
#
# Usage:
#   # Run full evaluation (all models, all configs):
#   bash src/lidar/tools/perf_evaluation/run_realtime_eval.sh
#
#   # Run single model with specific config:
#   bash src/lidar/tools/perf_evaluation/run_realtime_eval.sh --model parta2_anchor --config optimized
#
#   # Custom output directory and frame count:
#   bash src/lidar/tools/perf_evaluation/run_realtime_eval.sh --output-dir /data/my_results --max-frames 100

set -euo pipefail

# ─── Defaults ───────────────────────────────────────────────────────────────
OPENPCDET_ROOT="${OPENPCDET_ROOT:-/OpenPCDet}"
BIN_DIR="${BIN_DIR:-${OPENPCDET_ROOT}/data/kitti/training/velodyne}"
OUTPUT_BASE="${OUTPUT_BASE:-${OPENPCDET_ROOT}/data/perf_results}"
WARMUP_FRAMES=10
MAX_FRAMES=500
PUBLISH_RATE=10.0
SINGLE_MODEL=""
SINGLE_CONFIG=""
SETTLE_TIME=5       # Seconds to wait between runs for GPU/DDS cooldown
POINTCLOUD_TOPIC="interpolated_point_cloud"

# ─── Model definitions (matches eval_all.sh) ───────────────────────────────
declare -A MODELS
MODELS[parta2_anchor]="parta2_anchor.yaml PartA2_7940.pth"
MODELS[pointpillar]="pointpillar.yaml pointpillar_7728.pth"
MODELS[pointrcnn_iou]="pointrcnn_iou.yaml pointrcnn_iou_7875.pth"
MODELS[pv_rcnn]="pv_rcnn.yaml pv_rcnn_8369.pth"
MODELS[second]="second.yaml second_7862.pth"
MODELS[second_iou]="second_iou.yaml second_iou7909.pth"

# Interpolation configs to test
declare -A INTERP_CONFIGS
INTERP_CONFIGS[standard]="${OPENPCDET_ROOT}/packages/dynamic_lidar_interpolation/config/interpolation_config.yaml"
INTERP_CONFIGS[optimized]="${OPENPCDET_ROOT}/packages/dynamic_lidar_interpolation/config/interpolation_config_optimized.yaml"

# ─── Parse CLI arguments ───────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)       SINGLE_MODEL="$2"; shift 2 ;;
        --config)      SINGLE_CONFIG="$2"; shift 2 ;;
        --bin-dir)     BIN_DIR="$2"; shift 2 ;;
        --output-dir)  OUTPUT_BASE="$2"; shift 2 ;;
        --warmup)      WARMUP_FRAMES="$2"; shift 2 ;;
        --max-frames)  MAX_FRAMES="$2"; shift 2 ;;
        --rate)        PUBLISH_RATE="$2"; shift 2 ;;
        --settle)      SETTLE_TIME="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --model NAME       Run only this model (e.g., parta2_anchor)"
            echo "  --config NAME      Run only this config (standard or optimized)"
            echo "  --bin-dir DIR      Directory with .bin files"
            echo "  --output-dir DIR   Output directory for results"
            echo "  --warmup N         Warmup frames (default: 10)"
            echo "  --max-frames N     Max frames to record (default: 500, 0=unlimited)"
            echo "  --rate HZ          Publish rate (default: 10.0)"
            echo "  --settle SEC       Settle time between runs (default: 5)"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_DIR="${OUTPUT_BASE}/${TIMESTAMP}"
mkdir -p "${RUN_DIR}"

echo "================================================================"
echo "  Real-Time Performance Evaluation Pipeline"
echo "================================================================"
echo "  Timestamp:     ${TIMESTAMP}"
echo "  Output:        ${RUN_DIR}"
echo "  Bin directory: ${BIN_DIR}"
echo "  Warmup:        ${WARMUP_FRAMES} frames"
echo "  Max frames:    ${MAX_FRAMES}"
echo "  Publish rate:  ${PUBLISH_RATE} Hz"
echo "================================================================"
echo ""

# ─── Helper: kill background processes on exit ─────────────────────────────
PIDS=()
cleanup() {
    echo ""
    echo "Cleaning up background processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    PIDS=()
}
trap cleanup EXIT INT TERM

# ─── Helper: wait for a topic to become available ──────────────────────────
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-30}"
    local elapsed=0
    echo "  Waiting for topic ${topic} (timeout: ${timeout}s)..."
    while ! ros2 topic list 2>/dev/null | grep -q "${topic}"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "  WARNING: Timeout waiting for topic ${topic}"
            return 1
        fi
    done
    echo "  Topic ${topic} is available."
    return 0
}

# ─── Run a single evaluation ───────────────────────────────────────────────
run_evaluation() {
    local config_name="$1"
    local config_file="$2"
    local model_name="$3"
    local cfg_file="$4"
    local ckpt="$5"
    local run_output_dir="${RUN_DIR}/${config_name}/${model_name}"

    mkdir -p "${run_output_dir}"

    echo ""
    echo "────────────────────────────────────────────────────────────"
    echo "  Config: ${config_name}  |  Model: ${model_name}"
    echo "  Output: ${run_output_dir}"
    echo "────────────────────────────────────────────────────────────"

    # 1. Start performance collector (must be ready before data flows)
    echo "  Starting perf_collector_node..."
    python3 "${OPENPCDET_ROOT}/src/lidar/tools/perf_collector_node.py" \
        --ros-args \
        -p output_dir:="${run_output_dir}" \
        -p config_name:="${config_name}" \
        -p model_name:="${model_name}" \
        -p warmup_frames:="${WARMUP_FRAMES}" \
        -p max_frames:="${MAX_FRAMES}" &
    PIDS+=($!)
    local collector_pid=$!
    sleep 2

    # 2. Start detection node
    echo "  Starting pcdet_node (model: ${model_name})..."
    python3 "${OPENPCDET_ROOT}/src/lidar/tools/ros2_node.py" \
        --cfg_file "${cfg_file}" \
        --ckpt "${ckpt}" \
        --pointcloud_topic "${POINTCLOUD_TOPIC}" \
        --enable_perf_tracking \
        --model_name "${model_name}" &
    PIDS+=($!)
    local detect_pid=$!
    sleep 5  # Wait for model loading

    # 3. Start interpolation node (this will be in the velodyne_ros container
    #    in production, but can run locally for testing)
    echo "  Starting interpolation node..."
    ros2 run dynamic_lidar_interpolation pointcloud_interpolation_node \
        --ros-args \
        --params-file "${config_file}" \
        -p performance.enable_perf_tracking:=true \
        -p "performance.config_name:=${config_name}" &
    PIDS+=($!)
    local interp_pid=$!
    sleep 2

    # 4. Wait for topics to be ready
    wait_for_topic "${POINTCLOUD_TOPIC}" 30 || true
    wait_for_topic "/perf/interpolation" 10 || true
    wait_for_topic "/perf/detection" 10 || true

    # 5. Start bin_publisher (this triggers the pipeline)
    echo "  Starting bin_publisher (rate: ${PUBLISH_RATE} Hz)..."
    ros2 run pointcloud_utils bin_publisher_node \
        --ros-args \
        -p bin_directory:="${BIN_DIR}" \
        -p publish_rate:="${PUBLISH_RATE}" \
        -p loop:=false \
        -p frame_id:=velodyne \
        -p topic:=velodyne_points &
    PIDS+=($!)
    local publisher_pid=$!

    # 6. Wait for collector to finish (it shuts down after max_frames)
    echo "  Evaluation running... waiting for collector to finish."
    if [ "${MAX_FRAMES}" -gt 0 ]; then
        # Wait for collector with timeout
        local timeout_sec=$(( (WARMUP_FRAMES + MAX_FRAMES) * 3 + 60 ))
        local waited=0
        while kill -0 "$collector_pid" 2>/dev/null; do
            sleep 1
            waited=$((waited + 1))
            if [ "$waited" -ge "$timeout_sec" ]; then
                echo "  WARNING: Collector timeout after ${timeout_sec}s, stopping."
                break
            fi
        done
    else
        # Unlimited: wait for publisher to finish (no loop), then give extra time
        wait "$publisher_pid" 2>/dev/null || true
        echo "  Publisher finished. Waiting 10s for remaining frames..."
        sleep 10
    fi

    # 7. Stop all processes for this run
    echo "  Stopping nodes..."
    for pid in "$publisher_pid" "$interp_pid" "$detect_pid" "$collector_pid"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    # Remove from PIDS array
    PIDS=()

    echo "  Run complete: ${config_name}/${model_name}"

    # Settle time between runs
    echo "  Cooling down for ${SETTLE_TIME}s..."
    sleep "${SETTLE_TIME}"
}

# ─── Main evaluation loop ─────────────────────────────────────────────────
total_runs=0
completed_runs=0

# Count total runs
for CONFIG_NAME in "${!INTERP_CONFIGS[@]}"; do
    if [[ -n "${SINGLE_CONFIG}" && "${CONFIG_NAME}" != "${SINGLE_CONFIG}" ]]; then
        continue
    fi
    for MODEL in "${!MODELS[@]}"; do
        if [[ -n "${SINGLE_MODEL}" && "${MODEL}" != "${SINGLE_MODEL}" ]]; then
            continue
        fi
        total_runs=$((total_runs + 1))
    done
done

echo "Total evaluation runs: ${total_runs}"
echo ""

for CONFIG_NAME in "${!INTERP_CONFIGS[@]}"; do
    if [[ -n "${SINGLE_CONFIG}" && "${CONFIG_NAME}" != "${SINGLE_CONFIG}" ]]; then
        continue
    fi

    CONFIG_FILE="${INTERP_CONFIGS[${CONFIG_NAME}]}"
    echo "============================================================"
    echo "  Interpolation config: ${CONFIG_NAME}"
    echo "  Config file: ${CONFIG_FILE}"
    echo "============================================================"

    for MODEL in "${!MODELS[@]}"; do
        if [[ -n "${SINGLE_MODEL}" && "${MODEL}" != "${SINGLE_MODEL}" ]]; then
            continue
        fi

        read -r CFG CKPT <<< "${MODELS[$MODEL]}"
        CFG_FILE="src/lidar/tools/cfgs/kitti_models/interpolated/${CFG}"
        CKPT_FILE="data/pretrained-models/${CKPT}"

        completed_runs=$((completed_runs + 1))
        echo ""
        echo "[${completed_runs}/${total_runs}] Running: ${CONFIG_NAME} / ${MODEL}"

        run_evaluation \
            "${CONFIG_NAME}" \
            "${CONFIG_FILE}" \
            "${MODEL}" \
            "${CFG_FILE}" \
            "${CKPT_FILE}"
    done
done

echo ""
echo "================================================================"
echo "  All evaluations complete!"
echo "  Results stored in: ${RUN_DIR}"
echo "================================================================"

# Run aggregation if the script exists
AGG_SCRIPT="${OPENPCDET_ROOT}/src/lidar/tools/perf_evaluation/aggregate_realtime_results.py"
if [[ -f "${AGG_SCRIPT}" ]]; then
    echo ""
    echo "Running results aggregation..."
    python3 "${AGG_SCRIPT}" "${RUN_DIR}"
fi
