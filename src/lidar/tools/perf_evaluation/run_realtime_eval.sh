#!/usr/bin/env bash
# Real-Time Performance Evaluation Pipeline
#
# Evaluates end-to-end latency of the interpolation + detection pipeline
# for all 6 detection models across interpolation configurations.
#
# Architecture (two Docker containers, both network_mode: host + FastDDS UDPv4):
#
#   velodyne_ros2 container:
#     - bin_publisher_node    → publishes KITTI .bin as velodyne_points
#     - interpolation_node    → subscribes velodyne_points, publishes interpolated_point_cloud
#     - publishes /perf/interpolation
#
#   velodyne_openpcdet container:
#     - pcdet_node            → subscribes interpolated_point_cloud, runs inference
#     - perf_collector_node   → subscribes /perf/interpolation + /perf/detection
#     - publishes /perf/detection
#
# This script runs on the HOST and orchestrates both containers via docker exec.

set -euo pipefail

# ─── Container names (from compose.yml) ────────────────────────────────────
CONTAINER_VELODYNE="${CONTAINER_VELODYNE:-velodyne_ros2}"
CONTAINER_OPENPCDET="${CONTAINER_OPENPCDET:-velodyne_openpcdet}"

# ─── Paths inside each container ───────────────────────────────────────────
VEL_BIN_DIR="/app/data/ros2_bags"
VEL_INTERP_CONFIG_DIR="/app/data/config_files/interpolation"

OPC_ROOT="/OpenPCDet"
OPC_TOOLS="${OPC_ROOT}/src/lidar/tools"
OPC_MODELS_DIR="${OPC_ROOT}/data/pretrained-models"

# ─── Host-side defaults ────────────────────────────────────────────────────
OUTPUT_BASE="/OpenPCDet/output/perf_results"
WARMUP_FRAMES=10
MAX_FRAMES=500
PUBLISH_RATE=10.0
SINGLE_MODEL=""
SINGLE_CONFIG=""
SETTLE_TIME=5
POINTCLOUD_TOPIC="interpolated_point_cloud"

# ─── Model definitions ─────────────────────────────────────────────────────
declare -A MODELS
MODELS[parta2_anchor]="parta2_anchor.yaml PartA2_7940.pth"
MODELS[pointpillar]="pointpillar.yaml pointpillar_7728.pth"
MODELS[pointrcnn_iou]="pointrcnn_iou.yaml pointrcnn_iou_7875.pth"
MODELS[pv_rcnn]="pv_rcnn.yaml pv_rcnn_8369.pth"
MODELS[second]="second.yaml second_7862.pth"
MODELS[second_iou]="second_iou.yaml second_iou7909.pth"

declare -A INTERP_CONFIGS
INTERP_CONFIGS[standard]="${VEL_INTERP_CONFIG_DIR}/interpolation_config.yaml"
INTERP_CONFIGS[optimized]="${VEL_INTERP_CONFIG_DIR}/interpolation_config_optimized.yaml"

# ─── Parse CLI arguments ───────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)       SINGLE_MODEL="$2"; shift 2 ;;
        --config)      SINGLE_CONFIG="$2"; shift 2 ;;
        --bin-dir)     VEL_BIN_DIR="$2"; shift 2 ;;
        --output-dir)  OUTPUT_BASE="$2"; shift 2 ;;
        --warmup)      WARMUP_FRAMES="$2"; shift 2 ;;
        --max-frames)  MAX_FRAMES="$2"; shift 2 ;;
        --rate)        PUBLISH_RATE="$2"; shift 2 ;;
        --settle)      SETTLE_TIME="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_DIR="${OUTPUT_BASE}/${TIMESTAMP}"

echo "================================================================"
echo "  Real-Time Performance Evaluation Pipeline"
echo "================================================================"
echo "  Timestamp:         ${TIMESTAMP}"
echo "  Output (openpcdet):${RUN_DIR}"
echo "  Bin dir (vel_ros): ${VEL_BIN_DIR}"
echo "  Warmup:            ${WARMUP_FRAMES} frames"
echo "  Max frames:        ${MAX_FRAMES}"
echo "  Publish rate:      ${PUBLISH_RATE} Hz"
echo "  Container (vel):   ${CONTAINER_VELODYNE}"
echo "  Container (opc):   ${CONTAINER_OPENPCDET}"
echo "================================================================"
echo ""

# ─── Verify containers are running ────────────────────────────────────────
for ctr in "${CONTAINER_VELODYNE}" "${CONTAINER_OPENPCDET}"; do
    if ! docker inspect --format='{{.State.Running}}' "${ctr}" 2>/dev/null | grep -q true; then
        echo "ERROR: Container '${ctr}' is not running."
        echo "Start with:  docker compose up -d"
        exit 1
    fi
done
echo "Both containers are running."

docker exec "${CONTAINER_OPENPCDET}" mkdir -p "${RUN_DIR}"

# ─── Environment-aware docker exec helpers ─────────────────────────────────

dexec_bg() {
    local container="$1"
    shift

    if [ "$container" = "$CONTAINER_VELODYNE" ]; then
        docker exec "$container" bash -c "pixi shell -c \"$*\"" &
    else
        docker exec "$container" bash -c "source /opt/ros2_humble/install/setup.bash && $*" &
    fi

    echo $!
}

kill_in_container() {
    local container="$1"
    local pattern="$2"

    if [ "$container" = "$CONTAINER_VELODYNE" ]; then
        docker exec "$container" bash -c "pixi shell -c \"pkill -f '${pattern}' 2>/dev/null || true\""
    else
        docker exec "$container" bash -c "source /opt/ros2_humble/install/setup.bash && pkill -f '${pattern}' 2>/dev/null || true"
    fi
}

wait_for_topic() {
    local container="$1"
    local topic="$2"
    local timeout="${3:-30}"
    local elapsed=0

    echo "  Waiting for topic ${topic} in ${container} (timeout: ${timeout}s)..."

    while true; do
        if [ "$container" = "$CONTAINER_VELODYNE" ]; then
            docker exec "$container" bash -c \
                "pixi shell -c \"ros2 topic list 2>/dev/null\"" | grep -q "${topic}" && break
        else
            docker exec "$container" bash -c \
                "source /opt/ros2_humble/install/setup.bash && ros2 topic list 2>/dev/null" | grep -q "${topic}" && break
        fi

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

# ─── Cleanup on exit ──────────────────────────────────────────────────────
HOST_PIDS=()
cleanup() {
    echo ""
    echo "Cleaning up..."
    for pid in "${HOST_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    HOST_PIDS=()

    kill_in_container "${CONTAINER_VELODYNE}" "pointcloud_interpolation_node"
    kill_in_container "${CONTAINER_VELODYNE}" "bin_publisher_node"
    kill_in_container "${CONTAINER_OPENPCDET}" "ros2_node.py"
    kill_in_container "${CONTAINER_OPENPCDET}" "perf_collector_node.py"
}
trap cleanup EXIT INT TERM

# ─── Run a single evaluation ───────────────────────────────────────────────
run_evaluation() {
    local config_name="$1"
    local config_file="$2"
    local model_name="$3"
    local cfg_file="$4"
    local ckpt="$5"
    local run_output_dir="${RUN_DIR}/${config_name}/${model_name}"

    docker exec "${CONTAINER_OPENPCDET}" mkdir -p "${run_output_dir}"

    echo ""
    echo "────────────────────────────────────────────────────────────"
    echo "  Config: ${config_name}  |  Model: ${model_name}"
    echo "  Output: ${run_output_dir}"
    echo "────────────────────────────────────────────────────────────"

    echo "  [openpcdet] Starting perf_collector_node..."
    local collector_pid
    collector_pid=$(dexec_bg "${CONTAINER_OPENPCDET}" \
        "cd ${OPC_ROOT} && python3 ${OPC_TOOLS}/perf_collector_node.py \
            --ros-args \
            -p output_dir:=${run_output_dir} \
            -p config_name:=${config_name} \
            -p model_name:=${model_name} \
            -p warmup_frames:=${WARMUP_FRAMES} \
            -p max_frames:=${MAX_FRAMES}")
    HOST_PIDS+=("${collector_pid}")
    sleep 2

    echo "  [openpcdet] Starting pcdet_node (model: ${model_name})..."
    local detect_pid
    detect_pid=$(dexec_bg "${CONTAINER_OPENPCDET}" \
        "cd ${OPC_ROOT} && python3 ${OPC_TOOLS}/ros2_node.py \
            --cfg_file ${cfg_file} \
            --ckpt ${ckpt} \
            --pointcloud_topic ${POINTCLOUD_TOPIC} \
            --enable_perf_tracking \
            --model_name ${model_name}")
    HOST_PIDS+=("${detect_pid}")
    sleep 8

    echo "  [velodyne_ros] Starting interpolation node..."
    local interp_pid
    interp_pid=$(dexec_bg "${CONTAINER_VELODYNE}" \
        "ros2 run dynamic_lidar_interpolation pointcloud_interpolation_node \
            --ros-args \
            --params-file ${config_file} \
            -p performance.enable_perf_tracking:=true \
            -p performance.config_name:=${config_name}")
    HOST_PIDS+=("${interp_pid}")
    sleep 2

    # Wait until the interpolation node has registered its output topic, which
    # confirms it started successfully and is ready to receive velodyne_points.
    # /perf/interpolation and /perf/detection are data-driven — they only appear
    # once frames flow through the pipeline, so we check them after the publisher
    # is running.
    wait_for_topic "${CONTAINER_OPENPCDET}" "${POINTCLOUD_TOPIC}" 30 || true

    echo "  [velodyne_ros] Starting bin_publisher (rate: ${PUBLISH_RATE} Hz)..."
    local publisher_pid
    publisher_pid=$(dexec_bg "${CONTAINER_VELODYNE}" \
        "ros2 run pointcloud_utils bin_publisher_node \
            --ros-args \
            -p bin_directory:=${VEL_BIN_DIR} \
            -p publish_rate:=${PUBLISH_RATE} \
            -p loop:=false \
            -p frame_id:=velodyne \
            -p topic:=velodyne_points")
    HOST_PIDS+=("${publisher_pid}")

    echo "  Evaluation running... waiting for collector to finish."

    if [ "${MAX_FRAMES}" -gt 0 ]; then
        # Timeout: allow (warmup+max) frames at the given publish rate, plus 120 s buffer.
        # Uses awk for float division since PUBLISH_RATE may be a decimal.
        local timeout_sec
        timeout_sec=$(awk "BEGIN{printf \"%d\", (${WARMUP_FRAMES}+${MAX_FRAMES})/${PUBLISH_RATE}+120}")
        local waited=0
        while kill -0 "${collector_pid}" 2>/dev/null; do
            sleep 1
            waited=$((waited + 1))
            if [ "$waited" -ge "$timeout_sec" ]; then
                echo "  WARNING: Collector timeout after ${timeout_sec}s, stopping."
                break
            fi
        done
    else
        wait "${publisher_pid}" 2>/dev/null || true
        echo "  Publisher finished. Waiting 10s for remaining frames..."
        sleep 10
    fi

    echo "  Stopping nodes..."

    for pid in "${publisher_pid}" "${interp_pid}" "${detect_pid}" "${collector_pid}"; do
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done

    kill_in_container "${CONTAINER_VELODYNE}" "bin_publisher_node"
    kill_in_container "${CONTAINER_VELODYNE}" "pointcloud_interpolation_node"
    kill_in_container "${CONTAINER_OPENPCDET}" "ros2_node.py"
    kill_in_container "${CONTAINER_OPENPCDET}" "perf_collector_node.py"

    HOST_PIDS=()

    echo "  Run complete: ${config_name}/${model_name}"
    echo "  Cooling down for ${SETTLE_TIME}s..."
    sleep "${SETTLE_TIME}"
}

# ─── Main loop ─────────────────────────────────────────────────────────────
total_runs=0
completed_runs=0

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
    echo "  Config file (in velodyne_ros2): ${CONFIG_FILE}"
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
echo "  Results stored in: ${RUN_DIR} (inside ${CONTAINER_OPENPCDET})"
echo "================================================================"

echo ""
echo "Running results aggregation..."
docker exec "${CONTAINER_OPENPCDET}" bash -c \
    "source /opt/ros2_humble/install/setup.bash && \
     cd ${OPC_ROOT} && \
     python3 ${OPC_TOOLS}/perf_evaluation/aggregate_realtime_results.py ${RUN_DIR}" \
    || echo "WARNING: Aggregation failed. Run manually inside the container."
