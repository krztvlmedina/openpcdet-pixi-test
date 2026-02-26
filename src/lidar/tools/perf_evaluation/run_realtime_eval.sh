#!/usr/bin/env bash
set -euo pipefail

# ─── Container Names ───────────────────────────────────────────────────────
CONTAINER_VELODYNE="${CONTAINER_VELODYNE:-velodyne_ros2}"
CONTAINER_OPENPCDET="${CONTAINER_OPENPCDET:-velodyne_openpcdet}"

# ─── Paths ────────────────────────────────────────────────────────────────
VEL_DATA_PATH="/app/data/ros2_bags/converted_bags"
VEL_INTERP_CONFIG_DIR="/app/data/config_files/interpolation/final"
OPC_ROOT="/OpenPCDet"
OPC_TOOLS="${OPC_ROOT}/src/lidar/tools"
OPC_MODELS_DIR="${OPC_ROOT}/data/pretrained-models"

# ─── Defaults ─────────────────────────────────────────────────────────────
OUTPUT_BASE="/OpenPCDet/output/perf_results"
WARMUP_FRAMES=10
MAX_FRAMES=500
PUBLISH_RATE=10.0
SINGLE_MODEL="pointrcnn_iou"
SINGLE_CONFIG="nearest_extreme"
SETTLE_TIME=5
POINTCLOUD_TOPIC="interpolated_point_cloud"
DATA_TYPE="auto"
VEL_BAG_TOPIC="/velodyne_points"

# ─── Model definitions ─────────────────────────────────────────────────────
declare -A MODELS
MODELS[parta2_anchor]="parta2_anchor.yaml PartA2_7940.pth"
MODELS[pointpillar]="pointpillar.yaml pointpillar_7728.pth"
MODELS[pointrcnn_iou]="pointrcnn_iou.yaml pointrcnn_iou_7875.pth"
MODELS[pv_rcnn]="pv_rcnn.yaml pv_rcnn_8369.pth"
MODELS[second]="second.yaml second_7862.pth"
MODELS[second_iou]="second_iou.yaml second_iou7909.pth"

declare -A INTERP_CONFIGS

# ─── Parse CLI arguments ───────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model) SINGLE_MODEL="$2"; shift 2 ;;
        --config) SINGLE_CONFIG="$2"; shift 2 ;;
        --data-path|--bin-dir) VEL_DATA_PATH="$2"; shift 2 ;;
        --output-dir) OUTPUT_BASE="$2"; shift 2 ;;
        --warmup) WARMUP_FRAMES="$2"; shift 2 ;;
        --max-frames) MAX_FRAMES="$2"; shift 2 ;;
        --rate) PUBLISH_RATE="$2"; shift 2 ;;
        --settle) SETTLE_TIME="$2"; shift 2 ;;
        --data-type) DATA_TYPE="$2"; shift 2 ;;
        --bag-topic) VEL_BAG_TOPIC="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ─── Setup and Discovery ──────────────────────────────────────────────────
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_DIR="${OUTPUT_BASE}/${TIMESTAMP}"

echo "Searching for interpolation configs..."
while IFS= read -r cfg_file; do
    cfg_name=$(basename "${cfg_file}" .yaml)
    INTERP_CONFIGS["${cfg_name}"]="${cfg_file}"
done < <(docker exec "${CONTAINER_VELODYNE}" bash -c "ls '${VEL_INTERP_CONFIG_DIR}'/*.yaml 2>/dev/null | sort")

echo "Found ${#INTERP_CONFIGS[@]} configs. Starting evaluation..."
docker exec "${CONTAINER_OPENPCDET}" mkdir -p "${RUN_DIR}"

# ─── Helpers ──────────────────────────────────────────────────────────────
DEXEC_PID=""
HOST_PIDS=()

dexec_bg() {
    local container="$1"; shift
    if [ "$container" = "$CONTAINER_VELODYNE" ]; then
        docker exec "$container" bash -c "pixi run \"$*\"" &
    else
        docker exec "$container" bash -c "source /opt/ros2_humble/install/setup.bash && $*" &
    fi
    DEXEC_PID=$!
}

kill_in_container() {
    local container="$1"; local pattern="$2"
    docker exec "$container" bash -c "pkill -15 -f '${pattern}' 2>/dev/null || true" || true
    sleep 1
    docker exec "$container" bash -c "pkill -9 -f '${pattern}' 2>/dev/null || true" || true
}

wait_for_topic() {
    local container="$1"; local topic="$2"; local timeout="${3:-30}"; local elapsed=0
    echo "  Waiting for topic ${topic}..."
    while ! docker exec "$container" bash -c "source /opt/ros2_humble/install/setup.bash && ros2 topic list 2>/dev/null" | grep -q "${topic}"; do
        sleep 1; elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then return 1; fi
    done
    return 0
}

# ─── Main Evaluation Runner ───────────────────────────────────────────────
run_evaluation() {
    local config_name="$1"; local config_file="$2"; local model_name="$3"; local cfg_file="$4"; local ckpt="$5"
    local run_output_dir="${RUN_DIR}/${config_name}/${model_name}"
    docker exec "${CONTAINER_OPENPCDET}" mkdir -p "${run_output_dir}"

    echo "------------------------------------------------------------"
    echo " CONFIG: ${config_name} | MODEL: ${model_name}"
    echo "------------------------------------------------------------"

    dexec_bg "${CONTAINER_OPENPCDET}" "python3 ${OPC_TOOLS}/perf_collector_node.py --ros-args -p output_dir:=${run_output_dir} -p config_name:=${config_name} -p model_name:=${model_name} -p warmup_frames:=${WARMUP_FRAMES} -p max_frames:=${MAX_FRAMES}"
    local collector_pid=${DEXEC_PID}; HOST_PIDS+=("${collector_pid}")

    dexec_bg "${CONTAINER_OPENPCDET}" "python3 ${OPC_TOOLS}/ros2_node.py --cfg_file ${cfg_file} --ckpt ${ckpt} --pointcloud_topic ${POINTCLOUD_TOPIC} --enable_perf_tracking --model_name ${model_name}"
    local detect_pid=${DEXEC_PID}; HOST_PIDS+=("${detect_pid}")
    sleep 5

    dexec_bg "${CONTAINER_VELODYNE}" "ros2 run dynamic_lidar_interpolation pointcloud_interpolation_node --ros-args --params-file ${config_file} -p performance.enable_perf_tracking:=true -p performance.config_name:=${config_name}"
    local interp_pid=${DEXEC_PID}; HOST_PIDS+=("${interp_pid}")
    
    wait_for_topic "${CONTAINER_OPENPCDET}" "${POINTCLOUD_TOPIC}" 30 || true

    if [ "${DATA_TYPE}" = "bin" ]; then
        dexec_bg "${CONTAINER_VELODYNE}" "ros2 run pointcloud_utils bin_publisher_node --ros-args -p bin_directory:=${VEL_DATA_PATH} -p publish_rate:=${PUBLISH_RATE} -p loop:=false -p topic:=velodyne_points"
    else
        dexec_bg "${CONTAINER_VELODYNE}" "ros2 bag play ${VEL_DATA_PATH} --topics ${VEL_BAG_TOPIC}"
    fi
    local publisher_pid=${DEXEC_PID}; HOST_PIDS+=("${publisher_pid}")

    echo "  Evaluation in progress..."
    if [ "${MAX_FRAMES}" -gt 0 ]; then
        local timeout_sec=$(awk "BEGIN{printf \"%d\", (${WARMUP_FRAMES}+${MAX_FRAMES})/${PUBLISH_RATE}+150}")
        local waited=0
        while kill -0 "${collector_pid}" 2>/dev/null; do
            sleep 2; waited=$((waited + 2))
            if [ "$waited" -ge "$timeout_sec" ]; then echo "  Timeout reached!"; break; fi
        done
    else
        wait "${publisher_pid}" 2>/dev/null || true; sleep 10
    fi

    echo "  Shutting down nodes..."
    {
        for pid in "${publisher_pid}" "${interp_pid}" "${detect_pid}" "${collector_pid}"; do
            kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true
        done
    } 2>/dev/null

    kill_in_container "${CONTAINER_VELODYNE}" "pointcloud_interpolation_node"
    kill_in_container "${CONTAINER_OPENPCDET}" "ros2_node.py"
    
    echo "  Cooling down (${SETTLE_TIME}s)..."
    HOST_PIDS=(); sleep "${SETTLE_TIME}"
}

# ─── Iteration Loop ───────────────────────────────────────────────────────
total_runs=$((${#INTERP_CONFIGS[@]} * ${#MODELS[@]}))
current_run=0

for CONFIG_NAME in $(printf '%s\n' "${!INTERP_CONFIGS[@]}" | sort); do
    if [[ -n "${SINGLE_CONFIG}" && "${CONFIG_NAME}" != "${SINGLE_CONFIG}" ]]; then continue; fi
    for MODEL in $(printf '%s\n' "${!MODELS[@]}" | sort); do
        if [[ -n "${SINGLE_MODEL}" && "${MODEL}" != "${SINGLE_MODEL}" ]]; then continue; fi
        current_run=$((current_run + 1))
        echo "=== [${current_run}/${total_runs}] Running Pipeline ==="
        read -r CFG CKPT <<< "${MODELS[$MODEL]}"
        run_evaluation "${CONFIG_NAME}" "${INTERP_CONFIGS[${CONFIG_NAME}]}" "${MODEL}" "src/lidar/tools/cfgs/kitti_models/interpolated/${CFG}" "data/pretrained-models/${CKPT}"
    done
done

echo "Evaluation Complete. Running aggregation..."
docker exec "${CONTAINER_OPENPCDET}" bash -c "source /opt/ros2_humble/install/setup.bash && cd ${OPC_ROOT} && python3 ${OPC_TOOLS}/perf_evaluation/aggregate_realtime_results.py ${RUN_DIR}"