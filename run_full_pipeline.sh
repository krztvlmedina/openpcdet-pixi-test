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
#         detections/               ← save_detections.py output (per dataset/model)
#       desempeno_online/           ← run_realtime_eval.sh output
#     latex/
#       desempeno_offline/          ← AP + recall tables
#       desempeno_online/           ← aggregated realtime results
#
# Prerequisites:
#   • Docker containers ${CONTAINER_VELODYNE} and ${CONTAINER_OPENPCDET} running
#     and sharing the same ROS2 DDS domain (same network)
#   • KITTI original data mounted at ${OPC_DATA}/kitti  (images, calib, labels)
#   • Pretrained models at ${OPC_DATA}/pretrained-models
#   • Interpolation configs at ${VEL_INTERP_CONFIG_DIR} (velodyne container)
#
# Usage:
#   ./run_full_pipeline.sh [--skip-downsample] [--skip-interp] [--skip-offline]
#                          [--skip-online] [--single-model M] [--single-config C]
#                          [--rate HZ] [--max-frames N]
set -euo pipefail

# ─── Containers ──────────────────────────────────────────────────────────────
CONTAINER_VELODYNE="${CONTAINER_VELODYNE:-velodyne_ros2}"
CONTAINER_OPENPCDET="${CONTAINER_OPENPCDET:-velodyne_openpcdet}"

# ─── Paths (inside each container) ───────────────────────────────────────────
# openpcdet container
OPC_ROOT="/OpenPCDet"
OPC_DATA="${OPC_ROOT}/data"
OPC_TOOLS="${OPC_ROOT}/src/lidar/tools"
OPC_BATCH="${OPC_TOOLS}/batch_evaluation"

# velodyne container
VEL_SCRIPTS="/app/packages/pointcloud_utils/scripts"
VEL_INTERP_CONFIG_DIR="/app/data/config_files/interpolation/final"

# Shared data paths (mounted in both containers)
KITTI_ORIGINAL="${OPC_DATA}/kitti/training/velodyne"
KITTI_REDUCED="${OPC_DATA}/reduced-kitti"
KITTI_INTERP_BASE="${OPC_DATA}/interpolated-kitti"

# Minimum free space in GB required before starting
MIN_FREE_GB=50
SETTLE_TIME=5

# ─── Result root (on host, also mounted in openpcdet container) ───────────────
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RESULTS_HOST="$(pwd)/results/${TIMESTAMP}"
RESULTS_OPC="${OPC_ROOT}/results/${TIMESTAMP}"

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

# ─── Helpers (same pattern as run_realtime_eval.sh) ───────────────────────────
log()  { echo "[$(date +%H:%M:%S)] $*"; }
step() { echo; echo "════════════════════════════════════════════════════════"; \
         echo " $*"; echo "════════════════════════════════════════════════════════"; }

DEXEC_PID=""
HOST_PIDS=()

# Route background docker exec to the right container command prefix:
#   velodyne   → pixi run
#   openpcdet  → source ROS2 setup
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
    while ! docker exec "$container" bash -c \
            "source /opt/ros2_humble/install/setup.bash && ros2 topic list 2>/dev/null" \
            | grep -q "${topic}"; do
        sleep 1; elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then return 1; fi
    done
    return 0
}

# ─── Step 0: Storage check ───────────────────────────────────────────────────
check_storage() {
    log "Checking available storage..."
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
# Runs in openpcdet container (blocking); script path is mounted from velodyne app.
run_downsample() {
    step "Step 1: Downsampling KITTI 64-beam → 16-beam (VLP-16 simulation)"
    docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "cd ${OPC_ROOT} && python3 ${VEL_SCRIPTS}/downsample_64_to_16.py \
            ${KITTI_ORIGINAL} ${KITTI_REDUCED} --batch"
    log "Downsampled dataset written to ${KITTI_REDUCED}"
}

# ─── Step 2: Generate interpolated datasets ───────────────────────────────────
# One interpolated dataset per config, using the same 3-node pattern as
# run_evaluation() in run_realtime_eval.sh:
#   1. pointcloud_interpolation_node  (velodyne, background)
#   2. save_interpolated_clouds.py    (openpcdet, background — ROS2 subscriber)
#   3. bin_publisher_node             (velodyne, background — plays all frames once)
run_one_interpolation() {
    local cfg_file="$1"
    local cfg_name="$2"
    local out_velodyne="${KITTI_INTERP_BASE}/${cfg_name}/velodyne"

    echo "------------------------------------------------------------"
    echo " CONFIG: ${cfg_name}"
    echo "------------------------------------------------------------"

    docker exec "${CONTAINER_OPENPCDET}" bash -c "mkdir -p '${out_velodyne}'"

    # 1. Start interpolation node in velodyne container
    dexec_bg "${CONTAINER_VELODYNE}" \
        "ros2 run dynamic_lidar_interpolation pointcloud_interpolation_node \
         --ros-args --params-file '${cfg_file}'"
    local interp_pid=${DEXEC_PID}; HOST_PIDS+=("${interp_pid}")

    # 2. Wait for interpolated topic to be visible from openpcdet container,
    #    then start the cloud saver (ROS2 subscriber + file writer)
    wait_for_topic "${CONTAINER_OPENPCDET}" "interpolated_point_cloud" 30 || true

    dexec_bg "${CONTAINER_OPENPCDET}" \
        "python3 ${VEL_SCRIPTS}/save_interpolated_clouds.py \
            --input-dir  ${KITTI_REDUCED} \
            --output-dir ${out_velodyne} \
            --topic      interpolated_point_cloud \
            --timeout    60"
    local saver_pid=${DEXEC_PID}; HOST_PIDS+=("${saver_pid}")

    # 3. Publish reduced frames at 5 Hz, no loop (publisher exits when done)
    dexec_bg "${CONTAINER_VELODYNE}" \
        "ros2 run pointcloud_utils bin_publisher_node \
         --ros-args -p bin_directory:=${KITTI_REDUCED} \
                    -p publish_rate:=5.0 \
                    -p loop:=false \
                    -p topic:=velodyne_points"
    local publisher_pid=${DEXEC_PID}; HOST_PIDS+=("${publisher_pid}")

    # Wait for cloud saver to finish (exits via rclpy.shutdown() after all frames saved)
    local total_frames
    total_frames=$(docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "ls '${KITTI_REDUCED}'/*.bin 2>/dev/null | wc -l" || echo "0")
    local timeout_sec=$(( total_frames + 120 ))
    local waited=0
    echo "  Saving interpolated frames (expecting ~${total_frames})..."
    while kill -0 "${saver_pid}" 2>/dev/null; do
        sleep 2; waited=$((waited + 2))
        if [ "$waited" -ge "$timeout_sec" ]; then
            log "  WARNING: cloud saver timed out after ${waited}s"; break
        fi
    done

    echo "  Shutting down interpolation nodes..."
    {
        for pid in "${publisher_pid}" "${interp_pid}" "${saver_pid}"; do
            kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true
        done
    } 2>/dev/null
    kill_in_container "${CONTAINER_VELODYNE}"  "pointcloud_interpolation_node"
    kill_in_container "${CONTAINER_VELODYNE}"  "bin_publisher_node"
    kill_in_container "${CONTAINER_OPENPCDET}" "save_interpolated_clouds"

    HOST_PIDS=()
    log "  Cooling down (${SETTLE_TIME}s)..."
    sleep "${SETTLE_TIME}"
    log "Done: ${cfg_name}"
}

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
        [[ -z "${cfg_file}" ]] && continue
        local cfg_name
        cfg_name=$(basename "${cfg_file}" .yaml)
        [[ -n "${SINGLE_CONFIG}" && "${cfg_name}" != "${SINGLE_CONFIG}" ]] && continue
        run_one_interpolation "${cfg_file}" "${cfg_name}"
    done <<< "${configs}"
}

# ─── Step 3: Offline batch evaluations ───────────────────────────────────────
# All evaluation runs are blocking (no ROS2 nodes, pure GPU inference).
# Runs in openpcdet container.
run_batch_eval() {
    # Args: cfg_subdir  data_root  img_root  lidar_root  dest_dir  dataset_label
    local cfg_subdir="$1" data_root="$2" img_root="$3" lidar_root="$4" dest_dir="$5"
    local dataset_label="${6:-$(basename "${lidar_root}")}"

    docker exec "${CONTAINER_OPENPCDET}" bash -c "mkdir -p '${dest_dir}'"

    for MODEL in $(printf '%s\n' "${!MODELS[@]}" | sort); do
        [[ -n "${SINGLE_MODEL}" && "${MODEL}" != "${SINGLE_MODEL}" ]] && continue
        read -r CFG CKPT <<< "${MODELS[$MODEL]}"
        log "  [${dataset_label}] Model: ${MODEL}"
        docker exec "${CONTAINER_OPENPCDET}" bash -c \
            "cd ${OPC_ROOT} && \
             python3 ${OPC_TOOLS}/setupandruntest.py \
                --dataset-root '${data_root}' \
                --image-set-root '${img_root}' \
                --lidar-root '${lidar_root}' \
                --cfg_file ${OPC_TOOLS}/cfgs/kitti_models/${cfg_subdir}/${CFG} \
                --batch_size 1 \
                --ckpt ${OPC_DATA}/pretrained-models/${CKPT}"
        # Move output from OpenPCDet's default location to results directory
        local src="${OPC_ROOT}/output/src/lidar/tools/cfgs/kitti_models/${cfg_subdir}/${MODEL}"
        docker exec "${CONTAINER_OPENPCDET}" bash -c \
            "mkdir -p '${dest_dir}/${dataset_label}' && \
             mv '${src}' '${dest_dir}/${dataset_label}/${MODEL}' 2>/dev/null || true"
    done
}

run_offline_evals() {
    step "Step 3: Offline batch evaluations"
    docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "mkdir -p '${OFFLINE_RAW}/original' '${OFFLINE_RAW}/downsampled' \
                   '${OFFLINE_RAW}/interpolated'"

    log "--- Original KITTI ---"
    run_batch_eval "original" \
        "${OPC_DATA}/kitti" "${KITTI_ORIGINAL}" "${KITTI_ORIGINAL}" \
        "${OFFLINE_RAW}/original" "kitti"

    log "--- Downsampled (reduced-kitti) ---"
    run_batch_eval "downsampled" \
        "${KITTI_REDUCED}" "${KITTI_REDUCED}" "${KITTI_REDUCED}" \
        "${OFFLINE_RAW}/downsampled" "reduced"

    log "--- Interpolated datasets ---"
    local configs
    configs=$(docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "ls '${KITTI_INTERP_BASE}' 2>/dev/null | sort" || true)
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
# Runs in openpcdet container (blocking, no ROS2).
run_save_detections() {
    step "Step 4: Saving per-frame detection results (kitti_inspector compatible)"
    docker exec "${CONTAINER_OPENPCDET}" bash -c "mkdir -p '${OFFLINE_RAW}/detections'"

    local datasets=("original:${KITTI_ORIGINAL}" "downsampled:${KITTI_REDUCED}")

    local configs
    configs=$(docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "ls '${KITTI_INTERP_BASE}' 2>/dev/null | sort" || true)
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
            docker exec "${CONTAINER_OPENPCDET}" bash -c \
                "cd ${OPC_ROOT} && \
                 python3 ${OPC_BATCH}/save_detections.py \
                    --cfg_file  ${OPC_TOOLS}/cfgs/kitti_models/${CFG} \
                    --ckpt      ${OPC_DATA}/pretrained-models/${CKPT} \
                    --bin-dir   '${bin_dir}' \
                    --output-dir '${out}'" || \
            log "  WARNING: save_detections failed for ${label}/${MODEL}"
        done
    done
}

# ─── Step 5: Online (real-time) evaluation ────────────────────────────────────
# Delegates to run_realtime_eval.sh which manages its own container orchestration.
run_online_eval() {
    step "Step 5: Online (real-time) evaluation"
    docker exec "${CONTAINER_OPENPCDET}" bash -c "mkdir -p '${ONLINE_RAW}'"

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
# All LaTeX generation runs in openpcdet container (blocking Python scripts).
run_latex() {
    step "Step 6: Generating LaTeX tables"
    docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "mkdir -p '${OFFLINE_LATEX}' '${ONLINE_LATEX}'"

    # Offline: one table per single-dataset variant
    for variant in original downsampled; do
        local src="${OFFLINE_RAW}/${variant}"
        docker exec "${CONTAINER_OPENPCDET}" bash -c \
            "cd ${OPC_ROOT} && \
             python3 ${OPC_BATCH}/generate_latex_single_dataset.py '${src}' && \
             mv '${src}/latex/'*.tex '${OFFLINE_LATEX}/' 2>/dev/null || true"
    done

    # Offline: interpolated multi-dataset comparison table
    docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "cd ${OPC_ROOT} && \
         python3 ${OPC_BATCH}/generate_latex_tables.py '${OFFLINE_RAW}/interpolated' && \
         mv '${OFFLINE_RAW}/interpolated/latex/'*.tex '${OFFLINE_LATEX}/' 2>/dev/null || true"

    # Online: aggregate CSV results
    docker exec "${CONTAINER_OPENPCDET}" bash -c \
        "cd ${OPC_ROOT} && \
         python3 ${OPC_TOOLS}/perf_evaluation/aggregate_realtime_results.py '${ONLINE_RAW}' && \
         cp '${ONLINE_RAW}/'*.csv '${ONLINE_LATEX}/' 2>/dev/null || true"
}

# ─── Main ─────────────────────────────────────────────────────────────────────
echo "════════════════════════════════════════════════════════"
echo " Full pipeline  —  results → ${RESULTS_HOST}"
echo "════════════════════════════════════════════════════════"

check_storage

docker exec "${CONTAINER_OPENPCDET}" bash -c \
    "mkdir -p '${OFFLINE_RAW}' '${ONLINE_RAW}' '${OFFLINE_LATEX}' '${ONLINE_LATEX}'"

${SKIP_DOWNSAMPLE} || run_downsample
${SKIP_INTERP}     || run_interpolation
${SKIP_OFFLINE}    || { run_offline_evals; run_save_detections; }
${SKIP_ONLINE}     || run_online_eval
run_latex

log "Pipeline complete. Results: ${RESULTS_HOST}"
