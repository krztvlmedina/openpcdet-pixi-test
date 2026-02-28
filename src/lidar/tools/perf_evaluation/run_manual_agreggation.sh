CONTAINER_OPENPCDET="${CONTAINER_OPENPCDET:-velodyne_openpcdet}"

# ─── Paths ────────────────────────────────────────────────────────────────
OPC_ROOT="/OpenPCDet"
OPC_TOOLS="${OPC_ROOT}/src/lidar/tools"
OUTPUT_BASE="/OpenPCDet/output/perf_results"
RUN_DIR="${OUTPUT_BASE}/20260223_133853/"

echo "Running aggregation..."
docker exec "${CONTAINER_OPENPCDET}" bash -c "source /opt/ros2_humble/install/setup.bash && cd ${OPC_ROOT} && python3 ${OPC_TOOLS}/perf_evaluation/aggregate_realtime_results.py ${RUN_DIR}"