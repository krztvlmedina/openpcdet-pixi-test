# OpenPCDet ROS2 Pipeline Optimization

## Build & Run

```bash
# Build both original and optimized nodes
colcon build --packages-select dynamic_lidar_interpolation
call install\setup.bat

# Launch optimized version
ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_optimized_launch.py \
  config_file:=your_config.yaml
```

## Files Created

### Optimized Node
- `packages/dynamic_lidar_interpolation/src/pointcloud_interpolation_node_optimized.cpp`
- Hardcoded depth=1, frame dropping, performance stats

### Optimized Launch
- `packages/dynamic_lidar_interpolation/launch/pointcloud_interpolation_optimized_launch.py`
- Uses `pointcloud_interpolation_node_optimized` executable

### Optimized Config
- `packages/dynamic_lidar_interpolation/config/interpolation_config_optimized.yaml`
- `method: "nearest"`, `angular_resolution_x: 0.5`, `filter_pc: false`