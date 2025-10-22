av2/utils/typing.py: 
    La siguiente linea falla:
    NDArrayNumber = np.ndarray[Any, np.dtype[Union[np.integer[Any], np.floating[Any]]]]
pero se arreglar agregando comillas al valor asignado 
    NDArrayNumber = "np.ndarray[Any, np.dtype[Union[np.integer[Any], np.floating[Any]]]]"



python3 src/lidar/tools/ros2_node.py 
--cfg_file src/lidar/tools/cfgs/kitti_models/custom_pointrcnn_iou.yaml --ckpt data/pretrained-models/pointrcnn_iou_7875.pth --pointcloud_topic \interpolated_point_cloud