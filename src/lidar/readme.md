av2/utils/typing.py: 
    La siguiente linea falla:
    NDArrayNumber = np.ndarray[Any, np.dtype[Union[np.integer[Any], np.floating[Any]]]]
pero se arreglar agregando comillas al valor asignado 
    NDArrayNumber = "np.ndarray[Any, np.dtype[Union[np.integer[Any], np.floating[Any]]]]"

No obstante, esto ya está considerado en el Dockerfile del contenedor.


PointRCNN:

python3 src/lidar/tools/ros2_node.py --cfg_file src/lidar/tools/cfgs/kitti_models/custom_pointrcnn_iou.yaml --ckpt data/pretrained-models/pointrcnn_iou_7875.pth --pointcloud_topic \interpolated_point_cloud

PartA2:

python3 src/lidar/tools/ros2_node.py --cfg_file src/lidar/tools/cfgs/kitti_models/parta2_anchor.yaml --ckpt data/pretrained-models/PartA2_7940.pth --pointcloud_topic \interpolated_point_cloud


Testing:

python3 -m pcdet.datasets.kitti.kitti_dataset create_kitti_infos tools/cfgs/dataset_configs/kitti_dataset.yaml

python3 tools/test.py --cfg_file ${CONFIG_FILE} --batch_size ${BATCH_SIZE} --ckpt ${CKPT}

python3 tools/test.py --cfg_file src/lidar/tools/cfgs/kitti_models/custom_pointrcnn_iou.yaml --batch_size 1 --ckpt data/pretrained-models/pointrcnn_iou_7875.pth 