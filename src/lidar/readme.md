av2/utils/typing.py: 
    La siguiente linea falla:
    NDArrayNumber = np.ndarray[Any, np.dtype[Union[np.integer[Any], np.floating[Any]]]]
pero se puede arreglar agregando comillas al valor asignado 
    NDArrayNumber = "np.ndarray[Any, np.dtype[Union[np.integer[Any], np.floating[Any]]]]"

No obstante, esto ya está considerado en el Dockerfile del contenedor.

## Ejecución de modelos en tiempo real

PointRCNN:

python3 src/lidar/tools/ros2_node.py --cfg_file src/lidar/tools/cfgs/kitti_models/custom_pointrcnn_iou.yaml --ckpt data/pretrained-models/pointrcnn_iou_7875.pth --pointcloud_topic \interpolated_point_cloud

PartA2:

python3 src/lidar/tools/ros2_node.py --cfg_file src/lidar/tools/cfgs/kitti_models/parta2_anchor.yaml --ckpt data/pretrained-models/PartA2_7940.pth --pointcloud_topic \interpolated_point_cloud

Second_iou:

python3 src/lidar/tools/ros2_node.py --cfg_file src/lidar/tools/cfgs/kitti_models/interpolated/second_iou.yaml --ckpt data/pretrained-models/second_iou7909.pth --pointcloud_topic \interpolated_point_cloud

Para la creación de metadata de base de datos (ejecutando desde /OpenPCDet/):

- Original:
python3 -m pcdet.datasets.kitti.kitti_dataset create_kitti_infos tools/cfgs/dataset_configs/kitti_dataset.yaml
- Reducida:
python3 -m pcdet.datasets.kitti.fixed_kitti_dataset create_kitti_infos src/lidar/tools/cfgs/dataset_configs/create_dataset_info_downsampled_kitti_dataset.yaml

Testing:

python3 tools/test.py --cfg_file ${CONFIG_FILE} --batch_size ${BATCH_SIZE} --ckpt ${CKPT}

python3 tools/test.py --cfg_file src/lidar/tools/cfgs/kitti_models/custom_pointrcnn_iou.yaml --batch_size 1 --ckpt data/pretrained-models/pointrcnn_iou_7875.pth 


## Testeo con symlinks para archivos generales de configuración y datos interpolados

Ejemplo con PointRCNN_IOU:

python3 src/lidar/tools/setupandruntest.py --dataset-root <path-to-dataset-metadata> --image-sets-root <path-to-image-sets> --lidar-root <path-to-lidar-bins> --cfg_file src/lidar/tools/cfgs/kitti_models/custom_pointrcnn_iou.yaml --batch_size 1 --ckpt data/pretrained-models/pointrcnn_iou_7875.pth 

## Evaluación automática de modelos OpenPCDet y generación de tablas LaTeX

Esta sección describe los scripts utilizados para evaluar múltiples modelos de OpenPCDet sobre distintas variantes de datasets LiDAR interpolados y para generar tablas comparativas en formato LaTeX a partir de los resultados.

### Descripción general

El flujo implementado permite:
- Ejecutar evaluaciones de varios modelos sobre múltiples datasets interpolados
- Evitar la sobrescritura de resultados de OpenPCDet separando la salida por dataset
- Extraer métricas 3D AP_R40 en los niveles Easy Moderate y Hard desde los logs
- Generar una única tabla LaTeX por dataset incluyendo las clases Car Pedestrian y Cyclist

### Scripts relacionados
eval_all.sh

Script principal de evaluación.

Funciones principales:

- Recorre todas las carpetas dentro de la ruta /OpenPCDet/data/interpolated-kitti/

- Evalúa cada modelo con su checkpoint y archivo de configuración correspondiente

- Reubica automáticamente la carpeta de salida de OpenPCDet para evitar sobrescritura

### Uso típico:
    chmod +x eval_all.sh
    ./eval_all.sh
    parse_ap_r40.py

Script auxiliar para la extracción de métricas desde los logs de OpenPCDet.

Funciones principales:

Extrae métricas 3D AP_R40 para las clases Car Pedestrian y Cyclist

Obtiene los valores Easy Moderate y Hard

Uso típico:
python3 parse_ap_r40.py archivo_log

Este script suele ser invocado por otros scripts y no manualmente.

collect_results.sh

Script de agregación de resultados.

Funciones principales:

Procesa los logs generados por las evaluaciones

Consolida todas las métricas en un único archivo CSV llamado results.csv

Formato del archivo CSV:
dataset model class easy moderate hard

Uso típico:
chmod +x collect_results.sh
./collect_results.sh

latex_table_dataset.py

Script de generación de tablas LaTeX.

Funciones principales:

Genera una tabla LaTeX por dataset

Incluye todos los modelos evaluados

Reporta métricas AP_R40 para Car Pedestrian y Cyclist

Guarda la tabla en un archivo de texto con extensión txt

Uso típico:
python3 latex_table_dataset.py nombre_dataset

Ejemplo de salida generada:
table_interpolated_linear_01.txt

Flujo de trabajo recomendado

Orden de ejecución:

Ejecutar eval_all.sh

Ejecutar collect_results.sh

Ejecutar latex_table_dataset.py para cada dataset de interés

Observaciones

El formato de evaluación sigue el estándar de KITTI con niveles Easy Moderate y Hard

La estructura facilita la comparación directa entre modelos y datasets

Los scripts pueden extenderse para soportar otras métricas como bbox o BEV

También es posible automatizar la generación de tablas para todos los datasets


## Comparacion de nubes de puntos y bounding boxes

En container velodyne_ros2, se puede usar el archivo kitti_inspector.py, que permite publicar frames de todas las bases generadas en simultaneo, de forma sencilla:


pixi run python3 packages/pointcloud_utils/scripts/kitti_inspector.py --original-dir data/kitti/training/velodyne/ --reduced-dir data/reduced-kitti/training/velodyne/ --interp-dir "nearest_01" data/output/interpolated-kitti/nearest_01/training/velodyne/ --label-dir data/kitti/training/label_2/ --calib-dir data/kitti/training/calib/ --det-dir "second" data/output_runs/new-test/raw_results/desempeno_offline/original/kitti/second/default/eval/epoch_7862/val/default/final_result/data/