# Point Cloud Utils

Utilidades para procesamiento de nubes de puntos, herramientas para datasets KITTI, y publicación de archivos .bin.

Este paquete proporciona herramientas extraídas del paquete `dynamic_lidar_interpolation` para trabajar con datos de nubes de puntos en formato .bin (comúnmente usado en datasets de conducción autónoma).

## Características

### 1. Nodo Publicador de Archivos Bin (C++)

Un nodo de ROS2 que publica archivos de nubes de puntos .bin como mensajes `sensor_msgs::PointCloud2`.

**Características:**
- **Reproducción de directorio**: Publica secuencialmente todos los archivos .bin de un directorio
- **Tasa configurable**: Tasa de publicación ajustable (Hz) mediante parámetros
- **Sincronización de timestamps**: Opción para usar tiempos de modificación de archivos para reproducción realista
- **Modo loop**: Reproduce archivos continuamente o se detiene después de una pasada

**Uso:**

```bash
# Lanzar con parámetros de línea de comandos
ros2 launch pointcloud_utils bin_publisher_launch.py \
  bin_directory:=/ruta/a/archivos/bin \
  publish_rate:=10.0 \
  loop:=true

# Lanzar con archivo de configuración
ros2 launch pointcloud_utils bin_publisher_launch.py \
  config_file:=/ruta/a/config.yaml

# Ejecutar nodo directamente
ros2 run pointcloud_utils bin_publisher_node \
  --ros-args -p bin_directory:=/ruta/a/archivos/bin
```

**Parámetros:**
- `bin_directory` (string, requerido): Directorio que contiene archivos .bin
- `publish_rate` (double, default: 10.0): Frecuencia de publicación en Hz
- `loop` (bool, default: true): Repetir archivos continuamente
- `use_file_timestamps` (bool, default: false): Usar tiempos de modificación de archivos
- `frame_id` (string, default: "velodyne"): Frame ID para las nubes de puntos
- `topic` (string, default: "velodyne_points"): Nombre del topic de publicación
- `verbose` (bool, default: true): Habilitar logging detallado

### 2. Utilidades de Archivos Bin (Python)

Utilidades Python para leer y escribir archivos de nubes de puntos .bin.

**Uso:**

```python
from pointcloud_utils.scripts import bin_file_utils

# Leer un archivo .bin
x, y, z, intensity = bin_file_utils.read_bin_pointcloud("input.bin")

# Escribir un archivo .bin
bin_file_utils.write_bin_pointcloud("output.bin", x, y, z, intensity)

# Leer como array Nx3
xyz_array = bin_file_utils.bin_to_xyz_array("input.bin")

# Escribir desde array Nx3
bin_file_utils.xyz_array_to_bin(xyz_array, "output.bin", intensity)
```

**Línea de comandos:**

```bash
# Mostrar estadísticas del archivo
ros2 run pointcloud_utils bin_file_utils.py input.bin
```

### 3. Downsample de 64 a 16 Canales

Convertir datos de LiDAR de 64 canales (ej. Velodyne HDL-64E) a datos de 16 canales (ej. Velodyne VLP-16).

**Características:**
- Selección uniforme de anillos de 64 canales a 16 canales
- Procesamiento de archivo único o carpeta completa en batch
- Reporte de estadísticas
- Modo dry-run para pruebas

**Uso:**

```bash
# Procesar un solo archivo
ros2 run pointcloud_utils downsample_64_to_16.py \
  input_64ch.bin output_16ch.bin

# Procesar carpeta completa en batch
ros2 run pointcloud_utils downsample_64_to_16.py \
  input_folder/ output_folder/ --batch

# Especificar número de anillos
ros2 run pointcloud_utils downsample_64_to_16.py \
  input_folder/ output_folder/ --batch --rings 16

# Dry run (mostrar estadísticas sin guardar)
ros2 run pointcloud_utils downsample_64_to_16.py \
  input_folder/ output_folder/ --batch --dry-run
```

## Formato de Archivo

Los archivos .bin usan el formato del dataset KITTI:
- Archivo binario con valores float32
- Formato: `[x1, y1, z1, intensity1, x2, y2, z2, intensity2, ...]`
- 16 bytes por punto (4 floats × 4 bytes)

## Compilación

Este paquete se compila como parte del workspace de ROS2:

```bash
cd /ruta/al/workspace
colcon build --packages-select pointcloud_utils
source install/setup.bash
```

## Dependencias

- ROS2 (Humble o posterior)
- PCL (Point Cloud Library)
- Python 3
- numpy
- tqdm (para barras de progreso en scripts de Python)

## Créditos

- Paquete de interpolación original por Abdalrahman M. Amer
- Extracción de paquete, script de downsampling y nodo publicador ROS2 por Cristobal Medina

## Licencia

Licencia MIT (a menos que se especifique lo contrario en archivos individuales)
