OpenPcDET

git clone https://github.com/open-mmlab/OpenPCDet.git

OpenPCDet ros2 wrapper

cd src/
git clone https://github.com/Box-Robotics/ros2_numpy -b humble
git clone https://github.com/pradhanshrijal/pcdet_ros2
cd ..

Run rviz2

pixi run ros2 run rviz2 rviz2

---

## Pipeline de Procesamiento KITTI

Este proyecto incluye herramientas para procesar datos KITTI en tres etapas:
1. **Datos originales** (64 canales)
2. **Datos de baja densidad** (16 canales - simulando VLP-16)
3. **Datos interpolados** (densificados desde 16 canales)

### Estructura de Datos Resultante

```
../datos/kitti/
├── training/
│   ├── velodyne/              # Original (64 canales)
│   ├── velodyne_16ch/         # Baja densidad (16 canales)
│   └── velodyne_interpolated/ # Interpolado
└── testing/
    ├── velodyne/              # Original (64 canales)
    ├── velodyne_16ch/         # Baja densidad (16 canales)
    └── velodyne_interpolated/ # Interpolado
```

### Opción 1: Pipeline Completo Automatizado

Ejecuta todo el proceso con un solo comando:

```bash
cd scripts
chmod +x process_kitti_pipeline.sh
./process_kitti_pipeline.sh
```

Este script:
- Verifica que existan los datos originales
- Reduce la densidad de 64ch a 16ch
- Interpola los datos de 16ch
- Procesa tanto training como testing
- Muestra estadísticas al final

### Opción 2: Procesamiento Manual por Pasos

#### Paso 1: Reducir Densidad (64ch → 16ch)

```bash
cd scripts
chmod +x batch_downsample.sh

# Training
./batch_downsample.sh ../datos/kitti/training/velodyne ../datos/kitti/training/velodyne_16ch

# Testing
./batch_downsample.sh ../datos/kitti/testing/velodyne ../datos/kitti/testing/velodyne_16ch
```

#### Paso 2: Interpolar Datos (16ch → Denso)

**Terminal 1:** Iniciar nodo de interpolación
```bash
ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py
```

**Terminal 2:** Procesar archivos
```bash
cd scripts
chmod +x batch_interpolate.sh

# Training
./batch_interpolate.sh ../datos/kitti/training/velodyne_16ch ../datos/kitti/training/velodyne_interpolated

# Testing
./batch_interpolate.sh ../datos/kitti/testing/velodyne_16ch ../datos/kitti/testing/velodyne_interpolated
```

### Procesamiento de Archivos Individuales

#### Reducir densidad de un archivo

```bash
cd packages/dynamic_lidar_interpolation/scripts

python3 downsample_64_to_16.py entrada_64ch.bin salida_16ch.bin
```

#### Interpolar un archivo

```bash
# Terminal 1: Iniciar nodo
ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py

# Terminal 2: Procesar
cd packages/dynamic_lidar_interpolation/scripts
python3 interpolate_bin_file.py entrada_16ch.bin salida_interpolada.bin --scale-y 2.0
```

### Métodos de Interpolación Disponibles

- `linear` (recomendado) - Rápido y buena calidad
- `nearest` - Más rápido
- `bilateral` - Preserva bordes
- `edgeAware` - Mantiene límites de objetos
- `spline` - Más suave, más lento

### Parámetros de Interpolación

- `--method`: Método de interpolación (default: linear)
- `--scale-x`: Factor de escala horizontal (default: 1.0)
- `--scale-y`: Factor de escala vertical (default: 2.0)

### Verificar Resultados

```bash
# Ver estadísticas de un archivo
python3 packages/dynamic_lidar_interpolation/scripts/bin_file_utils.py archivo.bin

# Ver estadísticas con downsampling (sin guardar)
python3 packages/dynamic_lidar_interpolation/scripts/downsample_64_to_16.py \
    entrada.bin salida.bin --dry-run
```