# Scripts de Procesamiento KITTI

Este directorio contiene scripts para procesar el dataset KITTI completo.

## Scripts Disponibles

### `process_kitti_pipeline.sh`
Pipeline completo automatizado que ejecuta todo el proceso:
- Reduce densidad de 64ch a 16ch
- Interpola los datos de 16ch
- Procesa training y testing

**Uso:**
```bash
./process_kitti_pipeline.sh
```

### `batch_downsample.sh`
Reduce la densidad de múltiples archivos (64ch → 16ch).

**Uso:**
```bash
./batch_downsample.sh <dir_entrada> <dir_salida>
```

**Ejemplo:**
```bash
./batch_downsample.sh ../datos/kitti/training/velodyne ../datos/kitti/training/velodyne_16ch
```

### `batch_interpolate.sh`
Interpola múltiples archivos de baja densidad.

**Uso:**
```bash
./batch_interpolate.sh <dir_entrada> <dir_salida> [método] [scale_y]
```

**Ejemplo:**
```bash
./batch_interpolate.sh ../datos/kitti/training/velodyne_16ch ../datos/kitti/training/velodyne_interpolated linear 2.0
```

## Workflow Recomendado

1. **Preparar datos**: Asegúrate de tener los datos KITTI en `../datos/kitti/`

2. **Opción A - Automático**:
   ```bash
   chmod +x process_kitti_pipeline.sh
   ./process_kitti_pipeline.sh
   ```

3. **Opción B - Manual**:
   ```bash
   # Paso 1: Downsampling
   chmod +x batch_downsample.sh
   ./batch_downsample.sh ../datos/kitti/training/velodyne ../datos/kitti/training/velodyne_16ch

   # Paso 2: Interpolación (requiere nodo ROS2 corriendo)
   # Terminal 1:
   ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py

   # Terminal 2:
   chmod +x batch_interpolate.sh
   ./batch_interpolate.sh ../datos/kitti/training/velodyne_16ch ../datos/kitti/training/velodyne_interpolated
   ```

## Estructura de Salida

```
../datos/kitti/
├── training/
│   ├── velodyne/              # Original
│   ├── velodyne_16ch/         # Reducido
│   └── velodyne_interpolated/ # Interpolado
└── testing/
    ├── velodyne/
    ├── velodyne_16ch/
    └── velodyne_interpolated/
```
