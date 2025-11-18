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

## Interpolación de Archivos .bin

El paquete `dynamic_lidar_interpolation` ahora soporta procesar archivos .bin directamente (formato KITTI/nuScenes).

### Uso Rápido

**Paso 1:** Iniciar el nodo de interpolación
```bash
ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py
```

**Paso 2:** Procesar archivos .bin (en otra terminal)
```bash
cd packages/dynamic_lidar_interpolation/scripts

# Uso básico
python3 interpolate_bin_file.py entrada.bin salida.bin

# Con parámetros personalizados
python3 interpolate_bin_file.py entrada.bin salida.bin --method linear --scale-y 2.0
```

### Procesamiento por Lotes

```bash
#!/bin/bash
DIR_ENTRADA="ruta/a/archivos/velodyne"
DIR_SALIDA="ruta/a/salida/velodyne_denso"

mkdir -p "$DIR_SALIDA"

for archivo in "$DIR_ENTRADA"/*.bin; do
    nombre=$(basename "$archivo")
    python3 interpolate_bin_file.py "$archivo" "$DIR_SALIDA/$nombre" --scale-y 2.0
done
```

### Métodos de Interpolación Disponibles

- `linear` (recomendado) - Rápido y buena calidad
- `nearest` - Más rápido
- `bilateral` - Preserva bordes
- `edgeAware` - Mantiene límites de objetos
- `spline` - Más suave, más lento

### Parámetros Principales

- `--method`: Método de interpolación
- `--scale-x`: Factor de escala horizontal (default: 1.0)
- `--scale-y`: Factor de escala vertical (default: 2.0)
- `--config`: Archivo YAML de configuración personalizada