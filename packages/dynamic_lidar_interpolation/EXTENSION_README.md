# Extensión para Interpolación de Archivos .bin

Esta extensión permite interpolar archivos de nubes de puntos `.bin` (formato KITTI) sin necesidad de ROS2 en tiempo de ejecución. Todos los parámetros de interpolación se cargan desde un archivo de configuración YAML.

## Requisitos

```bash
pip install pybind11 numpy pyyaml
```

## Compilación

```bash
cd /path/to/your/workspace
colcon build --packages-select dynamic_lidar_interpolation
source install/setup.bash
```

## Uso

### Script principal: `interpolate_with_config.py`

```bash
# Uso básico
python3 scripts/interpolate_with_config.py input.bin output.bin --config config/interpolation_config.yaml

# Con salida detallada
python3 scripts/interpolate_with_config.py input.bin output.bin --config config.yaml --verbose

# Ver información del archivo .bin
python3 scripts/interpolate_with_config.py input.bin output.bin --info
```

### Script simplificado: `interpolate_bin_simple.py`

```bash
# Interpolar un archivo
python3 scripts/interpolate_bin_simple.py input.bin output.bin --config config.yaml

# Procesar múltiples archivos
for f in *.bin; do
    python3 scripts/interpolate_bin_simple.py "$f" "interpolated_$f" --config config.yaml
done
```

## Formato del Archivo de Configuración

El archivo de configuración es un YAML con la siguiente estructura:

```yaml
lidar:
  max_range: 100.0
  min_range: 0.0

range_image:
  angular_resolution_x: 0.25
  angular_resolution_y: 2.05
  max_angle_width: 360.0
  max_angle_height: 180.0
  min_angle_fov: 0.0
  max_angle_fov: 360.0

interpolation:
  method: "linear"  # linear, nearest, bilateral, edgeAware, spline
  scale_factor_x: 1.0
  scale_factor_y: 2.0
  interpolation_max_var: 50.0
  apply_variance_filter: false
  rotation_angle_x: 0.0
  extrapolation_value: "NaN"
  sensor_translation: [0.0, 0.0, 0.0]
```

Ver `config/interpolation_config.yaml` para más detalles sobre los parámetros disponibles.

## Archivos Creados

- `src/interpolation_bindings.cpp` - Bindings de Python para los algoritmos C++
- `scripts/interpolate_with_config.py` - Script principal con configuración desde archivo
- `scripts/interpolate_bin_simple.py` - Script simplificado (actualizado para usar configuración)
