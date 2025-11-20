#!/bin/bash
# Script para interpolar múltiples archivos .bin
#
# Uso: ./batch_interpolate.sh <directorio_entrada> <directorio_salida> [método] [scale_y]

# Parámetros por defecto
METHOD="linear"
SCALE_Y="2.0"

if [ $# -lt 2 ]; then
    echo "Uso: $0 <directorio_entrada> <directorio_salida> [método] [scale_y]"
    echo ""
    echo "Parámetros opcionales:"
    echo "  método:  linear (default), nearest, bilateral, edgeAware, spline"
    echo "  scale_y: factor de escala vertical (default: 2.0)"
    echo ""
    echo "Ejemplo:"
    echo "  $0 ../datos/kitti/training/velodyne_16ch ../datos/kitti/training/velodyne_interpolated"
    echo "  $0 ../datos/kitti/training/velodyne_16ch ../datos/kitti/training/velodyne_interpolated linear 2.0"
    exit 1
fi

INPUT_DIR=$1
OUTPUT_DIR=$2
[ $# -ge 3 ] && METHOD=$3
[ $# -ge 4 ] && SCALE_Y=$4

SCRIPT="packages/dynamic_lidar_interpolation/scripts/interpolate_bin_file.py"

# Verificar que existe el directorio de entrada
if [ ! -d "$INPUT_DIR" ]; then
    echo "ERROR: Directorio de entrada no existe: $INPUT_DIR"
    exit 1
fi

# Verificar que existe el script
if [ ! -f "$SCRIPT" ]; then
    echo "ERROR: Script no encontrado: $SCRIPT"
    exit 1
fi

# Crear directorio de salida
mkdir -p "$OUTPUT_DIR"

# Contar archivos
TOTAL=$(find "$INPUT_DIR" -name "*.bin" | wc -l)
echo "Encontrados $TOTAL archivos .bin"
echo "Entrada:  $INPUT_DIR"
echo "Salida:   $OUTPUT_DIR"
echo "Método:   $METHOD"
echo "Scale Y:  $SCALE_Y"
echo ""

echo "IMPORTANTE: Asegúrate de que el nodo de interpolación esté corriendo:"
echo "  ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py"
echo ""
read -p "¿Continuar? (s/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Ss]$ ]]; then
    exit 1
fi

# Procesar archivos
COUNT=0
for INPUT_FILE in "$INPUT_DIR"/*.bin; do
    if [ -f "$INPUT_FILE" ]; then
        FILENAME=$(basename "$INPUT_FILE")
        OUTPUT_FILE="$OUTPUT_DIR/$FILENAME"

        COUNT=$((COUNT + 1))
        echo "[$COUNT/$TOTAL] $FILENAME"

        python3 "$SCRIPT" \
            "$INPUT_FILE" \
            "$OUTPUT_FILE" \
            --method "$METHOD" \
            --scale-y "$SCALE_Y"

        if [ $? -ne 0 ]; then
            echo "ERROR procesando $FILENAME"
            exit 1
        fi
    fi
done

echo ""
echo "✓ Completado: $COUNT archivos procesados"
