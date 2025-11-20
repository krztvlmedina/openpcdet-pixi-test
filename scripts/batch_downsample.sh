#!/bin/bash
# Script para reducir densidad de múltiples archivos .bin (64ch -> 16ch)
#
# Uso: ./batch_downsample.sh <directorio_entrada> <directorio_salida>

if [ $# -ne 2 ]; then
    echo "Uso: $0 <directorio_entrada> <directorio_salida>"
    echo ""
    echo "Ejemplo:"
    echo "  $0 ../datos/kitti/training/velodyne ../datos/kitti/training/velodyne_16ch"
    exit 1
fi

INPUT_DIR=$1
OUTPUT_DIR=$2
SCRIPT="packages/dynamic_lidar_interpolation/scripts/downsample_64_to_16.py"

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
echo ""

# Procesar archivos
COUNT=0
for INPUT_FILE in "$INPUT_DIR"/*.bin; do
    if [ -f "$INPUT_FILE" ]; then
        FILENAME=$(basename "$INPUT_FILE")
        OUTPUT_FILE="$OUTPUT_DIR/$FILENAME"

        COUNT=$((COUNT + 1))
        echo "[$COUNT/$TOTAL] $FILENAME"

        python3 "$SCRIPT" "$INPUT_FILE" "$OUTPUT_FILE" --rings 16

        if [ $? -ne 0 ]; then
            echo "ERROR procesando $FILENAME"
            exit 1
        fi
    fi
done

echo ""
echo "✓ Completado: $COUNT archivos procesados"
