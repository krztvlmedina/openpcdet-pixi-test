#!/bin/bash
# Pipeline completo para procesar datos KITTI:
# 1. Datos originales (64 canales) -> Baja densidad (16 canales) -> Interpolado
#
# Uso: ./process_kitti_pipeline.sh

set -e  # Salir si hay errores

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

# Directorio base de datos KITTI
KITTI_BASE="../datos/kitti"

# Directorios de entrada (datos originales de 64 canales)
TRAINING_VELODYNE="${KITTI_BASE}/training/velodyne"
TESTING_VELODYNE="${KITTI_BASE}/testing/velodyne"

# Directorios de salida para datos de baja densidad (16 canales)
TRAINING_16CH="${KITTI_BASE}/training/velodyne_16ch"
TESTING_16CH="${KITTI_BASE}/testing/velodyne_16ch"

# Directorios de salida para datos interpolados
TRAINING_INTERPOLATED="${KITTI_BASE}/training/velodyne_interpolated"
TESTING_INTERPOLATED="${KITTI_BASE}/testing/velodyne_interpolated"

# Scripts
DOWNSAMPLE_SCRIPT="packages/dynamic_lidar_interpolation/scripts/downsample_64_to_16.py"
INTERPOLATE_SCRIPT="packages/dynamic_lidar_interpolation/scripts/interpolate_bin_file.py"

# Parámetros de interpolación
INTERPOLATION_METHOD="linear"
SCALE_FACTOR_Y="2.0"

# ============================================================================
# FUNCIONES
# ============================================================================

print_section() {
    echo ""
    echo "================================================================"
    echo "$1"
    echo "================================================================"
    echo ""
}

check_prerequisites() {
    print_section "Verificando prerequisitos"

    # Verificar que existen los directorios de datos originales
    if [ ! -d "$TRAINING_VELODYNE" ]; then
        echo "ERROR: No se encuentra el directorio: $TRAINING_VELODYNE"
        exit 1
    fi

    if [ ! -d "$TESTING_VELODYNE" ]; then
        echo "ADVERTENCIA: No se encuentra el directorio: $TESTING_VELODYNE"
        echo "Continuando solo con datos de entrenamiento..."
        PROCESS_TESTING=false
    else
        PROCESS_TESTING=true
    fi

    # Verificar que existen los scripts
    if [ ! -f "$DOWNSAMPLE_SCRIPT" ]; then
        echo "ERROR: No se encuentra el script: $DOWNSAMPLE_SCRIPT"
        exit 1
    fi

    if [ ! -f "$INTERPOLATE_SCRIPT" ]; then
        echo "ERROR: No se encuentra el script: $INTERPOLATE_SCRIPT"
        exit 1
    fi

    # Contar archivos
    TRAINING_FILES=$(find "$TRAINING_VELODYNE" -name "*.bin" | wc -l)
    echo "Archivos de entrenamiento encontrados: $TRAINING_FILES"

    if [ "$PROCESS_TESTING" = true ]; then
        TESTING_FILES=$(find "$TESTING_VELODYNE" -name "*.bin" | wc -l)
        echo "Archivos de prueba encontrados: $TESTING_FILES"
    fi
}

create_directories() {
    print_section "Creando directorios de salida"

    mkdir -p "$TRAINING_16CH"
    mkdir -p "$TRAINING_INTERPOLATED"

    if [ "$PROCESS_TESTING" = true ]; then
        mkdir -p "$TESTING_16CH"
        mkdir -p "$TESTING_INTERPOLATED"
    fi

    echo "Directorios creados:"
    echo "  - $TRAINING_16CH"
    echo "  - $TRAINING_INTERPOLATED"
    if [ "$PROCESS_TESTING" = true ]; then
        echo "  - $TESTING_16CH"
        echo "  - $TESTING_INTERPOLATED"
    fi
}

process_downsample() {
    local input_dir=$1
    local output_dir=$2
    local dataset_name=$3

    print_section "Paso 1: Reduciendo densidad ($dataset_name)"
    echo "Entrada:  $input_dir"
    echo "Salida:   $output_dir"
    echo ""

    local count=0
    local total=$(find "$input_dir" -name "*.bin" | wc -l)

    for input_file in "$input_dir"/*.bin; do
        if [ -f "$input_file" ]; then
            filename=$(basename "$input_file")
            output_file="$output_dir/$filename"

            count=$((count + 1))
            echo "[$count/$total] Procesando: $filename"

            python3 "$DOWNSAMPLE_SCRIPT" "$input_file" "$output_file" --rings 16

            if [ $? -ne 0 ]; then
                echo "ERROR procesando $filename"
                exit 1
            fi
        fi
    done

    echo ""
    echo "✓ Downsampling completado: $count archivos procesados"
}

process_interpolation() {
    local input_dir=$1
    local output_dir=$2
    local dataset_name=$3

    print_section "Paso 2: Interpolando datos ($dataset_name)"
    echo "Entrada:  $input_dir"
    echo "Salida:   $output_dir"
    echo "Método:   $INTERPOLATION_METHOD"
    echo "Scale Y:  $SCALE_FACTOR_Y"
    echo ""

    echo "NOTA: Asegúrate de que el nodo de interpolación esté corriendo:"
    echo "  ros2 launch dynamic_lidar_interpolation pointcloud_interpolation_launch.py"
    echo ""
    read -p "¿El nodo está corriendo? (s/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Ss]$ ]]; then
        echo "Por favor inicia el nodo y vuelve a ejecutar el script"
        exit 1
    fi

    local count=0
    local total=$(find "$input_dir" -name "*.bin" | wc -l)

    for input_file in "$input_dir"/*.bin; do
        if [ -f "$input_file" ]; then
            filename=$(basename "$input_file")
            output_file="$output_dir/$filename"

            count=$((count + 1))
            echo "[$count/$total] Interpolando: $filename"

            python3 "$INTERPOLATE_SCRIPT" \
                "$input_file" \
                "$output_file" \
                --method "$INTERPOLATION_METHOD" \
                --scale-y "$SCALE_FACTOR_Y"

            if [ $? -ne 0 ]; then
                echo "ERROR interpolando $filename"
                exit 1
            fi
        fi
    done

    echo ""
    echo "✓ Interpolación completada: $count archivos procesados"
}

print_summary() {
    print_section "Resumen del procesamiento"

    echo "Estructura de directorios resultante:"
    echo ""
    echo "$KITTI_BASE/"
    echo "├── training/"
    echo "│   ├── velodyne/              (original 64 canales)"
    echo "│   ├── velodyne_16ch/         (reducido a 16 canales)"
    echo "│   └── velodyne_interpolated/ (interpolado desde 16ch)"

    if [ "$PROCESS_TESTING" = true ]; then
        echo "└── testing/"
        echo "    ├── velodyne/              (original 64 canales)"
        echo "    ├── velodyne_16ch/         (reducido a 16 canales)"
        echo "    └── velodyne_interpolated/ (interpolado desde 16ch)"
    fi

    echo ""
    echo "Estadísticas:"
    echo "  Training original:     $(find "$TRAINING_VELODYNE" -name "*.bin" 2>/dev/null | wc -l) archivos"
    echo "  Training 16ch:         $(find "$TRAINING_16CH" -name "*.bin" 2>/dev/null | wc -l) archivos"
    echo "  Training interpolado:  $(find "$TRAINING_INTERPOLATED" -name "*.bin" 2>/dev/null | wc -l) archivos"

    if [ "$PROCESS_TESTING" = true ]; then
        echo "  Testing original:      $(find "$TESTING_VELODYNE" -name "*.bin" 2>/dev/null | wc -l) archivos"
        echo "  Testing 16ch:          $(find "$TESTING_16CH" -name "*.bin" 2>/dev/null | wc -l) archivos"
        echo "  Testing interpolado:   $(find "$TESTING_INTERPOLATED" -name "*.bin" 2>/dev/null | wc -l) archivos"
    fi
}

# ============================================================================
# MAIN
# ============================================================================

main() {
    echo "================================================================"
    echo "Pipeline de Procesamiento KITTI"
    echo "64ch -> 16ch -> Interpolado"
    echo "================================================================"

    # Verificar prerequisitos
    check_prerequisites

    # Crear directorios de salida
    create_directories

    # Procesar datos de entrenamiento
    process_downsample "$TRAINING_VELODYNE" "$TRAINING_16CH" "Training"
    process_interpolation "$TRAINING_16CH" "$TRAINING_INTERPOLATED" "Training"

    # Procesar datos de prueba si existen
    if [ "$PROCESS_TESTING" = true ]; then
        process_downsample "$TESTING_VELODYNE" "$TESTING_16CH" "Testing"
        process_interpolation "$TESTING_16CH" "$TESTING_INTERPOLATED" "Testing"
    fi

    # Mostrar resumen
    print_summary

    print_section "¡COMPLETADO!"
}

# Ejecutar main
main
