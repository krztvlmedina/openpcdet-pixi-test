#!/usr/bin/env python3
"""
Script para convertir datos de LiDAR de 64 canales a 16 canales.

Este script toma nubes de puntos capturadas con un LiDAR de 64 canales (como Velodyne HDL-64E)
y las reduce a 16 canales seleccionando anillos específicos, simulando la captura con un
LiDAR de 16 canales (como Velodyne VLP-16).

@author Abdalrahman M. Amer
@license AGPL-3.0
"""

import numpy as np
import argparse
from pathlib import Path
import sys

try:
    import bin_file_utils
except ImportError:
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bin_file_utils


def calculate_elevation_angle(x, y, z):
    """
    Calcula el ángulo de elevación para cada punto.

    Args:
        x, y, z: Coordenadas de los puntos

    Returns:
        Ángulos de elevación en grados
    """
    horizontal_dist = np.sqrt(x**2 + y**2)
    elevation = np.arctan2(z, horizontal_dist)
    return np.degrees(elevation)


def downsample_64_to_16_rings(x, y, z, intensity, num_rings=16):
    """
    Reduce datos de 64 canales a 16 canales seleccionando anillos uniformemente.

    Velodyne HDL-64E tiene FOV vertical de aproximadamente -24.9° a +2°
    Velodyne VLP-16 tiene FOV vertical de aproximadamente -15° a +15°

    Args:
        x, y, z: Coordenadas de los puntos
        intensity: Valores de intensidad
        num_rings: Número de anillos a mantener (default: 16)

    Returns:
        Tupla de (x_down, y_down, z_down, intensity_down) con datos reducidos
    """
    # Calcular ángulos de elevación
    elevations = calculate_elevation_angle(x, y, z)

    # Determinar el rango de elevación
    min_elev = elevations.min()
    max_elev = elevations.max()

    print(f"Rango de elevación original: {min_elev:.2f}° a {max_elev:.2f}°")

    # Crear bins para los anillos
    # Dividimos el rango de elevación en 64 bins (asumiendo 64 canales originales)
    num_original_rings = 64
    elevation_bins = np.linspace(min_elev, max_elev, num_original_rings + 1)

    # Asignar cada punto a su anillo correspondiente
    ring_indices = np.digitize(elevations, elevation_bins) - 1
    ring_indices = np.clip(ring_indices, 0, num_original_rings - 1)

    # Seleccionar cuáles anillos mantener (distribuidos uniformemente)
    # Por ejemplo, para 16 anillos de 64: seleccionar cada 4to anillo
    stride = num_original_rings // num_rings
    selected_rings = np.arange(0, num_original_rings, stride)[:num_rings]

    print(f"Anillos seleccionados: {selected_rings}")
    print(f"Ángulos de elevación aproximados:")
    for i, ring_idx in enumerate(selected_rings):
        angle = elevation_bins[ring_idx]
        print(f"  Anillo {i}: {angle:.2f}°")

    # Crear máscara para puntos que pertenecen a los anillos seleccionados
    mask = np.isin(ring_indices, selected_rings)

    # Filtrar puntos
    x_down = x[mask]
    y_down = y[mask]
    z_down = z[mask]
    intensity_down = intensity[mask]

    return x_down, y_down, z_down, intensity_down


def main():
    parser = argparse.ArgumentParser(
        description='Reducir datos de LiDAR de 64 canales a 16 canales',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  # Procesar un solo archivo
  %(prog)s input_64ch.bin output_16ch.bin

  # Especificar número de anillos
  %(prog)s input_64ch.bin output_16ch.bin --rings 16

  # Mostrar estadísticas sin guardar
  %(prog)s input_64ch.bin output_16ch.bin --dry-run
        """
    )

    parser.add_argument('input_file', type=str, help='Archivo .bin de entrada (64 canales)')
    parser.add_argument('output_file', type=str, help='Archivo .bin de salida (16 canales)')
    parser.add_argument('--rings', type=int, default=16,
                        help='Número de anillos a mantener (default: 16)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Mostrar estadísticas sin guardar archivo')

    args = parser.parse_args()

    # Validar archivo de entrada
    if not Path(args.input_file).exists():
        print(f"Error: Archivo no encontrado: {args.input_file}", file=sys.stderr)
        return 1

    try:
        # Leer archivo de entrada
        print(f"Leyendo: {args.input_file}")
        x, y, z, intensity = bin_file_utils.read_bin_pointcloud(args.input_file)
        print(f"Puntos originales (64 canales): {len(x)}")

        # Aplicar downsampling
        print(f"\nReduciendo a {args.rings} canales...")
        x_down, y_down, z_down, intensity_down = downsample_64_to_16_rings(
            x, y, z, intensity, num_rings=args.rings
        )

        print(f"\nPuntos reducidos ({args.rings} canales): {len(x_down)}")
        print(f"Reducción: {len(x_down)/len(x)*100:.1f}% de puntos originales")

        # Estadísticas
        print(f"\nEstadísticas de datos reducidos:")
        print(f"  X: [{x_down.min():.2f}, {x_down.max():.2f}]")
        print(f"  Y: [{y_down.min():.2f}, {y_down.max():.2f}]")
        print(f"  Z: [{z_down.min():.2f}, {z_down.max():.2f}]")
        print(f"  Intensidad: [{intensity_down.min():.2f}, {intensity_down.max():.2f}]")

        # Guardar resultado
        if not args.dry_run:
            print(f"\nGuardando: {args.output_file}")
            bin_file_utils.write_bin_pointcloud(
                args.output_file, x_down, y_down, z_down, intensity_down
            )
            print("¡Completado!")
        else:
            print("\n[Modo dry-run: archivo no guardado]")

        return 0

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
