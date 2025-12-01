#!/usr/bin/env python3
"""
Script para convertir datos de LiDAR de 64 canales a 16 canales.

Este script toma nubes de puntos capturadas con un LiDAR de 64 canales (como Velodyne HDL-64E)
y las reduce a 16 canales seleccionando anillos específicos, simulando la captura con un
LiDAR de 16 canales (como Velodyne VLP-16).

Soporta procesamiento de archivos individuales o de carpetas completas.

@author Cristobal Medina
@license AGPL-3.0
"""

import numpy as np
import argparse
from pathlib import Path
import sys
from tqdm import tqdm

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


def downsample_64_to_16_rings(x, y, z, intensity, num_rings=16, verbose=True):
    """
    Reduce datos de 64 canales a 16 canales seleccionando anillos uniformemente.

    Velodyne HDL-64E tiene FOV vertical de aproximadamente -24.9° a +2°
    Velodyne VLP-16 tiene FOV vertical de aproximadamente -15° a +15°

    Args:
        x, y, z: Coordenadas de los puntos
        intensity: Valores de intensidad
        num_rings: Número de anillos a mantener (default: 16)
        verbose: Si True, imprime información detallada

    Returns:
        Tupla de (x_down, y_down, z_down, intensity_down) con datos reducidos
    """
    # Calcular ángulos de elevación
    elevations = calculate_elevation_angle(x, y, z)

    # Determinar el rango de elevación
    min_elev = elevations.min()
    max_elev = elevations.max()

    if verbose:
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

    if verbose:
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


def process_single_file(input_file, output_file, num_rings=16, dry_run=False, verbose=True):
    """
    Procesa un solo archivo .bin

    Args:
        input_file: Ruta del archivo de entrada
        output_file: Ruta del archivo de salida
        num_rings: Número de anillos a mantener
        dry_run: Si True, no guarda el archivo
        verbose: Si True, imprime información detallada

    Returns:
        Diccionario con estadísticas del procesamiento
    """
    # Leer archivo de entrada
    if verbose:
        print(f"Leyendo: {input_file}")
    x, y, z, intensity = bin_file_utils.read_bin_pointcloud(str(input_file))

    original_points = len(x)
    if verbose:
        print(f"Puntos originales (64 canales): {original_points}")

    # Aplicar downsampling
    if verbose:
        print(f"\nReduciendo a {num_rings} canales...")
    x_down, y_down, z_down, intensity_down = downsample_64_to_16_rings(
        x, y, z, intensity, num_rings=num_rings, verbose=verbose
    )

    reduced_points = len(x_down)
    reduction_pct = reduced_points / original_points * 100

    if verbose:
        print(f"\nPuntos reducidos ({num_rings} canales): {reduced_points}")
        print(f"Reducción: {reduction_pct:.1f}% de puntos originales")

        # Estadísticas
        print(f"\nEstadísticas de datos reducidos:")
        print(f"  X: [{x_down.min():.2f}, {x_down.max():.2f}]")
        print(f"  Y: [{y_down.min():.2f}, {y_down.max():.2f}]")
        print(f"  Z: [{z_down.min():.2f}, {z_down.max():.2f}]")
        print(f"  Intensidad: [{intensity_down.min():.2f}, {intensity_down.max():.2f}]")

    # Guardar resultado
    if not dry_run:
        if verbose:
            print(f"\nGuardando: {output_file}")
        bin_file_utils.write_bin_pointcloud(
            str(output_file), x_down, y_down, z_down, intensity_down
        )
        if verbose:
            print("¡Completado!")
    else:
        if verbose:
            print("\n[Modo dry-run: archivo no guardado]")

    return {
        'input_file': input_file,
        'output_file': output_file,
        'original_points': original_points,
        'reduced_points': reduced_points,
        'reduction_pct': reduction_pct
    }


def process_folder(input_folder, output_folder, num_rings=16, dry_run=False):
    """
    Procesa todos los archivos .bin en una carpeta

    Args:
        input_folder: Carpeta con archivos de entrada
        output_folder: Carpeta para archivos de salida
        num_rings: Número de anillos a mantener
        dry_run: Si True, no guarda archivos
    """
    input_path = Path(input_folder)
    output_path = Path(output_folder)

    # Validar carpeta de entrada
    if not input_path.exists():
        raise FileNotFoundError(f"Carpeta no encontrada: {input_folder}")

    if not input_path.is_dir():
        raise NotADirectoryError(f"No es una carpeta: {input_folder}")

    # Buscar archivos .bin
    bin_files = sorted(list(input_path.glob("*.bin")))

    if not bin_files:
        print(f"No se encontraron archivos .bin en: {input_folder}")
        return

    print(f"Encontrados {len(bin_files)} archivos .bin en {input_folder}")
    print(f"Carpeta de salida: {output_folder}\n")

    # Crear carpeta de salida si no existe
    if not dry_run:
        output_path.mkdir(parents=True, exist_ok=True)

    # Procesar cada archivo
    stats = []
    for input_file in tqdm(bin_files, desc="Procesando archivos"):
        output_file = output_path / input_file.name

        try:
            stat = process_single_file(
                input_file,
                output_file,
                num_rings=num_rings,
                dry_run=dry_run,
                verbose=False  # No mostrar detalles para cada archivo en batch
            )
            stats.append(stat)
        except Exception as e:
            print(f"\nError procesando {input_file.name}: {e}")
            continue

    # Mostrar resumen
    print("\n" + "="*60)
    print("RESUMEN DEL PROCESAMIENTO")
    print("="*60)


    total_original = sum(s['original_points'] for s in stats)
    total_reduced = sum(s['reduced_points'] for s in stats)
    avg_reduction = np.mean([s['reduction_pct'] for s in stats])

    print(f"Archivos procesados: {len(stats)}/{len(bin_files)}")
    print(f"Total puntos originales: {total_original:,}")
    print(f"Total puntos reducidos: {total_reduced:,}")
    print(f"Reducción promedio: {avg_reduction:.1f}%")

    if dry_run:
        print("\n[Modo dry-run: archivos no guardados]")


def main():
    parser = argparse.ArgumentParser(
        description='Reducir datos de LiDAR de 64 canales a 16 canales',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  # Procesar un solo archivo
  %(prog)s input_64ch.bin output_16ch.bin

  # Procesar todos los archivos en una carpeta
  %(prog)s input_folder/ output_folder/ --batch

  # Especificar número de anillos
  %(prog)s input_folder/ output_folder/ --batch --rings 16

  # Mostrar estadísticas sin guardar
  %(prog)s input_folder/ output_folder/ --batch --dry-run
        """
    )

    parser.add_argument('input', type=str,
                        help='Archivo .bin de entrada o carpeta (con --batch)')
    parser.add_argument('output', type=str,
                        help='Archivo .bin de salida o carpeta (con --batch)')
    parser.add_argument('--batch', action='store_true',
                        help='Procesar todos los .bin en la carpeta de entrada')
    parser.add_argument('--rings', type=int, default=16,
                        help='Número de anillos a mantener (default: 16)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Mostrar estadísticas sin guardar archivos')

    args = parser.parse_args()

    try:
        if args.batch:
            # Modo batch: procesar carpeta completa
            process_folder(args.input, args.output, args.rings, args.dry_run)
        else:
            # Modo single: procesar archivo individual
            input_file = Path(args.input)
            if not input_file.exists():
                print(f"Error: Archivo no encontrado: {args.input}", file=sys.stderr)
                return 1

            process_single_file(
                input_file,
                Path(args.output),
                args.rings,
                args.dry_run,
                verbose=True
            )

        return 0

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
