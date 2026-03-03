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

# ─── Hardware-defined elevation angles ───────────────────────────────────────
#
# Velodyne HDL-64E S2 (KITTI sensor)
# Upper block: 32 beams from +2.00° down to −8.33°, spacing 1/3°
# Lower block: 32 beams from −8.83° down to −24.33°, spacing 1/2°
# Sorted ascending (ring 0 = lowest beam).
_HDL64E_UPPER = np.arange(32) / 3.0 - 8.33           # −8.33 … +2.00  (1/3° steps)
_HDL64E_LOWER = np.arange(32) * (-0.5) - 8.83         # −8.83 … −24.33 (0.5° steps)
HDL64E_ELEVATIONS = np.sort(np.concatenate([_HDL64E_UPPER, _HDL64E_LOWER]))  # 64 values

# Velodyne VLP-16: 16 beams from −15° to +15° in 2° uniform steps.
VLP16_ELEVATIONS = np.arange(16) * 2.0 - 15.0   # −15, −13, …, +13, +15

# Pre-compute which HDL-64E ring best represents each VLP-16 beam.
# VLP-16 beams above +2° (the HDL-64E maximum) all collapse to the top ring;
# after deduplication this leaves 10 unique rings for the overlapping FOV.
_VLP16_TO_HDL64_RING = np.array([
    int(np.argmin(np.abs(HDL64E_ELEVATIONS - a))) for a in VLP16_ELEVATIONS
])
# Mid-point bin edges for assigning measured points to HDL-64E rings.
_HDL64E_BIN_EDGES = np.concatenate([
    [-np.inf],
    (HDL64E_ELEVATIONS[:-1] + HDL64E_ELEVATIONS[1:]) / 2.0,
    [np.inf],
])

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
    Simula la captura de una Velodyne VLP-16 a partir de datos HDL-64E.

    Cada punto se asigna al anillo HDL-64E más cercano usando los ángulos de
    elevación nominales del hardware (fijos, independientes del frame).
    Luego se seleccionan los anillos HDL-64E que mejor representan los
    num_rings ángulos de elevación del VLP-16.

    Nota física: el HDL-64E cubre −24.9° a +2° y el VLP-16 cubre −15° a +15°.
    En la zona de solapamiento (−15° a +2°) hay 10 anillos HDL-64E únicos que
    corresponden a beams VLP-16; los 6 beams VLP-16 por encima de +2° colapsan
    al anillo superior del HDL-64E (+2°). Por tanto el resultado tiene entre 10
    y 16 anillos efectivos según num_rings.

    Args:
        x, y, z:    Coordenadas de los puntos
        intensity:  Valores de intensidad
        num_rings:  Número de beams VLP-16 a simular (default: 16)
        verbose:    Si True, imprime información detallada

    Returns:
        Tupla (x_down, y_down, z_down, intensity_down)
    """
    elevations = calculate_elevation_angle(x, y, z)

    # Asignar cada punto a su anillo HDL-64E más cercano usando bins fijos.
    hdl_ring = np.digitize(elevations, _HDL64E_BIN_EDGES) - 1
    hdl_ring = np.clip(hdl_ring, 0, 63)

    # Determinar qué anillos HDL-64E corresponden a los beams VLP-16 pedidos.
    vlp16_angles = VLP16_ELEVATIONS[:num_rings]
    target_rings = np.array([
        int(np.argmin(np.abs(HDL64E_ELEVATIONS - a))) for a in vlp16_angles
    ])
    # Deduplicar manteniendo orden (beams VLP-16 fuera del FOV HDL-64E colapsan).
    seen = set()
    selected_rings = []
    for r in target_rings:
        if r not in seen:
            seen.add(r)
            selected_rings.append(r)
    selected_rings = np.array(selected_rings)

    if verbose:
        print(f"Rango de elevación de los datos: "
              f"{elevations.min():.2f}° a {elevations.max():.2f}°")
        print(f"Anillos HDL-64E seleccionados ({len(selected_rings)} únicos de {num_rings} pedidos):")
        for i, ring_idx in enumerate(selected_rings):
            vlp_angle = vlp16_angles[i] if i < len(vlp16_angles) else "—"
            hdl_angle = HDL64E_ELEVATIONS[ring_idx]
            print(f"  VLP-16 beam {i:2d} ({vlp_angle:+.0f}°) → "
                  f"HDL-64E ring {ring_idx:2d} ({hdl_angle:+.2f}°)")
        if len(selected_rings) < num_rings:
            print(f"  [{num_rings - len(selected_rings)} beams VLP-16 fuera del "
                  f"FOV HDL-64E colapsaron al anillo superior]")

    mask = np.isin(hdl_ring, selected_rings)
    return x[mask], y[mask], z[mask], intensity[mask]


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
