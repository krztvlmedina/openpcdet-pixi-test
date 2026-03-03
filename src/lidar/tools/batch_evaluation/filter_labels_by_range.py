#!/usr/bin/env python3
"""
filter_labels_by_range.py — Filter KITTI label_2 files by maximum sensor range.

Objects whose 3-D Euclidean distance from the camera exceeds --max-distance are
removed from each label file.  DontCare annotations are kept by default (they
mark regions that were not labelled, regardless of depth).

Distance is computed in the camera (rectified) coordinate frame:
    dist = sqrt(x_cam² + y_cam² + z_cam²)
where (x_cam, y_cam, z_cam) = fields 11-13 of each KITTI label line.

This approximates the LiDAR sensor range; the camera and LiDAR are co-located
within ~0.3 m on the KITTI platform, so the error is negligible for typical
range thresholds (20–100 m).

KITTI label format (one object per line):
  type  truncated  occluded  alpha  x1 y1 x2 y2  h w l  x y z  ry  [score]
  [0]   [1]        [2]       [3]    [4-7]         [8-10] [11-13][14] [15]

Usage:
  python3 filter_labels_by_range.py \\
      --input-dir  /data/kitti/training/label_2 \\
      --output-dir /data/kitti-50m/training/label_2 \\
      --max-distance 50.0

  # Keep DontCare within range only (strip far DontCare too):
  python3 filter_labels_by_range.py \\
      --input-dir  /data/kitti/training/label_2 \\
      --output-dir /data/kitti-50m/training/label_2 \\
      --max-distance 50.0 --filter-dontcare
"""

import argparse
import sys
from pathlib import Path


# Classes whose distance should never be used for filtering (they are
# "unknown-label" regions, not real detections at a specific depth).
_DONTCARE = {'DontCare'}


def parse_label_line(line: str):
    """
    Parse one KITTI label line.

    Returns (cls_type, x_cam, y_cam, z_cam, raw_line) or None for blank/comment.
    """
    stripped = line.rstrip('\n')
    if not stripped or stripped.startswith('#'):
        return None
    parts = stripped.split()
    if len(parts) < 15:
        # Malformed line — pass through unchanged rather than silently drop it.
        return None
    cls_type = parts[0]
    try:
        x_cam = float(parts[11])
        y_cam = float(parts[12])
        z_cam = float(parts[13])
    except ValueError:
        return None
    return cls_type, x_cam, y_cam, z_cam, stripped


def filter_file(input_path: Path, output_path: Path,
                max_distance: float, filter_dontcare: bool) -> tuple[int, int]:
    """
    Filter one label file.

    Returns (total_objects, kept_objects).
    """
    lines = input_path.read_text().splitlines(keepends=False)
    kept = []
    total = 0

    for line in lines:
        parsed = parse_label_line(line)
        if parsed is None:
            # Blank or malformed — preserve as-is.
            kept.append(line)
            continue

        cls_type, x, y, z, raw = parsed
        total += 1

        is_dontcare = cls_type in _DONTCARE
        if is_dontcare and not filter_dontcare:
            # Always keep DontCare unless explicitly asked to filter them.
            kept.append(raw)
            continue

        dist = (x * x + y * y + z * z) ** 0.5
        if dist <= max_distance:
            kept.append(raw)

    output_path.write_text('\n'.join(kept) + ('\n' if kept else ''))
    kept_objects = sum(
        1 for line in kept
        if line and not line.startswith('#') and len(line.split()) >= 15
    )
    return total, kept_objects


def main():
    parser = argparse.ArgumentParser(
        description='Filter KITTI label_2 files by maximum 3-D sensor range.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--input-dir', required=True, metavar='DIR',
                        help='Source label_2 directory (*.txt files)')
    parser.add_argument('--output-dir', required=True, metavar='DIR',
                        help='Destination directory for filtered label files')
    parser.add_argument('--max-distance', required=True, type=float, metavar='M',
                        help='Maximum 3-D Euclidean distance from sensor (metres)')
    parser.add_argument('--filter-dontcare', action='store_true',
                        help='Also apply range filter to DontCare annotations '
                             '(default: keep all DontCare)')
    args = parser.parse_args()

    input_dir  = Path(args.input_dir)
    output_dir = Path(args.output_dir)

    if not input_dir.exists():
        print(f'ERROR: input directory not found: {input_dir}', file=sys.stderr)
        sys.exit(1)

    label_files = sorted(input_dir.glob('*.txt'))
    if not label_files:
        print(f'ERROR: no .txt files found in {input_dir}', file=sys.stderr)
        sys.exit(1)

    output_dir.mkdir(parents=True, exist_ok=True)

    total_objs = 0
    kept_objs  = 0
    n_files    = 0

    for src in label_files:
        dst = output_dir / src.name
        t, k = filter_file(src, dst, args.max_distance, args.filter_dontcare)
        total_objs += t
        kept_objs  += k
        n_files    += 1

    removed = total_objs - kept_objs
    pct_kept = 100.0 * kept_objs / total_objs if total_objs else 0.0

    print(f'Processed {n_files} files  |  max distance: {args.max_distance} m')
    print(f'  Total objects : {total_objs}')
    print(f'  Kept          : {kept_objs}  ({pct_kept:.1f}%)')
    print(f'  Removed       : {removed}')
    print(f'  Output        : {output_dir}')


if __name__ == '__main__':
    main()
