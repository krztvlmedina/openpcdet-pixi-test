#!/usr/bin/env python3
"""
generate_annotated_images.py — Generate annotated KITTI camera images.

Reads ground-truth labels and/or pre-computed detection results, draws 2D
bounding boxes on the corresponding camera images, and writes three output
subdirectories:

  <output-dir>/gt_only/           GT boxes only
  <output-dir>/det_only/          All detection boxes (no threshold filter)
  <output-dir>/det_thresholded/   Detection boxes filtered by per-class score
                                  thresholds from --threshold-config

KITTI detection files (from OpenPCDet --save_to_file) use camera-frame
coordinates; calibration is required to extract the 2D pixel bounding box
(fields 4-7 of the KITTI label format).

Usage:
  python3 generate_annotated_images.py \\
      --image-dir  /data/kitti/training/image_2 \\
      --label-dir  /data/kitti/training/label_2 \\
      --det-dir    /OpenPCDet/output/original/pv_rcnn/final_result/data \\
      --output-dir /data/output_runs/annotated/pv_rcnn \\
      --threshold-config /data/confs/thresholds.yaml

  # GT-only (no --det-dir needed):
  python3 generate_annotated_images.py \\
      --image-dir  /data/kitti/training/image_2 \\
      --label-dir  /data/kitti/training/label_2 \\
      --output-dir /data/output_runs/annotated/gt
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional, Any

import cv2
import numpy as np

try:
    import yaml as _yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False


# ─── Per-class box colours (BGR for OpenCV) ──────────────────────────────────

_GT_COLOR: Dict[str, tuple] = {
    'Car':            (255, 140,  51),
    'Van':            (255,  51, 140),
    'Truck':          ( 26, 204, 255),
    'Pedestrian':     (115, 255,  51),
    'Person_sitting': (115, 255,  51),
    'Cyclist':        ( 26, 140, 255),
    'Tram':           (255, 127, 127),
    'Misc':           (180, 180, 180),
}

_DET_COLOR: Dict[str, tuple] = {
    'Car':            ( 51,  76, 255),
    'Van':            (  0, 140, 255),
    'Truck':          (  0, 178, 229),
    'Pedestrian':     (178,  51, 255),
    'Person_sitting': (178,  51, 255),
    'Cyclist':        (  0, 255, 204),
    'Tram':           (204, 127, 204),
    'Misc':           (217, 217, 217),
}

_DEFAULT_GT_COLOR  = (200, 200, 200)
_DEFAULT_DET_COLOR = (  0, 153, 255)


# ─── KITTI label parsing ──────────────────────────────────────────────────────

def _parse_kitti_boxes(txt_path: Path, score_field: bool) -> List[Dict[str, Any]]:
    """
    Parse KITTI-format annotation or detection file.
    Only the 2D bbox fields (4-7) are used for image annotation.

    Lidar-frame format (9 fields, from ros2_node.py): no 2D bbox — skipped.
    KITTI camera format (>=15 fields): fields 4-7 are pixel coords x1 y1 x2 y2.
    """
    boxes = []
    with open(txt_path) as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            cls = parts[0]
            if cls == 'DontCare':
                continue
            if len(parts) < 15:
                continue  # lidar-format or incomplete — no 2D bbox available

            x1, y1, x2, y2 = (float(parts[4]), float(parts[5]),
                               float(parts[6]), float(parts[7]))
            score = float(parts[15]) if (score_field and len(parts) > 15) else 1.0
            boxes.append({
                'type':  cls,
                'bbox2d': np.array([x1, y1, x2, y2]),
                'score': score,
            })
    return boxes


# ─── Drawing helper ───────────────────────────────────────────────────────────

def _draw_boxes(img: np.ndarray, boxes: List[Dict[str, Any]],
                color_map: Dict[str, tuple], default_color: tuple,
                thickness: int = 2) -> np.ndarray:
    """Draw 2D bounding boxes on a copy of img. Returns the annotated copy."""
    out = img.copy()
    for box in boxes:
        bbox = box.get('bbox2d')
        if bbox is None:
            continue
        x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cls   = box.get('type', '')
        score = box.get('score', 1.0)
        bgr   = color_map.get(cls, default_color)
        cv2.rectangle(out, (x1, y1), (x2, y2), bgr, thickness)
        label = f'{cls} {score:.2f}' if score < 1.0 else cls
        cv2.putText(out, label, (x1, max(y1 - 4, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1, cv2.LINE_AA)
    return out


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Generate annotated KITTI camera images (gt_only / det_only / '
                    'det_thresholded) from pre-computed detection results.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--image-dir', required=True, metavar='DIR',
                        help='Source KITTI image_2 directory (.png/.jpg)')
    parser.add_argument('--gt', action='store_true',
                        help='Generate gt_only/ variant (requires --label-dir)')
    parser.add_argument('--label-dir', metavar='DIR',
                        help='KITTI label_2 directory (ground-truth boxes); '
                             'only used when --gt is given')
    parser.add_argument('--det-dir', metavar='DIR',
                        help='Detection results directory (KITTI camera-format .txt '
                             'files, e.g. final_result/data from OpenPCDet)')
    parser.add_argument('--output-dir', required=True, metavar='DIR',
                        help='Root output directory; subdirs gt_only/, det_only/, '
                             'det_thresholded/ are created automatically')
    parser.add_argument('--threshold-config', metavar='FILE',
                        help='YAML file with per-class confidence thresholds for '
                             'det_thresholded/ variant (e.g. Car: 0.5)')
    parser.add_argument('--ext', default='.png',
                        help='Output image extension (default: .png)')
    args = parser.parse_args()

    image_dir  = Path(args.image_dir)
    output_dir = Path(args.output_dir)
    det_dir    = Path(args.det_dir) if args.det_dir else None

    if args.gt and not args.label_dir:
        parser.error('--gt requires --label-dir.')
    label_dir = Path(args.label_dir) if (args.gt and args.label_dir) else None

    if not args.gt and det_dir is None:
        parser.error('Provide --det-dir, or --gt with --label-dir.')

    # Load thresholds
    thresholds: Dict[str, float] = {}
    if args.threshold_config:
        if not _YAML_AVAILABLE:
            print('ERROR: --threshold-config requires PyYAML (pip install pyyaml).',
                  file=sys.stderr)
            sys.exit(1)
        with open(args.threshold_config) as f:
            thresholds = _yaml.safe_load(f) or {}
        print(f'Thresholds: {thresholds}')

    # Create output subdirectories
    out_gt    = output_dir / 'gt_only'
    out_det   = output_dir / 'det_only'
    out_thr   = output_dir / 'det_thresholded'

    if label_dir:  # only when --gt was passed
        out_gt.mkdir(parents=True, exist_ok=True)
    if det_dir:
        out_det.mkdir(parents=True, exist_ok=True)
        out_thr.mkdir(parents=True, exist_ok=True)

    # Collect frame stems from image dir
    stems = sorted(
        p.stem for p in image_dir.iterdir()
        if p.suffix.lower() in ('.png', '.jpg', '.jpeg')
    )
    if not stems:
        print(f'No images found in {image_dir}', file=sys.stderr)
        sys.exit(1)

    print(f'Processing {len(stems)} frames...')
    for fid in stems:
        # Load source image
        img_path = image_dir / f'{fid}.png'
        if not img_path.exists():
            img_path = image_dir / f'{fid}.jpg'
        if not img_path.exists():
            continue

        img = cv2.imread(str(img_path))
        if img is None:
            print(f'  WARNING: could not read {img_path}', file=sys.stderr)
            continue

        ext = args.ext if args.ext.startswith('.') else f'.{args.ext}'

        # ── gt_only ──────────────────────────────────────────────────────────
        if label_dir:
            lp = label_dir / f'{fid}.txt'
            gt_boxes = _parse_kitti_boxes(lp, score_field=False) if lp.exists() else []
            annotated = _draw_boxes(img, gt_boxes, _GT_COLOR, _DEFAULT_GT_COLOR)
            cv2.imwrite(str(out_gt / f'{fid}{ext}'), annotated)

        # ── det_only & det_thresholded ────────────────────────────────────────
        if det_dir:
            dp = det_dir / f'{fid}.txt'
            det_boxes = _parse_kitti_boxes(dp, score_field=True) if dp.exists() else []

            # det_only — all detections regardless of score
            annotated = _draw_boxes(img, det_boxes, _DET_COLOR, _DEFAULT_DET_COLOR)
            cv2.imwrite(str(out_det / f'{fid}{ext}'), annotated)

            # det_thresholded — per-class score filter
            if thresholds:
                filtered = [
                    b for b in det_boxes
                    if b.get('score', 1.0) >= thresholds.get(b.get('type', ''), 0.0)
                ]
            else:
                filtered = det_boxes
            annotated = _draw_boxes(img, filtered, _DET_COLOR, _DEFAULT_DET_COLOR)
            cv2.imwrite(str(out_thr / f'{fid}{ext}'), annotated)

    print('Done.')
    if label_dir:
        print(f'  GT images:           {out_gt}')
    if det_dir:
        print(f'  Det images:          {out_det}')
        print(f'  Thresholded images:  {out_thr}')


if __name__ == '__main__':
    main()
