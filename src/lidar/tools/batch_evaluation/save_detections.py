#!/usr/bin/env python3
"""
save_detections.py — Batch detection runner for kitti_inspector visualisation.

Iterates every .bin file in a directory in sorted order, runs OpenPCDet
inference on each one, and writes per-frame results as simple lidar-frame
txt files that kitti_inspector can load with --det-dir.

Key differences from run_realtime_eval.sh:
  • No ROS2 — no subscriber queue, no frame dropping.
  • Every frame is processed exactly once, in order.
  • Output files are named {stem}.txt matching the original .bin stems,
    so kitti_inspector navigation aligns correctly with the point clouds.

Output format (one line per detected box):
  <class> <cx> <cy> <cz> <l> <w> <h> <heading> <score>
  (all in lidar frame; OpenPCDet [x, y, z, dx, dy, dz, heading] convention)

Usage:
  python3 save_detections.py \\
      --cfg_file  cfgs/kitti_models/interpolated/pv_rcnn.yaml \\
      --ckpt      data/pretrained-models/pv_rcnn_8369.pth \\
      --bin-dir   /data/kitti/training/velodyne_nearest_extreme \\
      --output-dir /results/pv_rcnn/nearest_extreme

  # kitti_inspector usage afterwards:
  python3 kitti_inspector.py \\
      --original-dir  /data/kitti/training/velodyne \\
      --interp-dir    nearest_extreme /data/kitti/training/velodyne_nearest_extreme \\
      --label-dir     /data/kitti/training/label_2 \\
      --calib-dir     /data/kitti/training/calib \\
      --det-dir       "pv_rcnn / nearest_extreme" /results/pv_rcnn/nearest_extreme
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils


# ─── Minimal single-frame dataset ────────────────────────────────────────────

class SingleFrameDataset(DatasetTemplate):
    """
    Minimal DatasetTemplate wrapper for processing individual point clouds.
    Mirrors the Ros2Dataset used in ros2_node.py, but without any ROS2 dependency.
    """

    def __init__(self, dataset_cfg, class_names, logger=None):
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=False,
            root_path=Path('.'),
            logger=logger,
        )
        self._points: np.ndarray = None

    def set_points(self, points: np.ndarray):
        self._points = points

    def __len__(self):
        return 1

    def __getitem__(self, index):
        if self._points is None:
            raise RuntimeError('No point cloud loaded. Call set_points() first.')
        return self.prepare_data({'points': self._points, 'frame_id': 0})


# ─── I/O helpers ─────────────────────────────────────────────────────────────

def load_bin(path: Path) -> np.ndarray:
    """Read a KITTI-format .bin file → Nx4 float32 array (x, y, z, intensity)."""
    return np.fromfile(str(path), dtype=np.float32).reshape(-1, 4)


def run_inference(model, dataset, points: np.ndarray):
    """
    Run OpenPCDet inference on a single point cloud.
    Returns (pred_boxes, pred_scores, pred_labels) as numpy arrays,
    or raises an exception on failure.
    """
    dataset.set_points(points)
    with torch.no_grad():
        data_dict = dataset[0]
        data_dict = dataset.collate_batch([data_dict])
        load_data_to_gpu(data_dict)
        pred_dicts, _ = model.forward(data_dict)

    boxes  = pred_dicts[0]['pred_boxes'].cpu().numpy()
    scores = pred_dicts[0]['pred_scores'].cpu().numpy()
    labels = pred_dicts[0]['pred_labels'].cpu().numpy()
    return boxes, scores, labels


def write_detections(out_path: Path, boxes, scores, labels,
                     class_names, score_threshold: float):
    """
    Write detection results to a lidar-frame txt file.
    Lines with score < score_threshold are omitted.
    An empty file is written when nothing passes the threshold.
    """
    lines = []
    for box, score, label in zip(boxes, scores, labels):
        if float(score) < score_threshold:
            continue
        cls = class_names[int(label) - 1]
        # OpenPCDet box layout: [x, y, z, dx(l), dy(w), dz(h), heading]
        lines.append(
            f'{cls} '
            f'{float(box[0]):.4f} {float(box[1]):.4f} {float(box[2]):.4f} '
            f'{float(box[3]):.4f} {float(box[4]):.4f} {float(box[5]):.4f} '
            f'{float(box[6]):.6f} {float(score):.4f}'
        )
    out_path.write_text('\n'.join(lines))
    return len(lines)


# ─── Main ─────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description='Batch detection runner — produces per-frame lidar-frame '
                    'txt files compatible with kitti_inspector --det-dir',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--cfg_file', required=True,
                        help='OpenPCDet model config YAML')
    parser.add_argument('--ckpt', required=True,
                        help='Model checkpoint (.pth)')
    parser.add_argument('--bin-dir', required=True, metavar='DIR',
                        help='Directory of input .bin point cloud files')
    parser.add_argument('--output-dir', required=True, metavar='DIR',
                        help='Directory to write per-frame detection .txt files')
    parser.add_argument('--score-threshold', type=float, default=0.3,
                        help='Minimum confidence score to save (default: 0.3)')
    return parser.parse_args()


def main():
    args = parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)

    bin_dir    = Path(args.bin_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bin_files = sorted(bin_dir.glob('*.bin'))
    if not bin_files:
        print(f'ERROR: no .bin files found in {bin_dir}', file=sys.stderr)
        sys.exit(1)

    logger = common_utils.create_logger()
    logger.info(f'Config:          {args.cfg_file}')
    logger.info(f'Checkpoint:      {args.ckpt}')
    logger.info(f'Input dir:       {bin_dir}  ({len(bin_files)} files)')
    logger.info(f'Output dir:      {output_dir}')
    logger.info(f'Score threshold: {args.score_threshold}')

    dataset = SingleFrameDataset(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        logger=logger,
    )

    model = build_network(
        model_cfg=cfg.MODEL,
        num_class=len(cfg.CLASS_NAMES),
        dataset=dataset,
    )
    model.load_params_from_file(filename=args.ckpt, logger=logger, to_cpu=True)
    model.cuda()
    model.eval()
    logger.info('Model loaded — starting inference.')

    n_total   = len(bin_files)
    n_ok      = 0   # frames with ≥1 detection above threshold
    n_empty   = 0   # frames with no detection above threshold
    n_errors  = 0   # frames that failed to process

    for i, bin_path in enumerate(bin_files):
        stem = bin_path.stem
        out_path = output_dir / f'{stem}.txt'

        try:
            points = load_bin(bin_path)
            if len(points) == 0:
                raise ValueError('empty point cloud')

            boxes, scores, labels = run_inference(model, dataset, points)
            n_saved = write_detections(
                out_path, boxes, scores, labels,
                cfg.CLASS_NAMES, args.score_threshold,
            )

            if n_saved > 0:
                n_ok += 1
            else:
                n_empty += 1

            logger.info(f'[{i + 1}/{n_total}] {stem}: {n_saved} detections')

        except Exception as exc:
            logger.warning(f'[{i + 1}/{n_total}] {stem}: ERROR — {exc}')
            # Write an empty file so kitti_inspector shows no detections rather
            # than falling back to a stale result from a previous run.
            out_path.write_text('')
            n_errors += 1

    logger.info(
        f'Finished. {n_ok} with detections, {n_empty} empty, '
        f'{n_errors} errors. Output: {output_dir}'
    )


if __name__ == '__main__':
    main()
