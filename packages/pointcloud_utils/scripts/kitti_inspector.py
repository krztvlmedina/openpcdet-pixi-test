#!/usr/bin/env python3
"""
kitti_inspector.py — Interactive KITTI point cloud inspector for RViz.

Publishes up to three point clouds (original / reduced / one interpolated at a
time), ground-truth + pre-computed detection bounding boxes, and a pre-generated
camera image for a selected KITTI frame.  Multiple interpolation datasets and
multiple pre-computed detection sets can be loaded simultaneously and cycled
through interactively.

Images are expected to be pre-generated (e.g. by generate_annotated_images.py)
and are published as-is — no drawing is done at runtime.

Topics published:
  /cloud/original          sensor_msgs/PointCloud2  (if --original-dir given)
  /cloud/reduced           sensor_msgs/PointCloud2  (if --reduced-dir  given)
  /cloud/interpolated      sensor_msgs/PointCloud2  (one interp at a time)
  /markers/groundtruth     visualization_msgs/MarkerArray
  /markers/detections      visualization_msgs/MarkerArray  (loaded from txt)
  /image/annotated         sensor_msgs/Image         (if --image-dir given)

3D detection markers are filtered by per-class confidence thresholds when
--threshold-config is provided.  The threshold config is a YAML file of the
form:
  Car: 0.5
  Pedestrian: 0.5
  Cyclist: 0.5

Interactive commands:
  n / <Enter>   next frame
  p             previous frame
  <number>      jump to frame by list index (0-based) or 6-digit KITTI id
  i             next interpolation dataset
  I             previous interpolation dataset
  d             next detection set (model / dataset)
  D             previous detection set
  r             refresh / republish current frame
  q             quit

Usage example:
  python3 kitti_inspector.py \\
      --original-dir /data/kitti/training/velodyne \\
      --reduced-dir  /data/kitti/training/velodyne_16ch \\
      --interp-dir nearest_extreme /data/kitti/training/velodyne_nearest_extreme \\
      --interp-dir linear_01       /data/kitti/training/velodyne_linear_01 \\
      --label-dir   /data/kitti/training/label_2 \\
      --calib-dir   /data/kitti/training/calib \\
      --image-dir   /path/to/annotated_images/det_thresholded \\
      --threshold-config /path/to/thresholds.yaml \\
      --det-dir "pv_rcnn / nearest_extreme"  /path/to/pv_rcnn/nearest_extreme/final_result/data \\
      --det-dir "pointpillar / nearest_extreme" /path/to/pointpillar/nearest_extreme/final_result/data
"""

import argparse
import threading
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

try:
    import yaml as _yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False


# ─── Per-class box colours (RGB 0–1) ─────────────────────────────────────────

_GT_COLOR: Dict[str, tuple] = {
    'Car':            (0.20, 0.55, 1.00),
    'Van':            (0.55, 0.20, 1.00),
    'Truck':          (1.00, 0.80, 0.10),
    'Pedestrian':     (0.20, 1.00, 0.45),
    'Person_sitting': (0.20, 1.00, 0.45),
    'Cyclist':        (1.00, 0.55, 0.10),
    'Tram':           (0.50, 0.50, 1.00),
    'Misc':           (0.70, 0.70, 0.70),
}

# Detections use warmer / shifted tints to distinguish from GT
_DET_COLOR: Dict[str, tuple] = {
    'Car':            (1.00, 0.30, 0.20),
    'Van':            (1.00, 0.55, 0.00),
    'Truck':          (0.90, 0.70, 0.00),
    'Pedestrian':     (1.00, 0.20, 0.70),
    'Person_sitting': (1.00, 0.20, 0.70),
    'Cyclist':        (0.80, 1.00, 0.00),
    'Tram':           (0.80, 0.50, 1.00),
    'Misc':           (0.85, 0.85, 0.85),
}
_DEFAULT_GT_COLOR  = (0.80, 0.80, 0.80)
_DEFAULT_DET_COLOR = (1.00, 0.60, 0.00)


# ─── KITTI calibration ───────────────────────────────────────────────────────

class KittiCalib:
    """
    Minimal KITTI calib reader.
    Provides a single transform: rectified-camera frame → lidar frame.
    """

    def __init__(self, path: Path):
        data: Dict[str, np.ndarray] = {}
        with open(path) as f:
            for line in f:
                if ':' not in line:
                    continue
                key, val = line.split(':', 1)
                data[key.strip()] = np.fromstring(val, sep=' ')

        R0 = data['R0_rect'].reshape(3, 3)
        Tr = data['Tr_velo_to_cam'].reshape(3, 4)

        R0_4       = np.eye(4);  R0_4[:3, :3] = R0
        Tr_4       = np.eye(4);  Tr_4[:3, :]  = Tr
        self._M = np.linalg.inv(Tr_4) @ np.linalg.inv(R0_4)

    def rect_to_lidar(self, pts: np.ndarray) -> np.ndarray:
        """Convert Nx3 points from rectified camera to lidar frame."""
        n = len(pts)
        pts_h = np.hstack([pts, np.ones((n, 1))])
        return (self._M @ pts_h.T).T[:, :3]


# ─── KITTI annotation helpers ─────────────────────────────────────────────────

def _parse_kitti_boxes(txt_path: Path, calib: Optional['KittiCalib'],
                       score_field: bool = False) -> List[Dict[str, Any]]:
    """
    Parse an annotation / detection file.  Two formats are recognised:

    Lidar format (9 fields) — written by ros2_node.py:
      class  cx  cy  cz  l  w  h  heading  score
      All values already in lidar frame; no calib needed.

    KITTI camera format (≥15 fields) — written by OpenPCDet test.py:
      type trunc occ alpha  x1 y1 x2 y2  h w l  x y z  ry  [score]
      Rectified-camera frame; requires calib for conversion.
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

            if len(parts) == 9:
                # ── Lidar-frame format ─────────────────────────────────────
                cx, cy, cz = float(parts[1]), float(parts[2]), float(parts[3])
                l, w, h    = float(parts[4]), float(parts[5]), float(parts[6])
                heading    = float(parts[7])
                score      = float(parts[8])
                boxes.append({
                    'type':    cls,
                    'center':  np.array([cx, cy, cz]),
                    'lwh':     np.array([l, w, h]),
                    'heading': heading,
                    'score':   score,
                })

            elif len(parts) >= 15:
                # ── KITTI camera format ────────────────────────────────────
                if calib is None:
                    continue   # can't convert without calibration
                # fields 4-7: 2D bounding box in image pixel coordinates
                x1, y1, x2, y2 = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                h, w, l     = float(parts[8]),  float(parts[9]),  float(parts[10])
                x, y, z, ry = float(parts[11]), float(parts[12]), float(parts[13]), float(parts[14])
                score        = float(parts[15]) if (score_field and len(parts) > 15) else 1.0

                # (x, y, z) is the bottom-face center in rectified camera coords
                # (y points down).  Subtract h/2 here to get the geometric centre
                # before transforming — no further correction needed after.
                center_cam = np.array([[x, y - h / 2, z]])
                center_lid = calib.rect_to_lidar(center_cam)[0]
                heading = -(np.pi / 2 + ry)

                boxes.append({
                    'type':    cls,
                    'center':  center_lid,
                    'lwh':     np.array([l, w, h]),
                    'heading': heading,
                    'score':   score,
                    'bbox2d':  np.array([x1, y1, x2, y2]),
                })
    return boxes


def load_gt_boxes(label_path: Path, calib: 'KittiCalib') -> List[Dict[str, Any]]:
    return _parse_kitti_boxes(label_path, calib, score_field=False)


def load_det_boxes(det_path: Path, calib: Optional['KittiCalib'] = None) -> List[Dict[str, Any]]:
    """Load detection results.  calib only needed for KITTI camera-format files."""
    return _parse_kitti_boxes(det_path, calib, score_field=True)


# ─── PointCloud2 helper ───────────────────────────────────────────────────────

_PC_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


def bin_to_pc2(path: Path, frame_id: str, stamp) -> PointCloud2:
    pts = np.fromfile(str(path), dtype=np.float32).reshape(-1, 4)
    hdr = Header()
    hdr.frame_id = frame_id
    hdr.stamp    = stamp
    return point_cloud2.create_cloud(hdr, _PC_FIELDS, pts)


# ─── MarkerArray helpers ──────────────────────────────────────────────────────

def boxes_to_marker_array(boxes: List[Dict], frame_id: str, stamp,
                           ns: str, color_map: Dict[str, tuple],
                           default_color: tuple,
                           fill_alpha: float = 0.20) -> MarkerArray:
    """Convert lidar-frame box dicts to a MarkerArray (filled cube + text label)."""
    ma = MarkerArray()
    for i, box in enumerate(boxes):
        cx, cy, cz = box['center'].tolist()
        l, w, h    = box['lwh'].tolist()
        heading    = float(box['heading'])
        cls        = box.get('type', '')
        score      = box.get('score', 1.0)
        r, g, b    = color_map.get(cls, default_color)

        # Filled box
        m = Marker()
        m.header.frame_id    = frame_id
        m.header.stamp       = stamp
        m.ns                 = ns
        m.id                 = i * 2
        m.type               = Marker.CUBE
        m.action             = Marker.ADD
        m.pose.position.x    = cx
        m.pose.position.y    = cy
        m.pose.position.z    = cz
        m.pose.orientation.z = float(np.sin(heading / 2))
        m.pose.orientation.w = float(np.cos(heading / 2))
        m.scale.x            = l
        m.scale.y            = w
        m.scale.z            = h
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, fill_alpha
        m.lifetime.sec = m.lifetime.nanosec = 0
        ma.markers.append(m)

        # Text label
        t = Marker()
        t.header.frame_id    = frame_id
        t.header.stamp       = stamp
        t.ns                 = ns + '_labels'
        t.id                 = i * 2 + 1
        t.type               = Marker.TEXT_VIEW_FACING
        t.action             = Marker.ADD
        t.pose.position.x    = cx
        t.pose.position.y    = cy
        t.pose.position.z    = cz + h / 2 + 0.3
        t.scale.z            = 0.4
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text               = f'{cls} {score:.2f}' if score < 1.0 else cls
        t.lifetime.sec = t.lifetime.nanosec = 0
        ma.markers.append(t)

    return ma


def _deleteall_ma(ns: str, frame_id: str, stamp) -> MarkerArray:
    ma = MarkerArray()
    for namespace in (ns, ns + '_labels'):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp    = stamp
        m.ns              = namespace
        m.action          = Marker.DELETEALL
        ma.markers.append(m)
    return ma


# ─── Inspector node ───────────────────────────────────────────────────────────

class InspectorNode(Node):

    def __init__(self, args):
        super().__init__('kitti_inspector')

        self.frame_id = args.frame_id

        self._orig_dir    = Path(args.original_dir) if args.original_dir else None
        self._reduced_dir = Path(args.reduced_dir)  if args.reduced_dir  else None

        # Ordered list of (name, Path) for interpolated datasets
        self._interp_dirs: List[Tuple[str, Path]] = []
        if args.interp_dir:
            for name, d in args.interp_dir:
                self._interp_dirs.append((name, Path(d)))
        self._interp_idx = 0

        # Ordered list of (label, Path) for pre-computed detection txt dirs
        self._det_sets: List[Tuple[str, Path]] = []
        if args.det_dir:
            for label, d in args.det_dir:
                self._det_sets.append((label, Path(d)))
        self._det_idx = 0

        self._label_dir = Path(args.label_dir) if args.label_dir else None
        self._calib_dir = Path(args.calib_dir) if args.calib_dir else None
        self._image_dir = Path(args.image_dir) if args.image_dir else None

        # Per-class confidence thresholds for 3D marker display
        self._thresholds: Dict[str, float] = {}
        if args.threshold_config:
            if not _YAML_AVAILABLE:
                import sys
                print('WARNING: --threshold-config given but PyYAML is not installed; '
                      'thresholds will not be applied.', file=sys.stderr)
            else:
                with open(args.threshold_config) as _f:
                    self._thresholds = _yaml.safe_load(_f) or {}

        self._frames = self._collect_frames()
        if not self._frames:
            self.get_logger().error('No .bin files found in any input directory.')
            raise SystemExit(1)
        self.get_logger().info(f'Loaded {len(self._frames)} frames.')

        if self._interp_dirs:
            self.get_logger().info(
                f'Interpolation sets ({len(self._interp_dirs)}): '
                + ', '.join(n for n, _ in self._interp_dirs))
        if self._det_sets:
            self.get_logger().info(
                f'Detection sets ({len(self._det_sets)}): '
                + ', '.join(l for l, _ in self._det_sets))

        self._idx = 0

        _latch = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._orig_pub   = (self.create_publisher(PointCloud2, '/cloud/original',     _latch)
                            if self._orig_dir    else None)
        self._red_pub    = (self.create_publisher(PointCloud2, '/cloud/reduced',      _latch)
                            if self._reduced_dir else None)
        self._interp_pub = (self.create_publisher(PointCloud2, '/cloud/interpolated', _latch)
                            if self._interp_dirs else None)

        self._gt_pub  = self.create_publisher(MarkerArray, '/markers/groundtruth', _latch)
        self._det_pub = self.create_publisher(MarkerArray, '/markers/detections',  _latch)
        self._img_pub = (self.create_publisher(Image, '/image/annotated', _latch)
                         if self._image_dir else None)

        if self._image_dir and not _CV2_AVAILABLE:
            self.get_logger().warn('--image-dir given but cv2 is not available; '
                                   'image publishing disabled.')
            self._img_pub = None

        if self._thresholds:
            self.get_logger().info(
                'Detection thresholds: '
                + ', '.join(f'{k}={v}' for k, v in sorted(self._thresholds.items())))

        self.publish_current()

    # ── Frame collection ──────────────────────────────────────────────────────

    def _collect_frames(self) -> List[str]:
        stems: set = set()
        all_dirs = [self._orig_dir, self._reduced_dir] + [p for _, p in self._interp_dirs]
        for d in all_dirs:
            if d and d.is_dir():
                stems.update(p.stem for p in d.glob('*.bin'))
        return sorted(stems)

    # ── Publishing ────────────────────────────────────────────────────────────

    def publish_current(self):
        fid   = self._frames[self._idx]
        stamp = self.get_clock().now().to_msg()

        # Original cloud
        if self._orig_pub and self._orig_dir:
            p = self._orig_dir / f'{fid}.bin'
            if p.exists():
                try:
                    self._orig_pub.publish(bin_to_pc2(p, self.frame_id, stamp))
                except Exception as exc:
                    self.get_logger().warn(f'original {fid}: {exc}')

        # Reduced cloud
        if self._red_pub and self._reduced_dir:
            p = self._reduced_dir / f'{fid}.bin'
            if p.exists():
                try:
                    self._red_pub.publish(bin_to_pc2(p, self.frame_id, stamp))
                except Exception as exc:
                    self.get_logger().warn(f'reduced {fid}: {exc}')

        # Current interpolated cloud
        interp_label = '—'
        if self._interp_pub and self._interp_dirs:
            name, d = self._interp_dirs[self._interp_idx]
            p = d / f'{fid}.bin'
            if p.exists():
                try:
                    self._interp_pub.publish(bin_to_pc2(p, self.frame_id, stamp))
                except Exception as exc:
                    self.get_logger().warn(f'interp {fid}: {exc}')
            interp_label = f'{name} [{self._interp_idx + 1}/{len(self._interp_dirs)}]'

        # Ground-truth boxes
        gt_boxes = []
        if self._label_dir and self._calib_dir:
            lp = self._label_dir / f'{fid}.txt'
            cp = self._calib_dir / f'{fid}.txt'
            if lp.exists() and cp.exists():
                try:
                    calib = KittiCalib(cp)
                    gt_boxes = load_gt_boxes(lp, calib)
                    self._gt_pub.publish(
                        boxes_to_marker_array(gt_boxes, self.frame_id, stamp,
                                              'gt', _GT_COLOR, _DEFAULT_GT_COLOR,
                                              fill_alpha=0.18))
                except Exception as exc:
                    self.get_logger().warn(f'GT {fid}: {exc}')
            else:
                self._gt_pub.publish(_deleteall_ma('gt', self.frame_id, stamp))

        # Pre-computed detections
        det_boxes = []
        det_label = '—'
        if self._det_sets:
            label, det_dir = self._det_sets[self._det_idx]
            det_txt = det_dir / f'{fid}.txt'
            if det_txt.exists():
                try:
                    # calib is only required for KITTI camera-format files;
                    # lidar-format files (from ros2_node.py) need no conversion.
                    calib = None
                    if self._calib_dir:
                        cp = self._calib_dir / f'{fid}.txt'
                        if cp.exists():
                            calib = KittiCalib(cp)
                    det_boxes = load_det_boxes(det_txt, calib)
                    # Apply per-class confidence thresholds for 3D display
                    display_det = det_boxes
                    if self._thresholds:
                        display_det = [
                            b for b in det_boxes
                            if b.get('score', 1.0) >= self._thresholds.get(
                                b.get('type', ''), 0.0)
                        ]
                    if display_det:
                        self._det_pub.publish(
                            boxes_to_marker_array(display_det, self.frame_id, stamp,
                                                  'det', _DET_COLOR, _DEFAULT_DET_COLOR,
                                                  fill_alpha=0.40))
                    else:
                        self._det_pub.publish(_deleteall_ma('det', self.frame_id, stamp))
                except Exception as exc:
                    self.get_logger().warn(f'det {fid}: {exc}')
            else:
                self._det_pub.publish(_deleteall_ma('det', self.frame_id, stamp))
            det_label = f'{label} [{self._det_idx + 1}/{len(self._det_sets)}]'

        # Pre-generated camera image
        self._publish_image(fid, stamp)

        print(f'  Frame [{self._idx + 1}/{len(self._frames)}]  id={fid}\n'
              f'    interp: {interp_label}\n'
              f'    det:    {det_label}\n'
              f'    image:  {"yes" if self._img_pub else "—"}')

    # ── Camera image ──────────────────────────────────────────────────────────

    def _publish_image(self, fid: str, stamp) -> None:
        """Publish a pre-generated image file as-is (no drawing at runtime)."""
        if not self._img_pub:
            return

        # Accept .png (KITTI default) or .jpg
        img_path = self._image_dir / f'{fid}.png'
        if not img_path.exists():
            img_path = self._image_dir / f'{fid}.jpg'
        if not img_path.exists():
            return

        img = cv2.imread(str(img_path))
        if img is None:
            self.get_logger().warn(f'Could not read image: {img_path}')
            return

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = img_rgb.shape[:2]
        msg = Image()
        msg.header.frame_id = 'camera'
        msg.header.stamp    = stamp
        msg.height          = h
        msg.width           = w
        msg.encoding        = 'rgb8'
        msg.is_bigendian    = False
        msg.step            = w * 3
        msg.data            = img_rgb.tobytes()
        self._img_pub.publish(msg)

    # ── Navigation ────────────────────────────────────────────────────────────

    def goto(self, idx: int):
        self._idx = max(0, min(len(self._frames) - 1, idx))
        self.publish_current()

    def next_frame(self): self.goto(self._idx + 1)
    def prev_frame(self): self.goto(self._idx - 1)

    def find_frame(self, token: str) -> Optional[int]:
        """Map a user string to a frame list index, or None if not found."""
        if token in self._frames:
            return self._frames.index(token)
        padded = token.zfill(6)
        if padded in self._frames:
            return self._frames.index(padded)
        if token.lstrip('-').isdigit():
            i = int(token)
            if 0 <= i < len(self._frames):
                return i
        return None

    def next_interp(self):
        if not self._interp_dirs:
            print('  No interpolation datasets loaded.')
            return
        self._interp_idx = (self._interp_idx + 1) % len(self._interp_dirs)
        self.publish_current()

    def prev_interp(self):
        if not self._interp_dirs:
            return
        self._interp_idx = (self._interp_idx - 1) % len(self._interp_dirs)
        self.publish_current()

    def next_det(self):
        if not self._det_sets:
            print('  No detection sets loaded.')
            return
        self._det_idx = (self._det_idx + 1) % len(self._det_sets)
        self.publish_current()

    def prev_det(self):
        if not self._det_sets:
            return
        self._det_idx = (self._det_idx - 1) % len(self._det_sets)
        self.publish_current()


# ─── Interactive loop ─────────────────────────────────────────────────────────

_HELP = """
  n / Enter   next frame
  p           previous frame
  <number>    jump to frame by list index (0-based) or KITTI id (e.g. 42 or 000042)
  i           next interpolation dataset
  I           previous interpolation dataset
  d           next detection set (model / dataset)
  D           previous detection set
  r           refresh / republish current frame
  q           quit
"""


def interactive_loop(node: InspectorNode):
    print(_HELP)
    while rclpy.ok():
        try:
            raw = input('inspector> ').strip()
        except (EOFError, KeyboardInterrupt):
            break

        cmd = raw.lower()
        if cmd in ('q', 'quit', 'exit'):
            break
        elif cmd in ('', 'n'):
            node.next_frame()
        elif cmd == 'p':
            node.prev_frame()
        elif cmd == 'r':
            node.publish_current()
        elif cmd == 'i':
            node.next_interp()
        elif raw == 'I':
            node.prev_interp()
        elif cmd == 'd':
            node.next_det()
        elif raw == 'D':
            node.prev_det()
        else:
            idx = node.find_frame(cmd)
            if idx is not None:
                node.goto(idx)
            else:
                print(f"  Unknown command or frame: {raw!r}  (type 'q' to quit)")

    rclpy.shutdown()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='KITTI multi-cloud inspector for RViz',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_HELP,
    )

    g = parser.add_argument_group('point cloud directories')
    g.add_argument('--original-dir', metavar='DIR',
                   help='Original 64-beam KITTI .bin directory')
    g.add_argument('--reduced-dir',  metavar='DIR',
                   help='Downsampled (e.g. 16-beam) .bin directory')
    g.add_argument('--interp-dir', metavar=('NAME', 'DIR'), nargs=2, action='append',
                   help='Interpolated .bin directory with a label '
                        '(repeatable): --interp-dir nearest_extreme /path/to/dir')

    g2 = parser.add_argument_group('ground truth')
    g2.add_argument('--label-dir', metavar='DIR',
                    help='KITTI label_2 directory (for GT bounding boxes)')
    g2.add_argument('--calib-dir', metavar='DIR',
                    help='KITTI calib directory (required for GT and detections)')
    g2.add_argument('--image-dir', metavar='DIR',
                    help='Directory of pre-generated annotated images (.png/.jpg) to '
                         'publish on /image/annotated (e.g. gt_only/, det_only/, '
                         'det_thresholded/ produced by generate_annotated_images.py)')
    g2.add_argument('--threshold-config', metavar='FILE',
                    help='YAML file with per-class confidence thresholds for 3D '
                         'detection marker display (e.g. Car: 0.5, Pedestrian: 0.5)')

    g3 = parser.add_argument_group('pre-computed detections')
    g3.add_argument('--det-dir', metavar=('LABEL', 'DIR'), nargs=2, action='append',
                    help='Pre-computed detection results directory containing '
                         'per-frame KITTI-format .txt files, with a label '
                         '(repeatable): --det-dir "pv_rcnn/nearest" /path/to/txt_results')

    parser.add_argument('--frame-id', default='velodyne',
                        help='TF frame_id for all messages (default: velodyne)')

    args = parser.parse_args()

    if not any([args.original_dir, args.reduced_dir, args.interp_dir]):
        parser.error('Provide at least one of --original-dir, --reduced-dir, --interp-dir.')

    rclpy.init()
    try:
        node = InspectorNode(args)
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        interactive_loop(node)
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
