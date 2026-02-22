#!/usr/bin/env python3
"""
Performance Collector Node for Real-Time Pipeline Evaluation

This node subscribes to performance metrics (published as JSON strings on
std_msgs/String topics) from the interpolation and detection nodes, correlates
the messages by timestamp proximity, computes derived metrics, and outputs results
to CSV and JSON files.

Both publishers (C++ interpolation node, Python detection node) serialize
performance data as JSON inside std_msgs/msg/String to avoid cross-package
build dependencies on custom message types.

Usage:
    python3 src/lidar/tools/perf_collector_node.py --ros-args \
        -p output_dir:=/path/to/output \
        -p warmup_frames:=10 \
        -p max_frames:=500 \
        -p config_name:=optimized \
        -p model_name:=parta2_anchor
"""

import csv
import json
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass
class FrameMetrics:
    """Metrics for a single frame."""
    sequence_id: int = 0
    # Interpolation metrics
    interp_receive: float = 0.0
    interp_start: float = 0.0
    interp_end: float = 0.0
    interp_publish: float = 0.0
    interp_input_pts: int = 0
    interp_output_pts: int = 0
    interp_config: str = ""
    # Detection metrics
    detect_receive: float = 0.0
    detect_preproc_end: float = 0.0
    detect_infer_end: float = 0.0
    detect_postproc_end: float = 0.0
    detect_input_pts: int = 0
    detection_count: int = 0
    model_name: str = ""
    # Computed metrics (in ms)
    interp_time_ms: float = 0.0
    network_time_ms: float = 0.0
    detect_time_ms: float = 0.0
    total_latency_ms: float = 0.0
    # Correlation status
    has_interpolation: bool = False
    has_detection: bool = False

    def compute_derived_metrics(self):
        """Compute derived timing metrics."""
        if self.has_interpolation:
            self.interp_time_ms = (self.interp_end - self.interp_start) * 1000.0

        if self.has_detection:
            self.detect_time_ms = (self.detect_postproc_end - self.detect_receive) * 1000.0

        if self.has_interpolation and self.has_detection:
            self.network_time_ms = (self.detect_receive - self.interp_publish) * 1000.0
            self.total_latency_ms = (self.detect_postproc_end - self.interp_receive) * 1000.0

    def is_complete(self) -> bool:
        """Check if both interpolation and detection data are available."""
        return self.has_interpolation and self.has_detection


@dataclass
class RunningStats:
    """Online statistics calculator using Welford's algorithm."""
    count: int = 0
    mean: float = 0.0
    m2: float = 0.0  # Sum of squares of differences from mean
    min_val: float = float('inf')
    max_val: float = float('-inf')
    values: List[float] = field(default_factory=list)  # For percentile calculation

    def update(self, value: float):
        """Update statistics with a new value."""
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        delta2 = value - self.mean
        self.m2 += delta * delta2
        self.min_val = min(self.min_val, value)
        self.max_val = max(self.max_val, value)
        self.values.append(value)

    @property
    def std(self) -> float:
        """Return sample standard deviation."""
        if self.count < 2:
            return 0.0
        return (self.m2 / (self.count - 1)) ** 0.5

    def percentile(self, p: float) -> float:
        """Calculate percentile (0-100)."""
        if not self.values:
            return 0.0
        sorted_vals = sorted(self.values)
        k = (len(sorted_vals) - 1) * p / 100.0
        f = int(k)
        c = f + 1 if f + 1 < len(sorted_vals) else f
        return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f) if c != f else sorted_vals[f]

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "mean": self.mean,
            "std": self.std,
            "min": self.min_val if self.min_val != float('inf') else 0.0,
            "max": self.max_val if self.max_val != float('-inf') else 0.0,
            "p50": self.percentile(50),
            "p95": self.percentile(95),
            "p99": self.percentile(99),
        }


class PerfCollectorNode(Node):
    """ROS2 node for collecting and correlating performance metrics."""

    def __init__(self):
        super().__init__('perf_collector_node')

        # Declare parameters
        self.declare_parameter('output_dir', '/tmp/perf_results')
        self.declare_parameter('warmup_frames', 10)
        self.declare_parameter('max_frames', 0)  # 0 = unlimited
        self.declare_parameter('config_name', 'default')
        self.declare_parameter('model_name', 'unknown')

        # Get parameters
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.warmup_frames = self.get_parameter('warmup_frames').value
        self.max_frames = self.get_parameter('max_frames').value
        self.config_name = self.get_parameter('config_name').value
        self.model_name = self.get_parameter('model_name').value

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Generate output file names with timestamp
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = self.output_dir / f"perf_results_{self.config_name}_{self.model_name}_{timestamp_str}.csv"
        self.json_filename = self.output_dir / f"summary_{self.config_name}_{self.model_name}_{timestamp_str}.json"

        # Pending messages buffer (waiting for correlation).
        # Matching is done by publish_timestamp proximity so that frames dropped
        # by the (slower) detection node do not desync independent seq counters.
        self.pending_interp_list: List[dict] = []
        self.pending_detect_list: List[dict] = []
        # Max transit time from interp publish to detect receive (seconds)
        self._MAX_MATCH_LAG = 2.0
        # Drop pending messages older than this (seconds)
        self._MAX_PENDING_AGE = 10.0

        # Completed frame metrics
        self.completed_frames: List[FrameMetrics] = []
        self.total_received_interp = 0
        self.total_received_detect = 0

        # Running statistics (excluding warmup)
        self.stats_interp_time = RunningStats()
        self.stats_network_time = RunningStats()
        self.stats_detect_time = RunningStats()
        self.stats_total_latency = RunningStats()
        self.stats_input_pts = RunningStats()
        self.stats_output_pts = RunningStats()
        self.stats_detection_count = RunningStats()

        # Subscribe to JSON string topics (no custom msg dependency)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=100)
        self.interp_sub = self.create_subscription(
            String, '/perf/interpolation', self.interp_callback, qos)
        self.detect_sub = self.create_subscription(
            String, '/perf/detection', self.detect_callback, qos)

        self.get_logger().info(f"Performance collector initialized. Output dir: {self.output_dir}")
        self.get_logger().info(f"Config: {self.config_name}, Model: {self.model_name}")
        self.get_logger().info(f"Warmup frames: {self.warmup_frames}, Max frames: {self.max_frames}")

        # CSV file handle (opened lazily)
        self.csv_file = None
        self.csv_writer = None

        # Start time for throughput calculation
        self.first_frame_time: Optional[float] = None
        self.last_frame_time: Optional[float] = None

    def interp_callback(self, msg: String):
        """Handle incoming interpolation performance JSON message."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse interp perf JSON: {e}")
            return

        # Reject messages from a different interpolation config to prevent
        # cross-run contamination (old interpolation node still alive).
        msg_config = data.get("config_name", "")
        if msg_config and self.config_name not in ("default", "") \
                and msg_config != self.config_name:
            self.get_logger().warn(
                f"Ignoring interp msg from config '{msg_config}' "
                f"(expected '{self.config_name}')")
            return

        self.total_received_interp += 1
        self.get_logger().info(f"Received interp perf seq {data.get('sequence_id')}")
        self.pending_interp_list.append(data)
        self._try_match_pending()
        self._cleanup_by_age()

    def detect_callback(self, msg: String):
        """Handle incoming detection performance JSON message."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse detect perf JSON: {e}")
            return

        # Reject messages from a different model to prevent cross-run
        # contamination (old pcdet_node still alive and processing new frames).
        msg_model = data.get("model_name", "")
        if msg_model and self.model_name not in ("unknown", "") \
                and msg_model != self.model_name:
            self.get_logger().warn(
                f"Ignoring detect msg from model '{msg_model}' "
                f"(expected '{self.model_name}')")
            return

        self.total_received_detect += 1
        self.get_logger().info(f"Received detect perf seq {data.get('sequence_id')}")
        self.pending_detect_list.append(data)
        self._try_match_pending()
        self._cleanup_by_age()

    def _try_match_pending(self):
        """Match pending interp+detect pairs by publish→receive timestamp proximity.

        For each waiting detection message, find the interpolation message whose
        publish_timestamp is closest to (and just before) detect.receive_timestamp.
        This is robust to dropped frames because it never relies on equal seq IDs.
        """
        made_match = True
        while made_match:
            made_match = False
            for i, detect_data in enumerate(self.pending_detect_list):
                t_recv = detect_data.get("receive_timestamp", 0.0)
                best_interp = None
                best_delta = self._MAX_MATCH_LAG + 1.0
                best_j = -1
                for j, interp_data in enumerate(self.pending_interp_list):
                    t_pub = interp_data.get("publish_timestamp", 0.0)
                    delta = t_recv - t_pub
                    if 0.0 <= delta < best_delta:
                        best_delta = delta
                        best_interp = interp_data
                        best_j = j
                if best_interp is not None:
                    self.pending_detect_list.pop(i)
                    self.pending_interp_list.pop(best_j)
                    self._process_correlated_pair(best_interp, detect_data)
                    made_match = True
                    break  # lists changed — restart outer loop

    def _cleanup_by_age(self):
        """Drop pending messages older than _MAX_PENDING_AGE to prevent memory growth."""
        now = time.time()
        cutoff = now - self._MAX_PENDING_AGE

        stale_interp = [d for d in self.pending_interp_list
                        if d.get("publish_timestamp", 0.0) < cutoff]
        for d in stale_interp:
            self.get_logger().warn(
                f"Dropping stale interp msg (seq {d.get('sequence_id')})")
            self.pending_interp_list.remove(d)

        stale_detect = [d for d in self.pending_detect_list
                        if d.get("receive_timestamp", 0.0) < cutoff]
        for d in stale_detect:
            self.get_logger().warn(
                f"Dropping stale detect msg (seq {d.get('sequence_id')})")
            self.pending_detect_list.remove(d)

    def _process_correlated_pair(self, interp_data: dict, detect_data: dict):
        """Process a correlated pair of interpolation and detection JSON data."""
        frame = FrameMetrics(
            sequence_id=interp_data.get("sequence_id", 0),
            # Interpolation data
            interp_receive=interp_data.get("receive_timestamp", 0.0),
            interp_start=interp_data.get("process_start_timestamp", 0.0),
            interp_end=interp_data.get("process_end_timestamp", 0.0),
            interp_publish=interp_data.get("publish_timestamp", 0.0),
            interp_input_pts=interp_data.get("input_point_count", 0),
            interp_output_pts=interp_data.get("output_point_count", 0),
            interp_config=interp_data.get("config_name", ""),
            # Detection data
            detect_receive=detect_data.get("receive_timestamp", 0.0),
            detect_preproc_end=detect_data.get("preprocess_end_timestamp", 0.0),
            detect_infer_end=detect_data.get("inference_end_timestamp", 0.0),
            detect_postproc_end=detect_data.get("postprocess_end_timestamp", 0.0),
            detect_input_pts=detect_data.get("input_point_count", 0),
            detection_count=detect_data.get("detection_count", 0),
            model_name=detect_data.get("model_name", ""),
            has_interpolation=True,
            has_detection=True,
        )

        # Compute derived metrics
        frame.compute_derived_metrics()

        # Track frame times for throughput
        if self.first_frame_time is None:
            self.first_frame_time = frame.interp_receive
        self.last_frame_time = frame.detect_postproc_end

        # Check if we should record (after warmup, before max)
        frame_number = len(self.completed_frames)
        should_record = frame_number >= self.warmup_frames

        if self.max_frames > 0 and frame_number >= self.warmup_frames + self.max_frames:
            # We've collected enough frames, trigger shutdown
            self.get_logger().info(f"Reached max_frames ({self.max_frames}). Writing summary...")
            self._write_summary()
            self.get_logger().info(f"Shutting down...")
            raise SystemExit
            return

        self.completed_frames.append(frame)

        if should_record:
            # Update running statistics
            self.stats_interp_time.update(frame.interp_time_ms)
            self.stats_network_time.update(frame.network_time_ms)
            self.stats_detect_time.update(frame.detect_time_ms)
            self.stats_total_latency.update(frame.total_latency_ms)
            self.stats_input_pts.update(frame.interp_input_pts)
            self.stats_output_pts.update(frame.interp_output_pts)
            self.stats_detection_count.update(frame.detection_count)

            # Write to CSV
            self._write_csv_row(frame)

            # Log progress periodically
            effective_frame = frame_number - self.warmup_frames + 1
            if effective_frame % 50 == 0:
                self.get_logger().info(
                    f"Recorded {effective_frame} frames. "
                    f"Avg latency: {self.stats_total_latency.mean:.1f}ms, "
                    f"Avg interp: {self.stats_interp_time.mean:.1f}ms, "
                    f"Avg detect: {self.stats_detect_time.mean:.1f}ms"
                )
        else:
            self.get_logger().debug(f"Warmup frame {frame_number + 1}/{self.warmup_frames}")

    def _write_csv_row(self, frame: FrameMetrics):
        """Write a single frame to CSV file."""
        if self.csv_file is None:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header
            self.csv_writer.writerow([
                'sequence_id',
                'interp_receive', 'interp_start', 'interp_end', 'interp_publish',
                'interp_input_pts', 'interp_output_pts',
                'detect_receive', 'detect_preproc_end', 'detect_infer_end', 'detect_postproc_end',
                'detect_input_pts', 'detection_count',
                'interp_time_ms', 'network_time_ms', 'detect_time_ms', 'total_latency_ms'
            ])

        self.csv_writer.writerow([
            frame.sequence_id,
            f"{frame.interp_receive:.6f}", f"{frame.interp_start:.6f}",
            f"{frame.interp_end:.6f}", f"{frame.interp_publish:.6f}",
            frame.interp_input_pts, frame.interp_output_pts,
            f"{frame.detect_receive:.6f}", f"{frame.detect_preproc_end:.6f}",
            f"{frame.detect_infer_end:.6f}", f"{frame.detect_postproc_end:.6f}",
            frame.detect_input_pts, frame.detection_count,
            f"{frame.interp_time_ms:.3f}", f"{frame.network_time_ms:.3f}",
            f"{frame.detect_time_ms:.3f}", f"{frame.total_latency_ms:.3f}"
        ])
        self.csv_file.flush()

    def _write_summary(self):
        """Write aggregate summary to JSON file."""
        effective_frames = len(self.completed_frames) - self.warmup_frames

        # Calculate throughput
        throughput_fps = 0.0
        if self.first_frame_time and self.last_frame_time and effective_frames > 1:
            effective_start = self.completed_frames[self.warmup_frames].interp_receive if len(self.completed_frames) > self.warmup_frames else self.first_frame_time
            duration = self.last_frame_time - effective_start
            if duration > 0:
                throughput_fps = (effective_frames - 1) / duration

        # Calculate point amplification
        point_amplification = 0.0
        if self.stats_input_pts.mean > 0:
            point_amplification = self.stats_output_pts.mean / self.stats_input_pts.mean

        summary = {
            "config": {
                "interpolation_config": self.config_name,
                "model": self.model_name,
                "total_frames": len(self.completed_frames),
                "warmup_frames": self.warmup_frames,
                "effective_frames": max(0, effective_frames),
            },
            "metrics": {
                "interpolation_time_ms": self.stats_interp_time.to_dict(),
                "network_transfer_time_ms": self.stats_network_time.to_dict(),
                "detection_time_ms": self.stats_detect_time.to_dict(),
                "total_latency_ms": self.stats_total_latency.to_dict(),
                "throughput_fps": {
                    "mean": throughput_fps,
                    "achieved_rate": throughput_fps,
                },
                "point_amplification": {
                    "mean": point_amplification,
                    "input_mean": self.stats_input_pts.mean,
                    "output_mean": self.stats_output_pts.mean,
                },
                "detections_per_frame": self.stats_detection_count.to_dict(),
            },
            "collection_info": {
                "total_interp_messages": self.total_received_interp,
                "total_detect_messages": self.total_received_detect,
                "unmatched_interp": len(self.pending_interp_list),
                "unmatched_detect": len(self.pending_detect_list),
                "timestamp": datetime.now().isoformat(),
            }
        }

        with open(self.json_filename, 'w') as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(f"Summary written to {self.json_filename}")
        self.get_logger().info(f"CSV data written to {self.csv_filename}")

        # Print summary to console
        self.get_logger().info("=" * 60)
        self.get_logger().info("PERFORMANCE EVALUATION SUMMARY")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Config: {self.config_name}, Model: {self.model_name}")
        self.get_logger().info(f"Effective frames: {effective_frames}")
        self.get_logger().info(f"Throughput: {throughput_fps:.2f} FPS")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Interpolation time: {self.stats_interp_time.mean:.2f} +/- {self.stats_interp_time.std:.2f} ms")
        self.get_logger().info(f"Network transfer:   {self.stats_network_time.mean:.2f} +/- {self.stats_network_time.std:.2f} ms")
        self.get_logger().info(f"Detection time:     {self.stats_detect_time.mean:.2f} +/- {self.stats_detect_time.std:.2f} ms")
        self.get_logger().info(f"Total latency:      {self.stats_total_latency.mean:.2f} +/- {self.stats_total_latency.std:.2f} ms")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Point amplification: {point_amplification:.2f}x")
        self.get_logger().info(f"Avg detections/frame: {self.stats_detection_count.mean:.1f}")
        self.get_logger().info("=" * 60)

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self._write_summary()
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PerfCollectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        rclpy.logging.get_logger().info('Done')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
