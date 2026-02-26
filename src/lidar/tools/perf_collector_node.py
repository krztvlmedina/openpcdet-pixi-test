#!/usr/bin/env python3
"""
Performance Collector Node for Real-Time Pipeline Evaluation

This node subscribes to performance metrics (published as JSON strings on
std_msgs/String topics) from the interpolation and detection nodes, correlates
the messages by timestamp proximity, computes derived metrics, and outputs results
to CSV and JSON files.
"""

import csv
import json
import time
import os
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

        # Correlation buffers
        self.pending_interp_list: List[dict] = []
        self.pending_detect_list: List[dict] = []
        self._MAX_MATCH_LAG = 2.0
        self._MAX_PENDING_AGE = 10.0

        # Metrics lists
        self.completed_frames: List[FrameMetrics] = []
        self.total_received_interp = 0
        self.total_received_detect = 0

        # Statistics
        self.stats_interp_time = RunningStats()
        self.stats_network_time = RunningStats()
        self.stats_detect_time = RunningStats()
        self.stats_total_latency = RunningStats()
        self.stats_input_pts = RunningStats()
        self.stats_output_pts = RunningStats()
        self.stats_detection_count = RunningStats()

        # Subscriptions
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=100)
        self.interp_sub = self.create_subscription(String, '/perf/interpolation', self.interp_callback, qos)
        self.detect_sub = self.create_subscription(String, '/perf/detection', self.detect_callback, qos)

        self.get_logger().info(f"Collector initialized: {self.config_name} / {self.model_name}")

        self.csv_file = None
        self.csv_writer = None
        self.first_frame_time: Optional[float] = None
        self.last_frame_time: Optional[float] = None

    def interp_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("config_name", "") != self.config_name and self.config_name != "default":
                return
            self.total_received_interp += 1
            self.pending_interp_list.append(data)
            self._try_match_pending()
            self._cleanup_by_age()
        except Exception as e:
            self.get_logger().error(f"Interp callback error: {e}")

    def detect_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("model_name", "") != self.model_name and self.model_name != "unknown":
                return
            self.total_received_detect += 1
            self.pending_detect_list.append(data)
            self._try_match_pending()
            self._cleanup_by_age()
        except Exception as e:
            self.get_logger().error(f"Detect callback error: {e}")

    def _try_match_pending(self):
        made_match = True
        while made_match:
            made_match = False
            for i, detect_data in enumerate(self.pending_detect_list):
                t_recv = detect_data.get("receive_timestamp", 0.0)
                best_interp, best_delta, best_j = None, self._MAX_MATCH_LAG + 1.0, -1
                for j, interp_data in enumerate(self.pending_interp_list):
                    delta = t_recv - interp_data.get("publish_timestamp", 0.0)
                    if 0.0 <= delta < best_delta:
                        best_delta, best_interp, best_j = delta, interp_data, j
                if best_interp:
                    self.pending_detect_list.pop(i)
                    self.pending_interp_list.pop(best_j)
                    self._process_correlated_pair(best_interp, detect_data)
                    made_match = True
                    break
            self.get_logger().info("Failed to find match for pending frames.")
            break
    def _cleanup_by_age(self):
        cutoff = time.time() - self._MAX_PENDING_AGE
        self.pending_interp_list = [d for d in self.pending_interp_list if d.get("publish_timestamp", 0.0) > cutoff]
        self.pending_detect_list = [d for d in self.pending_detect_list if d.get("receive_timestamp", 0.0) > cutoff]

    def _process_correlated_pair(self, interp_data: dict, detect_data: dict):
        frame = FrameMetrics(
            sequence_id=interp_data.get("sequence_id", 0),
            interp_receive=interp_data.get("receive_timestamp", 0.0),
            interp_start=interp_data.get("process_start_timestamp", 0.0),
            interp_end=interp_data.get("process_end_timestamp", 0.0),
            interp_publish=interp_data.get("publish_timestamp", 0.0),
            interp_input_pts=interp_data.get("input_point_count", 0),
            interp_output_pts=interp_data.get("output_point_count", 0),
            interp_config=interp_data.get("config_name", ""),
            detect_receive=detect_data.get("receive_timestamp", 0.0),
            detect_preproc_end=detect_data.get("preprocess_end_timestamp", 0.0),
            detect_infer_end=detect_data.get("inference_end_timestamp", 0.0),
            detect_postproc_end=detect_data.get("postprocess_end_timestamp", 0.0),
            detect_input_pts=detect_data.get("input_point_count", 0),
            detection_count=detect_data.get("detection_count", 0),
            model_name=detect_data.get("model_name", ""),
            has_interpolation=True, has_detection=True,
        )
        frame.compute_derived_metrics()

        if self.first_frame_time is None:
            self.first_frame_time = frame.interp_receive
        self.last_frame_time = frame.detect_postproc_end

        frame_number = len(self.completed_frames)
        should_record = frame_number >= self.warmup_frames

        if self.max_frames > 0 and frame_number >= self.warmup_frames + self.max_frames:
            self._write_summary()
            time.sleep(1.0) # Ensure files are flushed
            os._exit(0) # Hard exit for Docker loop reliability

        self.completed_frames.append(frame)

        if should_record:
            self.stats_interp_time.update(frame.interp_time_ms)
            self.stats_network_time.update(frame.network_time_ms)
            self.stats_detect_time.update(frame.detect_time_ms)
            self.stats_total_latency.update(frame.total_latency_ms)
            self.stats_input_pts.update(frame.interp_input_pts)
            self.stats_output_pts.update(frame.interp_output_pts)
            self.stats_detection_count.update(frame.detection_count)
            self._write_csv_row(frame)

            effective_frame = frame_number - self.warmup_frames + 1
            if effective_frame % 50 == 0:
                self.get_logger().info(f"Recorded {effective_frame} frames. Avg Latency: {self.stats_total_latency.mean:.1f}ms")

    def _write_csv_row(self, frame: FrameMetrics):
        if self.csv_file is None:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'sequence_id', 'interp_receive', 'interp_start', 'interp_end', 'interp_publish',
                'interp_input_pts', 'interp_output_pts', 'detect_receive', 'detect_preproc_end',
                'detect_infer_end', 'detect_postproc_end', 'detect_input_pts', 'detection_count',
                'interp_time_ms', 'network_time_ms', 'detect_time_ms', 'total_latency_ms'
            ])
        self.csv_writer.writerow([
            frame.sequence_id, f"{frame.interp_receive:.6f}", f"{frame.interp_start:.6f}",
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
        effective_frames = len(self.completed_frames) - self.warmup_frames
        throughput_fps = 0.0
        if self.first_frame_time and self.last_frame_time and effective_frames > 1:
            start = self.completed_frames[self.warmup_frames].interp_receive
            duration = self.last_frame_time - start
            if duration > 0: throughput_fps = (effective_frames - 1) / duration

        point_amp = self.stats_output_pts.mean / self.stats_input_pts.mean if self.stats_input_pts.mean > 0 else 0.0

        summary = {
            "config": {
                "interpolation_config": self.config_name, "model": self.model_name,
                "total_frames": len(self.completed_frames), "warmup_frames": self.warmup_frames,
                "effective_frames": max(0, effective_frames),
            },
            "metrics": {
                "interpolation_time_ms": self.stats_interp_time.to_dict(),
                "network_transfer_time_ms": self.stats_network_time.to_dict(),
                "detection_time_ms": self.stats_detect_time.to_dict(),
                "total_latency_ms": self.stats_total_latency.to_dict(),
                "throughput_fps": {"mean": throughput_fps, "achieved_rate": throughput_fps},
                "point_amplification": {"mean": point_amp, "input_mean": self.stats_input_pts.mean, "output_mean": self.stats_output_pts.mean},
                "detections_per_frame": self.stats_detection_count.to_dict(),
            }
        }

        with open(self.json_filename, 'w') as f:
            json.dump(summary, f, indent=2)

        # FULL SUMMARY BLOCK RESTORED
        print("\n" + "=" * 60)
        print("PERFORMANCE EVALUATION SUMMARY")
        print("=" * 60)
        print(f"Config: {self.config_name} | Model: {self.model_name}")
        print(f"Effective frames: {effective_frames}")
        print(f"Throughput:       {throughput_fps:.2f} FPS")
        print("-" * 60)
        print(f"Interpolation:    {self.stats_interp_time.mean:.2f} ± {self.stats_interp_time.std:.2f} ms")
        print(f"Network:          {self.stats_network_time.mean:.2f} ± {self.stats_network_time.std:.2f} ms")
        print(f"Detection:        {self.stats_detect_time.mean:.2f} ± {self.stats_detect_time.std:.2f} ms")
        print(f"Total Latency:    {self.stats_total_latency.mean:.2f} ± {self.stats_total_latency.std:.2f} ms")
        print("-" * 60)
        print(f"Point Amp:        {point_amp:.2f}x")
        print(f"Avg Detections:   {self.stats_detection_count.mean:.1f}")
        print("=" * 60 + "\n")

    def destroy_node(self):
        self._write_summary()
        if self.csv_file: self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PerfCollectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()