#!/usr/bin/env python3
"""
Aggregate Real-Time Performance Evaluation Results

Reads JSON summary files from a performance evaluation run,
generates comparison tables, timing breakdown charts, and LaTeX tables.

Usage:
    python3 aggregate_realtime_results.py <results_dir>

    # Example:
    python3 aggregate_realtime_results.py /app/data/perf_results/20240101_120000
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Optional: matplotlib for charts
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.ticker as mticker
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

METRICS_OF_INTEREST = [
    ("interpolation_time_ms", "Interpolation"),
    ("network_transfer_time_ms", "Network Transfer"),
    ("detection_time_ms", "Detection"),
    ("total_latency_ms", "Total Latency"),
]

MODEL_ORDER = [
    "parta2_anchor",
    "pointpillar",
    "pointrcnn_iou",
    "pv_rcnn",
    "second",
    "second_iou",
]

MODEL_DISPLAY_NAMES = {
    "parta2_anchor": "PartA2",
    "pointpillar": "PointPillar",
    "pointrcnn_iou": "PointRCNN",
    "pv_rcnn": "PV-RCNN",
    "second": "SECOND",
    "second_iou": "SECOND-IoU",
}


def load_summaries(results_dir: Path) -> Dict[str, Dict[str, dict]]:
    """Load all JSON summaries, organized by config then model.

    Returns:
        {config_name: {model_name: summary_dict}}
    """
    data = {}
    for json_file in sorted(results_dir.rglob("summary_*.json")):
        with open(json_file) as f:
            summary = json.load(f)

        config_name = summary["config"]["interpolation_config"]
        model_name = summary["config"]["model"]

        if config_name not in data:
            data[config_name] = {}
        data[config_name][model_name] = summary

    return data


def generate_comparison_table(data: Dict[str, Dict[str, dict]]) -> str:
    """Generate a text comparison table."""
    lines = []
    header_fmt = "{:<15} {:>12} {:>12} {:>12} {:>12} {:>8} {:>8}"
    row_fmt = "{:<15} {:>12.2f} {:>12.2f} {:>12.2f} {:>12.2f} {:>8.2f} {:>8.1f}"

    for config_name, models in sorted(data.items()):
        lines.append(f"\n{'=' * 90}")
        lines.append(f"Interpolation Config: {config_name}")
        lines.append(f"{'=' * 90}")
        lines.append(header_fmt.format(
            "Model", "Interp (ms)", "Network (ms)", "Detect (ms)",
            "Total (ms)", "FPS", "Det/Frame"
        ))
        lines.append("-" * 90)

        for model_name in MODEL_ORDER:
            if model_name not in models:
                continue

            summary = models[model_name]
            m = summary["metrics"]

            lines.append(row_fmt.format(
                model_name,
                m["interpolation_time_ms"]["mean"],
                m["network_transfer_time_ms"]["mean"],
                m["detection_time_ms"]["mean"],
                m["total_latency_ms"]["mean"],
                m["throughput_fps"]["mean"],
                m.get("detections_per_frame", {}).get("mean", 0),
            ))

        lines.append("-" * 90)

    return "\n".join(lines)


def generate_latex_table(data: Dict[str, Dict[str, dict]]) -> str:
    """Generate a LaTeX table with timing breakdown per model."""
    lines = []

    for config_name, models in sorted(data.items()):
        lines.append(f"% Config: {config_name}")
        lines.append(r"\begin{tabular}{lrrrrrr}")
        lines.append(r"\toprule")
        lines.append(
            r"Model & Interp.\ (ms) & Network (ms) & Detection (ms) "
            r"& Total (ms) & FPS & Point Amp.\ \\"
        )
        lines.append(r"\midrule")

        for model_name in MODEL_ORDER:
            if model_name not in models:
                continue

            summary = models[model_name]
            m = summary["metrics"]
            display = MODEL_DISPLAY_NAMES.get(model_name, model_name).replace("_", r"\_")

            interp = m["interpolation_time_ms"]
            network = m["network_transfer_time_ms"]
            detect = m["detection_time_ms"]
            total = m["total_latency_ms"]
            fps = m["throughput_fps"]["mean"]
            amp = m["point_amplification"]["mean"]

            lines.append(
                f"{display} & "
                f"${interp['mean']:.1f} \\pm {interp['std']:.1f}$ & "
                f"${network['mean']:.1f} \\pm {network['std']:.1f}$ & "
                f"${detect['mean']:.1f} \\pm {detect['std']:.1f}$ & "
                f"${total['mean']:.1f} \\pm {total['std']:.1f}$ & "
                f"{fps:.1f} & "
                f"{amp:.2f}x"
                r" \\"
            )

        lines.append(r"\bottomrule")
        lines.append(r"\end{tabular}")
        lines.append("")

    return "\n".join(lines)


def generate_detailed_latex_table(data: Dict[str, Dict[str, dict]]) -> str:
    """Generate a detailed LaTeX table with percentiles."""
    lines = []

    for config_name, models in sorted(data.items()):
        lines.append(f"% Detailed timing: Config {config_name}")
        lines.append(r"\begin{tabular}{lrrrrrr}")
        lines.append(r"\toprule")
        lines.append(
            r"Model & Mean & Std & Min & P50 & P95 & P99 \\"
        )

        for metric_key, metric_label in METRICS_OF_INTEREST:
            lines.append(r"\midrule")
            lines.append(rf"\multicolumn{{7}}{{c}}{{\textbf{{{metric_label} (ms)}}}} \\")
            lines.append(r"\midrule")

            for model_name in MODEL_ORDER:
                if model_name not in models:
                    continue

                summary = models[model_name]
                m = summary["metrics"].get(metric_key, {})
                display = MODEL_DISPLAY_NAMES.get(model_name, model_name).replace("_", r"\_")

                lines.append(
                    f"{display} & "
                    f"{m.get('mean', 0):.1f} & "
                    f"{m.get('std', 0):.1f} & "
                    f"{m.get('min', 0):.1f} & "
                    f"{m.get('p50', 0):.1f} & "
                    f"{m.get('p95', 0):.1f} & "
                    f"{m.get('p99', 0):.1f}"
                    r" \\"
                )

        lines.append(r"\bottomrule")
        lines.append(r"\end{tabular}")
        lines.append("")

    return "\n".join(lines)


def generate_timing_chart(data: Dict[str, Dict[str, dict]], output_dir: Path):
    """Generate a stacked bar chart showing timing breakdown per model."""
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available, skipping chart generation.")
        return

    # ── Compute global axis limits across ALL configs and models ─────────────
    global_max_total_ms = 0.0
    global_max_fps = 0.0
    for models in data.values():
        for model_name, summary in models.items():
            m = summary["metrics"]
            total = (m["interpolation_time_ms"]["mean"]
                     + m["network_transfer_time_ms"]["mean"]
                     + m["detection_time_ms"]["mean"])
            global_max_total_ms = max(global_max_total_ms, total)
            global_max_fps = max(global_max_fps, m["throughput_fps"]["mean"])

    latency_ylim = global_max_total_ms * 1.18   # headroom for top labels
    fps_ylim     = global_max_fps      * 1.18

    # ── Per-config stacked latency breakdown charts ───────────────────────────
    for config_name, models in sorted(data.items()):
        model_names = []
        interp_times = []
        network_times = []
        detect_times = []

        for model_name in MODEL_ORDER:
            if model_name not in models:
                continue
            m = models[model_name]["metrics"]
            model_names.append(MODEL_DISPLAY_NAMES.get(model_name, model_name))
            interp_times.append(m["interpolation_time_ms"]["mean"])
            network_times.append(m["network_transfer_time_ms"]["mean"])
            detect_times.append(m["detection_time_ms"]["mean"])

        if not model_names:
            continue

        fig, ax = plt.subplots(figsize=(10, 6))

        x = range(len(model_names))
        width = 0.6

        ax.bar(x, interp_times, width, label='Interpolación', color='#2196F3')
        ax.bar(x, network_times, width, bottom=interp_times,
               label='Transferencia de red', color='#FF9800')
        ax.bar(
            x, detect_times, width,
            bottom=[i + n for i, n in zip(interp_times, network_times)],
            label='Detección', color='#4CAF50'
        )

        ax.set_ylabel('Latencia (ms)')
        ax.set_title(f'Desglose de latencia del pipeline — {config_name}')
        ax.set_xticks(list(x))
        ax.set_xticklabels(model_names, rotation=45, ha='right')
        ax.set_ylim(0, latency_ylim)
        ax.yaxis.set_major_locator(mticker.MultipleLocator(
            max(20, round(global_max_total_ms / 8 / 10) * 10)
        ))
        ax.legend()
        ax.grid(axis='y', alpha=0.3)

        # Total latency label on top of each bar
        for i, (interp, net, det) in enumerate(zip(interp_times, network_times, detect_times)):
            total = interp + net + det
            ax.text(i, total + latency_ylim * 0.01, f'{total:.0f} ms',
                    ha='center', va='bottom', fontsize=9)

        plt.tight_layout()
        chart_path = output_dir / f"timing_breakdown_{config_name}.png"
        plt.savefig(chart_path, dpi=150)
        plt.close()
        print(f"Generated chart: {chart_path}")

    # ── FPS comparison chart (all configs × all models) ───────────────────────
    config_names_list = sorted(data.keys())
    bar_width = 0.8 / max(len(config_names_list), 1)
    all_models = []
    for cn in config_names_list:
        for mn in MODEL_ORDER:
            if mn in data[cn] and mn not in all_models:
                all_models.append(mn)

    if all_models:
        fig, ax = plt.subplots(figsize=(10, 6))
        x = range(len(all_models))

        for ci, config_name in enumerate(config_names_list):
            fps_values = [
                data[config_name][mn]["metrics"]["throughput_fps"]["mean"]
                if mn in data[config_name] else 0
                for mn in all_models
            ]
            offset = (ci - len(config_names_list) / 2 + 0.5) * bar_width
            ax.bar([xi + offset for xi in x], fps_values, bar_width, label=config_name)

        ax.set_ylabel('Rendimiento (FPS)')
        ax.set_title('Comparación de rendimiento')
        ax.set_xticks(list(x))
        ax.set_xticklabels(
            [MODEL_DISPLAY_NAMES.get(m, m) for m in all_models],
            rotation=45, ha='right'
        )
        ax.set_ylim(0, fps_ylim)
        ax.axhline(y=10, color='r', linestyle='--', alpha=0.5, label='Objetivo 10 FPS')
        ax.legend()
        ax.grid(axis='y', alpha=0.3)

        plt.tight_layout()
        chart_path = output_dir / "fps_comparison.png"
        plt.savefig(chart_path, dpi=150)
        plt.close()
        print(f"Generated chart: {chart_path}")


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <results_dir>")
        print("  results_dir: Directory containing perf evaluation results")
        sys.exit(1)

    results_dir = Path(sys.argv[1])
    if not results_dir.exists():
        print(f"Error: Directory not found: {results_dir}")
        sys.exit(1)

    # Load all summaries
    data = load_summaries(results_dir)
    if not data:
        print(f"No summary files found in {results_dir}")
        sys.exit(1)

    print(f"Loaded results for {sum(len(m) for m in data.values())} model/config combinations")

    # Create output directory for aggregated results
    output_dir = results_dir / "aggregated"
    output_dir.mkdir(exist_ok=True)

    # Generate text comparison table
    table = generate_comparison_table(data)
    print(table)
    table_path = output_dir / "comparison_table.txt"
    table_path.write_text(table)
    print(f"\nComparison table saved to: {table_path}")

    # Generate LaTeX tables
    latex = generate_latex_table(data)
    latex_path = output_dir / "latency_table.tex"
    latex_path.write_text(latex)
    print(f"LaTeX table saved to: {latex_path}")

    detailed_latex = generate_detailed_latex_table(data)
    detailed_latex_path = output_dir / "latency_detailed_table.tex"
    detailed_latex_path.write_text(detailed_latex)
    print(f"Detailed LaTeX table saved to: {detailed_latex_path}")

    # Generate charts
    generate_timing_chart(data, output_dir)

    # Write combined JSON with all results
    combined_path = output_dir / "all_results.json"
    with open(combined_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Combined results saved to: {combined_path}")

    print("\nAggregation complete!")


if __name__ == "__main__":
    main()
