import sys
from pathlib import Path

CLASSES = ["Car", "Pedestrian", "Cyclist"]
DIFFICULTIES = ["Easy", "Moderate", "Hard"]
RECALL_KEYS = [
    "recall_roi_0.3",  "recall_rcnn_0.3",
    "recall_roi_0.5",  "recall_rcnn_0.5",
    "recall_roi_0.7",  "recall_rcnn_0.7",
]

# -------------------------------------------------
# Parseo del log: AP_R40 → bbox AP + recall values
# -------------------------------------------------

def parse_log(log_path):
    ap_results = {}
    recall_results = {}
    current_class = None
    in_ap_r40_block = False

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()

            # Recall lines look like: "... INFO  recall_roi_0.3: 0.970099"
            for key in RECALL_KEYS:
                tag = key + ":"
                if tag in line:
                    try:
                        recall_results[key] = float(line.split(tag)[1].strip())
                    except Exception:
                        pass

            # Inicio del bloque AP_R40 para una clase
            for cls in CLASSES:
                if line.startswith(f"{cls} AP_R40@"):
                    current_class = cls
                    in_ap_r40_block = True
                    break

            if not in_ap_r40_block or current_class is None:
                continue

            # Línea bbox AP
            if line.startswith("bbox"):
                try:
                    values = line.split(":")[1].split(",")
                    ap_results[current_class] = {
                        "Easy":     float(values[0]),
                        "Moderate": float(values[1]),
                        "Hard":     float(values[2]),
                    }
                except Exception:
                    pass

                current_class = None
                in_ap_r40_block = False

    return ap_results, recall_results

# -------------------------------------------------
# Tabla LaTeX: bbox AP_R40
# -------------------------------------------------

def latex_ap_table(data):
    lines = []
    lines.append(r"\begin{tabular}{lccc ccc ccc}")
    lines.append(r"\hline")
    lines.append(
        r"Model & "
        r"\multicolumn{3}{c}{Car} & "
        r"\multicolumn{3}{c}{Pedestrian} & "
        r"\multicolumn{3}{c}{Cyclist} \\"
    )
    lines.append(
        r"& Easy & Mod & Hard & "
        r"Easy & Mod & Hard & "
        r"Easy & Mod & Hard \\"
    )
    lines.append(r"\hline")

    for model, res in sorted(data.items()):
        row = [model.replace("_", r"\_")]
        for cls in CLASSES:
            if cls in res:
                for diff in DIFFICULTIES:
                    row.append(f"{res[cls][diff]:.2f}")
            else:
                row.extend(["-", "-", "-"])
        lines.append(" & ".join(row) + r" \\")

    lines.append(r"\hline")
    lines.append(r"\end{tabular}")
    return "\n".join(lines)

# -------------------------------------------------
# Tabla LaTeX: recall values
# -------------------------------------------------

RECALL_HEADERS = [
    r"roi@0.3", r"rcnn@0.3",
    r"roi@0.5", r"rcnn@0.5",
    r"roi@0.7", r"rcnn@0.7",
]

def latex_recall_table(data):
    lines = []
    lines.append(r"\begin{tabular}{l" + "c" * len(RECALL_KEYS) + "}")
    lines.append(r"\hline")
    lines.append("Model & " + " & ".join(RECALL_HEADERS) + r" \\")
    lines.append(r"\hline")

    for model, recall in sorted(data.items()):
        row = [model.replace("_", r"\_")]
        for key in RECALL_KEYS:
            if key in recall:
                row.append(f"{recall[key]:.4f}")
            else:
                row.append("-")
        lines.append(" & ".join(row) + r" \\")

    lines.append(r"\hline")
    lines.append(r"\end{tabular}")
    return "\n".join(lines)

# -------------------------------------------------
# Main
# -------------------------------------------------

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 generate_latex_tables.py <output/date>")
        sys.exit(1)

    run_root = Path(sys.argv[1])
    models_root = run_root

    if not models_root.exists():
        print(f"Invalid path: {models_root}")
        sys.exit(1)

    latex_dir = run_root / "latex"
    latex_dir.mkdir(exist_ok=True)

    for dataset_dir in sorted(models_root.iterdir()):
        if not dataset_dir.is_dir():
            continue

        dataset_name = dataset_dir.name
        ap_data = {}
        recall_data = {}

        for model_dir in dataset_dir.iterdir():
            if not model_dir.is_dir():
                continue

            model_name = model_dir.name
            logs = sorted(model_dir.rglob("log_eval_*.txt"))
            if not logs:
                continue

            ap_res, recall_res = parse_log(logs[-1])

            if ap_res:
                ap_data[model_name] = ap_res
            if recall_res:
                recall_data[model_name] = recall_res

        if not ap_data and not recall_data:
            continue

        if ap_data:
            out_ap = latex_dir / f"table_{dataset_name}.tex"
            out_ap.write_text(latex_ap_table(ap_data))
            print(f"Generated AP table:     {out_ap}")

        if recall_data:
            out_recall = latex_dir / f"table_{dataset_name}_recall.tex"
            out_recall.write_text(latex_recall_table(recall_data))
            print(f"Generated recall table: {out_recall}")

if __name__ == "__main__":
    main()
