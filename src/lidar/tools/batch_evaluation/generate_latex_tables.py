import sys
from pathlib import Path

CLASSES = ["Car", "Pedestrian", "Cyclist"]
DIFFICULTIES = ["Easy", "Moderate", "Hard"]

# -------------------------------------------------
# Parseo del log: AP_R40 → bbox AP
# -------------------------------------------------

def parse_log(log_path):
    results = {}
    current_class = None
    in_ap_r40_block = False

    with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()

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
                    results[current_class] = {
                        "Easy": float(values[0]),
                        "Moderate": float(values[1]),
                        "Hard": float(values[2]),
                    }
                except Exception:
                    pass

                # Cerrar bloque
                current_class = None
                in_ap_r40_block = False

    return results

# -------------------------------------------------
# Generación de tabla LaTeX
# -------------------------------------------------

def latex_table(data):
    lines = []

    lines.append(r"\begin{tabular}{lccc ccc ccc}")
    lines.append(r"\toprule")
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
    lines.append(r"\midrule")

    for model, res in sorted(data.items()):
        row = [model.replace("_", r"\_")]

        for cls in CLASSES:
            if cls in res:
                for diff in DIFFICULTIES:
                    row.append(f"{res[cls][diff]:.2f}")
            else:
                row.extend(["-", "-", "-"])

        lines.append(" & ".join(row) + r" \\")

    lines.append(r"\bottomrule")
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
    models_root = run_root / "kitti_models"

    if not models_root.exists():
        print(f"Invalid path: {models_root}")
        sys.exit(1)

    latex_dir = run_root / "latex"
    latex_dir.mkdir(exist_ok=True)

    for dataset_dir in sorted(models_root.iterdir()):
        if not dataset_dir.is_dir():
            continue

        dataset_name = dataset_dir.name
        table_data = {}

        for model_dir in dataset_dir.iterdir():
            if not model_dir.is_dir():
                continue

            model_name = model_dir.name

            logs = sorted(model_dir.rglob("log_eval_*.txt"))
            if not logs:
                continue

            log_path = logs[-1]
            parsed = parse_log(log_path)

            if parsed:
                table_data[model_name] = parsed

        if not table_data:
            continue

        latex = latex_table(table_data)
        out_file = latex_dir / f"table_{dataset_name}.txt"
        out_file.write_text(latex)

        print(f"Generated bbox AP_R40 table: {out_file}")

if __name__ == "__main__":
    main()
