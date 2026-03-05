import sys
from pathlib import Path

from generate_latex_tables import parse_log, latex_ap_table, latex_recall_table

# -------------------------------------------------
# Main
# -------------------------------------------------
# Handles the flat output structure produced by eval_all_downsampled.sh,
# where models are stored directly under kitti_models/ with no dataset
# subdirectory:
#
#   <run_root>/kitti_models/{model}/default/eval/.../log_eval_*.txt
#
# Generates:
#   <run_root>/latex/table_ap_r40.tex
#   <run_root>/latex/table_recall.tex

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 generate_latex_single_dataset.py <output/date>")
        sys.exit(1)

    run_root = Path(sys.argv[1])
    models_root = run_root

    if not models_root.exists():
        print(f"Invalid path: {models_root}")
        sys.exit(1)

    latex_dir = run_root / "latex"
    latex_dir.mkdir(exist_ok=True)

    ap_data = {}
    recall_data = {}

    for model_dir in sorted(models_root.iterdir()):
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
        print("No evaluation logs found.")
        sys.exit(1)

    if ap_data:
        out_ap = latex_dir / "table_ap_r40.tex"
        out_ap.write_text(latex_ap_table(ap_data))
        print(f"Generated AP table:     {out_ap}")

    if recall_data:
        out_recall = latex_dir / "table_recall.tex"
        out_recall.write_text(latex_recall_table(recall_data))
        print(f"Generated recall table: {out_recall}")

if __name__ == "__main__":
    main()
