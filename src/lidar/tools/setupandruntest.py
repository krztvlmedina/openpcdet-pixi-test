#!/usr/bin/env python3
import sys
import argparse
import subprocess
from pathlib import Path
#Debe ser ejecutado desde raiz de proyecto
# ------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------
OPCDET_TEST_SCRIPT = Path("tools/test.py")

PKL_FILES = [
    "kitti_infos_test.pkl",
    "kitti_infos_train.pkl",
    "kitti_infos_val.pkl",
    "kitti_infos_trainval.pkl",
    "kitti_dbinfos_train.pkl",
]

# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------
def symlink(src: Path, dst: Path):
    if not src.exists():
        raise FileNotFoundError(f"Source does not exist: {src}")
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    dst.symlink_to(src)

def mkdir(path: Path):
    path.mkdir(parents=True, exist_ok=True)

# ------------------------------------------------------------------
# Symlink setup
# ------------------------------------------------------------------
def setup_symlinks(
    dataset_root: Path,
    imageset_root: Path,
    lidar_root: Path,
    workspace: Path,
):
    print("▶ Setting up KITTI symlinks")

    train_velodyne = lidar_root / "training/velodyne"
    test_velodyne  = lidar_root / "testing/velodyne"

    if not train_velodyne.exists():
        raise FileNotFoundError(f"Missing: {train_velodyne}")
    if not test_velodyne.exists():
        raise FileNotFoundError(f"Missing: {test_velodyne}")

    mkdir(workspace)

    # ---- Pickle metadata ----
    for pkl in PKL_FILES:
        symlink(dataset_root / pkl, workspace / pkl)

    # ---- Top-level directories ----
    symlink(imageset_root / "ImageSets",   workspace / "ImageSets")
    symlink(dataset_root / "db_infos",    workspace / "db_infos")
    symlink(dataset_root / "gt_database", workspace / "gt_database")

    # ---- Training ----
    mkdir(workspace / "training")
    symlink(dataset_root / "training/calib",    workspace / "training/calib")
    symlink(dataset_root / "training/image_2",  workspace / "training/image_2")
    symlink(dataset_root / "training/label_2",  workspace / "training/label_2")
    symlink(train_velodyne,                     workspace / "training/velodyne")

    # ---- Testing ----
    mkdir(workspace / "testing")
    symlink(dataset_root / "testing/calib",     workspace / "testing/calib")
    symlink(dataset_root / "testing/image_2",   workspace / "testing/image_2")
    symlink(test_velodyne,                      workspace / "testing/velodyne")

    print("✔ Symlinks ready")

# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="OpenPCDet test wrapper with explicit dataset & LiDAR root",
        add_help=True
    )

    parser.add_argument(
        "--dataset-root",
        required=True,
        type=Path,
        help="Root directory containing KITTI metadata, labels, PKLs"
    )

    parser.add_argument(
        "--image-set-root",
        required=True,
        type=Path,
        help="Root directory containing KITTI images"
    )

    parser.add_argument(
        "--lidar-root",
        required=True,
        type=Path,
        help="Root directory containing training/velodyne and testing/velodyne"
    )

    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path("/OpenPCDet/data/kitti"),
        help="OpenPCDet workspace directory (default: /OpenPCDet/data/kitti)"
    )

    # Everything else goes straight to tools/test.py
    args, passthrough = parser.parse_known_args()

    if not OPCDET_TEST_SCRIPT.exists():
        raise FileNotFoundError(
            "tools/test.py not found. Run this script from the OpenPCDet root."
        )

    setup_symlinks(
        dataset_root=args.dataset_root.resolve(),
        imageset_root=args.image_set_root.resolve(),
        lidar_root=args.lidar_root.resolve(),
        workspace=args.workspace.resolve(),
    )

    cmd = ["python3", str(OPCDET_TEST_SCRIPT), "--save_to_file"] + passthrough

    print("▶ Executing OpenPCDet test:")
    print(" ", " ".join(cmd))

    subprocess.run(cmd, check=True)

if __name__ == "__main__":
    main()
