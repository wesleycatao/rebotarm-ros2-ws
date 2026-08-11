#!/usr/bin/env python3
"""Train a lightweight YOLO model for reBotArm object detection.

This script is meant to run on a normal PC (with a CUDA GPU or CPU). It expects
a YOLO-format dataset described by a ``data.yaml`` file.

Example ``data.yaml``::

    path: /home/user/datasets/rebotarm_pieces
    train: images/train
    val: images/val
    test: images/test

    names:
      0: piece

    # Optional: tell the trainer which augmentation preset to use.
    # rebotarm_orientation_augment: true

Run::

    ros2 run rebotarm_yolo_trainer yolo_train --dataset /path/to/data.yaml

or directly::

    python3 -m rebotarm_yolo_trainer.train --dataset /path/to/data.yaml
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

import yaml


def _check_ultralytics() -> None:
    try:
        import ultralytics  # noqa: F401
    except ImportError as exc:
        print(
            "ERROR: ultralytics is not installed.\n"
            "Install it with:\n"
            "  pip install 'ultralytics>=8.3.0'\n"
        )
        raise SystemExit(1) from exc


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train a lightweight YOLO detector for reBotArm.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--dataset",
        "-d",
        type=Path,
        required=True,
        help="Path to the YOLO data.yaml file.",
    )
    parser.add_argument(
        "--model",
        "-m",
        type=str,
        default="yolo11n.pt",
        help="Base YOLO model to train. Recommended: yolo11n.pt, yolo8n.pt, yolov8n.pt.",
    )
    parser.add_argument(
        "--epochs",
        "-e",
        type=int,
        default=150,
        help="Number of training epochs.",
    )
    parser.add_argument(
        "--imgsz",
        "-s",
        type=int,
        default=640,
        help="Training/input image size. Use 640 for quality, 416 or 320 for speed on the Pi.",
    )
    parser.add_argument(
        "--batch",
        "-b",
        type=int,
        default=16,
        help="Batch size. Auto is used if it fails.",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="0",
        help="Device: '0' for first GPU, 'cpu' for CPU, 'mps' for Apple Silicon.",
    )
    parser.add_argument(
        "--project",
        "-p",
        type=Path,
        default=Path.home() / "rebotarm_yolo_runs",
        help="Directory where training runs are saved.",
    )
    parser.add_argument(
        "--name",
        "-n",
        type=str,
        default="",
        help="Run name. Defaults to 'train_<model>_<timestamp>'.",
    )
    parser.add_argument(
        "--no-orientation-augment",
        action="store_true",
        help="Disable rotation/flip augmentation tuned for lying/standing objects.",
    )
    parser.add_argument(
        "--patience",
        type=int,
        default=30,
        help="Early stopping patience (epochs without improvement).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume the most recent training run.",
    )
    parser.add_argument(
        "--export-ncnn",
        action="store_true",
        default=True,
        help="Export the best weights to NCNN after training (for Raspberry Pi 5).",
    )
    parser.add_argument(
        "--export-onnx",
        action="store_true",
        default=False,
        help="Also export to ONNX format.",
    )
    parser.add_argument(
        "--export-imgsz",
        type=int,
        default=None,
        help="Export image size. Defaults to --imgsz.",
    )
    return parser.parse_args(argv)


def _resolve_data_yaml(dataset: Path) -> Path:
    path = Path(dataset)
    if not path.exists():
        raise FileNotFoundError(f"Dataset file not found: {path}")
    if path.is_dir():
        candidate = path / "data.yaml"
        if not candidate.exists():
            raise FileNotFoundError(f"Expected {candidate} inside dataset directory.")
        return candidate
    return path


def _load_data_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def _validate_dataset(data: dict, yaml_path: Path) -> None:
    errors: list[str] = []
    names = data.get("names")
    if not names:
        errors.append("'names' field is missing in data.yaml")
    if "path" not in data:
        errors.append("'path' field is missing in data.yaml")
    else:
        root = Path(data["path"])
        if not root.is_absolute():
            root = yaml_path.parent / root
        for split in ("train", "val"):
            split_path = data.get(split)
            if not split_path:
                errors.append(f"'{split}' split is missing in data.yaml")
                continue
            full = root / split_path
            if not full.exists():
                errors.append(f"{split} split not found: {full}")
    if errors:
        print("Dataset validation failed:")
        for error in errors:
            print(f"  - {error}")
        raise SystemExit(1)


def _default_run_name(model: str) -> str:
    clean = Path(model).stem
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return f"train_{clean}_{timestamp}"


def _orientation_augmentation_kwargs() -> dict[str, object]:
    """Augmentation tuned for objects in different orientations and partial occlusion."""
    return {
        "hsv_h": 0.015,
        "hsv_s": 0.7,
        "hsv_v": 0.4,
        "degrees": 45.0,
        "translate": 0.1,
        "scale": 0.5,
        "shear": 5.0,
        "perspective": 0.0,
        "flipud": 0.5,
        "fliplr": 0.5,
        "mosaic": 1.0,
        "mixup": 0.05,
        "copy_paste": 0.15,
        "erasing": 0.0,
    }


def _train(args: argparse.Namespace) -> Path:
    from ultralytics import YOLO

    yaml_path = _resolve_data_yaml(args.dataset)
    data = _load_data_yaml(yaml_path)
    _validate_dataset(data, yaml_path)

    project = Path(args.project)
    project.mkdir(parents=True, exist_ok=True)
    name = args.name or _default_run_name(args.model)

    print(f"Dataset : {yaml_path}")
    print(f"Model   : {args.model}")
    print(f"Project : {project}")
    print(f"Name    : {name}")
    print(f"Image size: {args.imgsz}")
    print(f"Epochs  : {args.epochs}")

    model = YOLO(args.model)

    train_kwargs: dict[str, object] = {
        "data": str(yaml_path),
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "patience": args.patience,
        "seed": args.seed,
        "project": str(project),
        "name": name,
        "exist_ok": True,
        "device": args.device,
        "batch": args.batch,
        "save": True,
        "plots": True,
    }

    if not args.no_orientation_augment:
        train_kwargs.update(_orientation_augmentation_kwargs())
        print("Orientation augmentation enabled (lying/standing/occlusion aware).")

    if args.resume:
        train_kwargs["resume"] = True

    model.train(**train_kwargs)

    run_dir = project / name
    best_pt = run_dir / "weights" / "best.pt"
    if not best_pt.exists():
        # Older Ultralytics may place weights under train/weights/best.pt
        best_pt = run_dir / "train" / "weights" / "best.pt"
    if not best_pt.exists():
        raise FileNotFoundError(f"Trained weights not found in {run_dir}")

    print(f"\nTraining complete. Best weights: {best_pt}")
    return Path(best_pt)


def _export(best_pt: Path, args: argparse.Namespace) -> dict[str, Path]:
    from ultralytics import YOLO

    exported: dict[str, Path] = {}
    export_imgsz = args.export_imgsz or args.imgsz
    model = YOLO(str(best_pt))

    if args.export_ncnn:
        print(f"\nExporting to NCNN (imgsz={export_imgsz}) for Raspberry Pi 5...")
        try:
            ncnn_path = model.export(format="ncnn", imgsz=export_imgsz, half=False)
            exported["ncnn"] = Path(ncnn_path)
            print(f"NCNN model saved to: {ncnn_path}")
        except Exception as exc:
            print(f"WARNING: NCNN export failed: {exc}")

    if args.export_onnx:
        print(f"\nExporting to ONNX (imgsz={export_imgsz})...")
        try:
            onnx_path = model.export(format="onnx", imgsz=export_imgsz, half=False)
            exported["onnx"] = Path(onnx_path)
            print(f"ONNX model saved to: {onnx_path}")
        except Exception as exc:
            print(f"WARNING: ONNX export failed: {exc}")

    return exported


def _copy_for_deployment(best_pt: Path, exported: dict[str, Path], deploy_dir: Path) -> None:
    """Collect all useful artefacts into a single deployment folder."""
    deploy_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nCollecting deployment artefacts in: {deploy_dir}")

    shutil.copy2(best_pt, deploy_dir / "best.pt")
    for fmt, path in exported.items():
        if path.is_dir():
            dest = deploy_dir / path.name
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(path, dest)
            print(f"  copied {fmt}: {dest}")
        else:
            shutil.copy2(path, deploy_dir / path.name)
            print(f"  copied {fmt}: {deploy_dir / path.name}")

    # Write a small manifest so the Pi launch file knows what to load.
    manifest = {
        "pt": str(deploy_dir / "best.pt"),
        "ncnn": str(deploy_dir / exported.get("ncnn", best_pt).name) if "ncnn" in exported else None,
        "onnx": str(deploy_dir / exported["onnx"].name) if "onnx" in exported else None,
    }
    manifest_path = deploy_dir / "manifest.yaml"
    with manifest_path.open("w", encoding="utf-8") as file:
        yaml.dump(manifest, file, default_flow_style=False, sort_keys=False)
    print(f"  manifest: {manifest_path}")


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    _check_ultralytics()

    best_pt = _train(args)
    exported = _export(best_pt, args)

    deploy_dir = Path(args.project) / (args.name or _default_run_name(args.model)) / "deploy"
    _copy_for_deployment(best_pt, exported, deploy_dir)

    print("\n" + "=" * 60)
    print("All done.")
    print(f"PyTorch weights: {deploy_dir / 'best.pt'}")
    if "ncnn" in exported:
        print(f"NCNN model     : {deploy_dir / exported['ncnn'].name}")
        print("\nTo use on the Raspberry Pi 5, copy the NCNN folder to the Pi and set")
        print(f"  model_path: {deploy_dir / exported['ncnn'].name}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
