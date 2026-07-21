#!/usr/bin/env python3
"""Export an already trained YOLO model to deployment formats.

Useful when you want to re-export a trained ``best.pt`` with a different
input size (e.g. 320 for the Raspberry Pi) without retraining.

Example::

    ros2 run rebotarm_yolo_trainer yolo_export \
        --weights ~/rebotarm_yolo_runs/train_yolo11n_20260115_143000/weights/best.pt \
        --imgsz 320 \
        --ncnn
"""

from __future__ import annotations

import argparse
import shutil
import sys
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
        description="Export a trained YOLO model to NCNN/ONNX.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--weights",
        "-w",
        type=Path,
        required=True,
        help="Path to the trained best.pt file.",
    )
    parser.add_argument(
        "--imgsz",
        "-s",
        type=int,
        default=320,
        help="Export image size. 320 is a good compromise for the Raspberry Pi 5.",
    )
    parser.add_argument(
        "--ncnn",
        action="store_true",
        default=True,
        help="Export to NCNN.",
    )
    parser.add_argument(
        "--onnx",
        action="store_true",
        default=False,
        help="Export to ONNX.",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        default=False,
        help="Use FP16 (not recommended for Raspberry Pi).",
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=None,
        help="Output directory. Defaults to the weights directory.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    _check_ultralytics()

    weights = Path(args.weights)
    if not weights.exists():
        print(f"ERROR: weights not found: {weights}")
        return 1

    output_dir = Path(args.output_dir) if args.output_dir else weights.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    from ultralytics import YOLO

    model = YOLO(str(weights))
    exported: dict[str, Path] = {}

    if args.ncnn:
        print(f"Exporting {weights} to NCNN (imgsz={args.imgsz})...")
        path = model.export(format="ncnn", imgsz=args.imgsz, half=args.half)
        exported["ncnn"] = Path(path)

    if args.onnx:
        print(f"Exporting {weights} to ONNX (imgsz={args.imgsz})...")
        path = model.export(format="onnx", imgsz=args.imgsz, half=args.half)
        exported["onnx"] = Path(path)

    if not exported:
        print("No export format selected. Use --ncnn and/or --onnx.")
        return 1

    print(f"\nExported artefacts saved to: {output_dir}")
    for fmt, path in exported.items():
        if path.is_dir():
            dest = output_dir / path.name
            if dest.exists():
                shutil.rmtree(dest)
            shutil.copytree(path, dest)
            print(f"  {fmt}: {dest}")
        else:
            dest = output_dir / path.name
            shutil.copy2(path, dest)
            print(f"  {fmt}: {dest}")

    manifest = {
        "source": str(weights),
        "imgsz": args.imgsz,
        "ncnn": str(output_dir / exported["ncnn"].name) if "ncnn" in exported else None,
        "onnx": str(output_dir / exported["onnx"].name) if "onnx" in exported else None,
    }
    manifest_path = output_dir / "manifest.yaml"
    with manifest_path.open("w", encoding="utf-8") as file:
        yaml.dump(manifest, file, default_flow_style=False, sort_keys=False)
    print(f"  manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
