#!/usr/bin/env python3
"""Inspect a YOLO-format dataset and show statistics / sample images.

Example::

    ros2 run rebotarm_yolo_trainer yolo_inspect --dataset /path/to/data.yaml
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

import cv2
import yaml


YOLO_COLOR = (0, 200, 80)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect a YOLO detection dataset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--dataset",
        "-d",
        type=Path,
        required=True,
        help="Path to data.yaml or dataset root directory.",
    )
    parser.add_argument(
        "--split",
        type=str,
        default="train",
        help="Split to inspect (train/val/test).",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of random samples to display.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Only print statistics, do not open OpenCV windows.",
    )
    return parser.parse_args(argv)


def _resolve_yaml(path: Path) -> Path:
    if path.is_dir():
        candidate = path / "data.yaml"
        if not candidate.exists():
            raise FileNotFoundError(f"Expected {candidate}")
        return candidate
    return path


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def _collect_samples(root: Path, split_images: Path, split_labels: Path) -> list[tuple[Path, Path]]:
    image_dir = root / split_images
    label_dir = root / split_labels
    extensions = {"*.jpg", "*.jpeg", "*.png", "*.bmp"}
    images: list[Path] = []
    for ext in extensions:
        images.extend(image_dir.glob(ext))
        images.extend(image_dir.glob(ext.upper()))

    pairs: list[tuple[Path, Path]] = []
    for image_path in images:
        label_path = label_dir / (image_path.stem + ".txt")
        if label_path.exists():
            pairs.append((image_path, label_path))
    return pairs


def _stats(pairs: list[tuple[Path, Path]], names: dict[int, str]) -> dict:
    class_counts: dict[int, int] = {}
    bbox_count = 0
    empty_labels = 0
    for image_path, label_path in pairs:
        text = label_path.read_text(encoding="utf-8").strip()
        if not text:
            empty_labels += 1
            continue
        for line in text.splitlines():
            parts = line.strip().split()
            if not parts:
                continue
            try:
                class_id = int(parts[0])
            except ValueError:
                continue
            bbox_count += 1
            class_counts[class_id] = class_counts.get(class_id, 0) + 1

    return {
        "images": len(pairs),
        "bboxes": bbox_count,
        "empty_labels": empty_labels,
        "class_counts": {names.get(k, f"class_{k}"): v for k, v in sorted(class_counts.items())},
    }


def _draw_labels(image_path: Path, label_path: Path, names: dict[int, str]) -> None:
    image = cv2.imread(str(image_path))
    if image is None:
        return
    height, width = image.shape[:2]
    text = label_path.read_text(encoding="utf-8").strip()
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) < 5:
            continue
        try:
            class_id = int(parts[0])
            cx, cy, bw, bh = map(float, parts[1:5])
        except ValueError:
            continue
        x1 = int((cx - bw / 2) * width)
        y1 = int((cy - bh / 2) * height)
        x2 = int((cx + bw / 2) * width)
        y2 = int((cy + bh / 2) * height)
        cv2.rectangle(image, (x1, y1), (x2, y2), YOLO_COLOR, 2)
        label = names.get(class_id, f"class_{class_id}")
        cv2.putText(image, label, (x1, max(y1 - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, YOLO_COLOR, 2)
    cv2.imshow("YOLO sample - press any key", image)
    cv2.waitKey(0)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    yaml_path = _resolve_yaml(args.dataset)
    data = _load_yaml(yaml_path)
    root = Path(data.get("path", yaml_path.parent))
    if not root.is_absolute():
        root = yaml_path.parent / root

    names = data.get("names", {})
    if isinstance(names, list):
        names = {i: n for i, n in enumerate(names)}

    split_images = Path(data.get(args.split, f"images/{args.split}"))
    split_labels = Path(str(split_images).replace("images", "labels"))

    pairs = _collect_samples(root, split_images, split_labels)
    stats = _stats(pairs, names)

    print(f"Dataset root : {root}")
    print(f"Split        : {args.split}")
    print(f"Images       : {stats['images']}")
    print(f"BBoxes       : {stats['bboxes']}")
    print(f"Empty labels : {stats['empty_labels']}")
    print("Class counts :")
    for cls, count in stats["class_counts"].items():
        print(f"  {cls}: {count}")

    if args.no_show or not pairs:
        return 0

    random.seed(42)
    for image_path, label_path in random.sample(pairs, min(args.samples, len(pairs))):
        print(f"Showing {image_path.name}")
        _draw_labels(image_path, label_path, names)

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
