"""Tests for the unified YOLO/workspace/pick JSON contract."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from rebotarm_vision.task_config import VisionTaskConfig


class TestVisionTaskConfig(unittest.TestCase):
    def test_roundtrip_and_sanitization(self) -> None:
        config = VisionTaskConfig.from_dict(
            {
                "detector": {
                    "backend": "yolo",
                    "model_path": "best.pt",
                    "confidence_percent": 120,
                    "target_classes": "bottle, cap",
                },
                "object": {
                    "orientation": "horizontal",
                    "auxiliary_attribute": "PET",
                },
                "workspace": {
                    "shape": "polygon",
                    "vertices": [[0, 0], [100, 0], [100, 80], [0, 80]],
                    "clearance_px": 12,
                },
                "grasp": {
                    "clearance_px": 4,
                    "min_clearance_horizontal_px": 16,
                    "min_clearance_vertical_px": 80,
                },
                "pick_route": {"strategy": "nearest_neighbor"},
            }
        )
        self.assertEqual(config.detector.confidence_percent, 100.0)
        self.assertEqual(config.detector.target_classes, ["bottle", "cap"])
        self.assertTrue(config.workspace.enabled)
        self.assertEqual(config.grasp.min_clearance_vertical_px, 80.0)
        self.assertEqual(config.pick_route.strategy, "nearest_neighbor")
        restored = VisionTaskConfig.from_json(config.to_json())
        self.assertEqual(restored.object.auxiliary_attribute, "PET")
        self.assertEqual(restored.workspace.vertices[2], [100.0, 80.0])

    def test_ignores_removed_alias_fields(self) -> None:
        config = VisionTaskConfig.from_dict(
            {
                "confidence_threshold": 0.42,
                "workspace_limit": {
                    "enabled": True,
                    "shape": "polygon",
                    "points": [0, 0, 10, 0, 10, 10],
                    "margin_px": 3,
                },
            }
        )
        self.assertEqual(config.detector.confidence_percent, 35.0)
        self.assertEqual(config.workspace.vertices, [])
        self.assertEqual(config.workspace.clearance_px, 0.0)

    def test_file_roundtrip(self) -> None:
        config = VisionTaskConfig()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = config.save(Path(tmpdir) / "config.json")
            loaded = VisionTaskConfig.load(path)
        self.assertEqual(loaded.schema_version, 3)


if __name__ == "__main__":
    unittest.main()
