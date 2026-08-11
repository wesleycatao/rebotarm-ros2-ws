"""Unit tests for SegmentationNode helpers."""

from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import cv2
from std_msgs.msg import Header

from rebotarm_vision.segmentation_node import SegmentationNode


DEFAULT_PARAMS = {
    "calibrated_profile_json": "{}",
    "canny_high": 130,
    "canny_low": 45,
    "center_method": "moments",
    "circle_min_circularity": 0.72,
    "brightness_invert": False,
    "direction_method": "min_area_rect",
    "direction_sense_method": "none",
    "draw_clearance": False,
    "draw_workspace_limit": True,
    "edge_aperture_size": 3,
    "edge_dilate_iterations": 1,
    "edge_erode_iterations": 0,
    "grasp_inset_ratio": 0.10,
    "gripper_width_px": 0.0,
    "hsv_lower": [0, 40, 40],
    "hsv_upper": [179, 255, 255],
    "lying_aspect_ratio_threshold": 1.8,
    "max_area_px": 0.0,
    "max_area_ratio": 0.50,
    "max_aspect_ratio": 0.0,
    "max_bbox_long_side_px": 0.0,
    "max_bbox_short_side_px": 0.0,
    "max_contour_points": 48,
    "max_extent": 1.0,
    "min_area_px": 500.0,
    "min_aspect_ratio": 0.0,
    "min_bbox_long_side_px": 0.0,
    "min_bbox_short_side_px": 0.0,
    "min_extent": 0.0,
    "min_solidity": 0.0,
    "morph_close_iterations": 1,
    "morph_kernel": 5,
    "morph_open_iterations": 1,
    "opencv_mode": "hsv",
    "rgb_lower": [0, 0, 0],
    "rgb_upper": [255, 255, 255],
    "roi_margin_px": 0,
    "shape_epsilon_ratio": 0.035,
    "shape_filter": "any",
    "square_aspect_tolerance": 0.22,
    "use_workspace_mask": False,
    "use_orientation_profiles": False,
    "vertical_profile_json": "{}",
    "horizontal_profile_json": "{}",
    "workspace_circle_pixel": [320.0, 240.0, 180.0],
    "workspace_limit_pixels": [],
    "workspace_limit_shape": "polygon",
    "workspace_mask_margin_px": 0.0,
}


class OfflineSegmentationHarness(SegmentationNode):
    """Use SegmentationNode's OpenCV implementation without creating a ROS node."""

    def __init__(self, params: dict):
        self._params = {**DEFAULT_PARAMS, **params}
        self._latest_frame = None
        self._profile_overrides = None

    def get_parameter(self, name: str):
        return SimpleNamespace(value=self._params[name])


class TestClassifyOrientation(unittest.TestCase):
    def test_horizontal(self):
        # Long axis near 0 radians.
        self.assertEqual(
            SegmentationNode._classify_orientation(0.0, 100.0, 20.0, tolerance_deg=20.0),
            "horizontal",
        )
        self.assertEqual(
            SegmentationNode._classify_orientation(math.radians(170), 100.0, 20.0, tolerance_deg=20.0),
            "horizontal",
        )

    def test_vertical(self):
        # Long axis near 90 degrees; note width < height shifts angle by 90 deg.
        self.assertEqual(
            SegmentationNode._classify_orientation(math.radians(0), 20.0, 100.0, tolerance_deg=20.0),
            "vertical",
        )

    def test_ambiguous(self):
        # 45 degrees is ambiguous with default tolerance.
        self.assertEqual(
            SegmentationNode._classify_orientation(math.radians(45), 100.0, 20.0, tolerance_deg=20.0),
            "ambiguous",
        )


class TestBundledBottleImage(unittest.TestCase):
    def test_mixed_orientation_config_detects_25_bottles(self):
        package_root = Path(__file__).resolve().parents[1]
        image_path = package_root / "test_images" / "garrafas.png"
        config_path = package_root / "config" / "vision_bottles_mixed_orientation_config.json"
        frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        self.assertIsNotNone(frame)

        config = json.loads(config_path.read_text(encoding="utf-8"))
        segmenter = OfflineSegmentationHarness(config)
        segmenter._latest_frame = frame
        objects, _debug = segmenter._segment_with_opencv(frame, Header())

        self.assertEqual(len(objects), 25)
        self.assertGreater(min(obj.area_px for obj in objects), 7000.0)


if __name__ == "__main__":
    unittest.main()
