"""Unit tests for rebotarm_vision.vision_profile."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from rebotarm_vision.vision_profile import (
    ASPECT_RATIO_MAX,
    HUE_MAX,
    MORPH_OPERATION_MAX,
    UINT8_MAX,
    CalibratedProfile,
    MergeSettings,
    ReferenceObject,
    RotatedBBox,
    Scene,
    VisionProfile,
    api_response_schema,
    apply_scene_workspace_polygon,
    image_hash,
    load_profile,
    profile_to_segmentation_params,
    save_profile,
    sanitize_profile,
)


class TestImageHash(unittest.TestCase):
    def test_stable_and_unique(self):
        data = b"not really an image"
        self.assertEqual(image_hash(data), image_hash(data))
        self.assertNotEqual(image_hash(data), image_hash(data + b"x"))


class TestScene(unittest.TestCase):
    def test_roundtrip(self):
        scene = Scene(expected_objects=12, workspace_polygon=[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        restored = Scene.from_dict(scene.to_dict())
        self.assertEqual(restored.expected_objects, 12)
        self.assertEqual(restored.workspace_polygon, [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])


class TestReferenceObject(unittest.TestCase):
    def test_roundtrip(self):
        obj = ReferenceObject(
            class_name="garrafa_pet",
            bbox_rotated=RotatedBBox(cx=0.25, cy=0.18, w=0.16, h=0.07, angle_deg=18.0),
            orientation="horizontal",
            grasp_zone=[[-0.25, -0.35], [0.25, -0.35], [0.25, 0.35], [-0.25, 0.35]],
            grasp_inset_ratio=0.10,
            confidence=0.98,
        )
        restored = ReferenceObject.from_dict(obj.to_dict())
        self.assertEqual(restored.class_name, "garrafa_pet")
        self.assertAlmostEqual(restored.bbox_rotated.cx, 0.25)
        self.assertEqual(restored.orientation, "horizontal")
        self.assertAlmostEqual(restored.grasp_inset_ratio, 0.10)
        self.assertAlmostEqual(restored.confidence, 0.98)

    def test_invalid_orientation_becomes_ambiguous(self):
        obj = ReferenceObject.from_dict({"orientation": "sideways"})
        self.assertEqual(obj.orientation, "ambiguous")


class TestCalibratedProfile(unittest.TestCase):
    def test_sanitisation(self):
        profile = CalibratedProfile(
            opencv_mode="invalid_mode",
            brightness_threshold=300,
            blur_kernel=4,
            morph_kernel=2,
            morph_operation=20,
            canny_low=500,
            hsv_lower=[-10, 300, -5],
            hsv_upper=[200, 255, 255],
            min_aspect_ratio=50.0,
            max_aspect_ratio=-5.0,
            max_extent=-1.0,
        )
        sanitized = sanitize_profile(profile, 640, 480)
        self.assertEqual(sanitized.opencv_mode, "bright")
        self.assertEqual(sanitized.brightness_threshold, UINT8_MAX)
        self.assertEqual(sanitized.blur_kernel, 5)  # even -> next odd
        self.assertEqual(sanitized.morph_kernel, 3)  # even -> next odd
        self.assertEqual(sanitized.morph_operation, MORPH_OPERATION_MAX)
        self.assertEqual(sanitized.canny_low, UINT8_MAX)
        self.assertEqual(sanitized.hsv_lower, [0, UINT8_MAX, 0])
        self.assertEqual(sanitized.hsv_upper, [HUE_MAX, UINT8_MAX, UINT8_MAX])
        self.assertEqual(sanitized.min_aspect_ratio, ASPECT_RATIO_MAX)
        self.assertEqual(sanitized.max_aspect_ratio, 0.0)
        self.assertEqual(sanitized.max_extent, 0.0)  # negative clamped to lower bound = disabled

    def test_active_filters_normalised(self):
        profile = CalibratedProfile(opencv_mode="hsv_or_edge", active_filters=["min_area"])
        sanitized = sanitize_profile(profile, 640, 480)
        self.assertIn("detect_hsv", sanitized.active_filters)
        self.assertIn("detect_edge", sanitized.active_filters)
        self.assertIn("min_area", sanitized.active_filters)
        self.assertIn("max_area", sanitized.active_filters)

    def test_to_segmentation_params(self):
        profile = CalibratedProfile(
            morph_operation=-2,
            edge_operation=3,
        )
        params = profile_to_segmentation_params(profile)
        self.assertEqual(params["morph_open_iterations"], 2)
        self.assertEqual(params["morph_close_iterations"], 0)
        self.assertEqual(params["edge_dilate_iterations"], 3)
        self.assertEqual(params["edge_erode_iterations"], 0)


class TestVisionProfile(unittest.TestCase):
    def test_roundtrip(self):
        profile = VisionProfile(
            image_hash="abc123",
            image_width=640,
            image_height=480,
            prompt="detect bottles",
            model="gpt-5-mini",
            scene=Scene(expected_objects=5),
            reference_objects=[
                ReferenceObject(
                    class_name="bottle",
                    bbox_rotated=RotatedBBox(cx=0.5, cy=0.5, w=0.1, h=0.04, angle_deg=0.0),
                )
            ],
            merge=MergeSettings(rotated_iou_threshold=0.5),
        )
        restored = VisionProfile.from_dict(profile.to_dict())
        self.assertEqual(restored.image_hash, "abc123")
        self.assertEqual(restored.scene.expected_objects, 5)
        self.assertEqual(len(restored.reference_objects), 1)
        self.assertEqual(restored.profiles["vertical"].opencv_mode, "bright")

    def test_save_and_load(self):
        profile = VisionProfile(image_hash="hash_test", prompt="prompt")
        with tempfile.TemporaryDirectory() as tmpdir:
            path = save_profile(profile, Path(tmpdir))
            self.assertTrue(path.exists())
            loaded = load_profile(path)
            self.assertEqual(loaded.image_hash, "hash_test")
            self.assertEqual(loaded.prompt, "prompt")

    def test_scene_workspace_polygon_populates_detector_profiles(self):
        profile = VisionProfile(
            scene=Scene(
                workspace_polygon=[
                    [60.0, 70.0],
                    [1378.0, 70.0],
                    [1378.0, 1005.0],
                    [60.0, 1005.0],
                ]
            )
        )
        apply_scene_workspace_polygon(profile)

        expected = [60.0, 70.0, 1378.0, 70.0, 1378.0, 1005.0, 60.0, 1005.0]
        for detector_profile in profile.profiles.values():
            self.assertTrue(detector_profile.use_workspace_mask)
            self.assertTrue(detector_profile.draw_workspace_limit)
            self.assertEqual(detector_profile.workspace_limit_shape, "polygon")
            self.assertEqual(detector_profile.workspace_limit_pixels, expected)


class TestApiSchema(unittest.TestCase):
    def test_schema_structure(self):
        schema = api_response_schema()
        self.assertEqual(schema["type"], "object")
        self.assertIn("profiles", schema["properties"])
        self.assertIn("vertical", schema["properties"]["profiles"]["properties"])
        self.assertIn("reference_objects", schema["properties"])
        scene = schema["properties"]["scene"]["properties"]
        self.assertIn("area of interest", scene["workspace_polygon"]["description"])

    def test_schema_serialises(self):
        schema = api_response_schema()
        text = json.dumps(schema)
        self.assertIsInstance(text, str)


if __name__ == "__main__":
    unittest.main()
