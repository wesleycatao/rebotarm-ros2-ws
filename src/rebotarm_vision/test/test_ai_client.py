"""Unit tests for rebotarm_vision.ai_client helpers."""

from __future__ import annotations

import unittest

from rebotarm_vision.ai_client import (
    _build_prompt,
    _calibration_config_template,
    _expected_count_from_prompt,
    _extract_response_text,
    _vision_profile_from_api_response,
    encode_image_to_png_b64,
)
from rebotarm_vision.vision_profile import Scene, VisionProfile


class TestEncodeImage(unittest.TestCase):
    def test_returns_data_url(self):
        result = encode_image_to_png_b64(b"fake_image")
        self.assertTrue(result.startswith("data:image/png;base64,"))


class TestExtractResponseText(unittest.TestCase):
    def test_direct_output_text(self):
        response = {"output_text": "  {\"scene\": {}}  "}
        self.assertEqual(_extract_response_text(response), "{\"scene\": {}}")

    def test_nested_output(self):
        response = {
            "output": [
                {
                    "content": [
                        {"type": "text", "text": "part1"},
                        {"type": "text", "text": "part2"},
                    ]
                }
            ]
        }
        self.assertEqual(_extract_response_text(response), "part1\npart2")

    def test_empty_raises(self):
        with self.assertRaises(RuntimeError):
            _extract_response_text({})


class TestBuildPrompt(unittest.TestCase):
    def test_expected_count_sums_subgroups(self):
        prompt = "17 garrafas deitadas e 13 garrafas em pé"
        self.assertEqual(_expected_count_from_prompt(prompt), 30)
        self.assertIn(
            "Contagem esperada: 30.",
            _build_prompt(prompt, iteration=0, previous_profile=None, metrics={}, notes=""),
        )

    def test_initial_prompt_sends_config_and_schema_without_manual_preset(self):
        prompt = _build_prompt(
            "25 garrafas PET",
            iteration=0,
            previous_profile=None,
            metrics={},
            notes="",
        )
        self.assertIn("Entrada do operador:\n25 garrafas PET", prompt)
        self.assertIn("Configuração base JSON:", prompt)
        self.assertIn("Schema JSON dos parâmetros aceitos:", prompt)
        self.assertIn('"profiles"', prompt)
        self.assertIn('"brightness_threshold"', prompt)
        self.assertIn('"morph_kernel"', prompt)
        self.assertIn('"workspace_polygon"', prompt)
        self.assertIn("área útil de trabalho", prompt)
        self.assertIn("use_workspace_mask=true", prompt)
        self.assertIn("Reconheça na imagem se as unidades estão em pé", prompt)
        self.assertNotIn("brightness_threshold entre", prompt)
        self.assertNotIn("usar opencv_mode='bright'", prompt)

    def test_refinement_omits_persisted_metadata(self):
        profile = VisionProfile(
            image_hash="large-persisted-hash",
            prompt="persisted prompt",
            model="persisted model",
            scene=Scene(expected_objects=25),
        )
        prompt = _build_prompt(
            "25 garrafas",
            iteration=1,
            previous_profile=profile,
            metrics={"count_error": 2},
            notes="ajustar threshold",
        )
        self.assertIn('"expected_objects":25', prompt)
        self.assertIn('"count_error": 2', prompt)
        self.assertNotIn("large-persisted-hash", prompt)
        self.assertNotIn("persisted model", prompt)

    def test_config_template_contains_only_mutable_response_shape(self):
        template = _calibration_config_template()
        self.assertEqual(
            sorted(template.keys()),
            ["iteration", "merge", "profiles", "reference_objects", "scene"],
        )
        self.assertIn("vertical", template["profiles"])
        self.assertIn("horizontal", template["profiles"])


class TestVisionProfileFromApiResponse(unittest.TestCase):
    def test_minimal_response(self):
        data = {
            "scene": {"expected_objects": 3, "workspace_polygon": []},
            "reference_objects": [],
            "profiles": {
                "vertical": {
                    "opencv_mode": "bright",
                    "active_filters": ["detect_bright", "min_area", "max_area"],
                    "center_method": "moments",
                    "direction_method": "min_area_rect",
                    "direction_sense_method": "none",
                    "shape_filter": "any",
                    "min_area_px": 100.0,
                    "max_area_px": 0.0,
                    "min_aspect_ratio": 0.0,
                    "max_aspect_ratio": 0.0,
                    "morph_kernel": 7,
                    "morph_operation": 1,
                    "roi_margin_px": 0,
                    "brightness_threshold": 105,
                    "brightness_invert": False,
                    "blur_kernel": 5,
                    "canny_low": 45,
                    "canny_high": 130,
                    "edge_aperture_size": 3,
                    "edge_operation": 0,
                    "hsv_lower": [0, 40, 40],
                    "hsv_upper": [179, 255, 255],
                    "rgb_lower": [0, 0, 0],
                    "rgb_upper": [255, 255, 255],
                    "shape_epsilon_ratio": 0.035,
                    "circle_min_circularity": 0.72,
                    "square_aspect_tolerance": 0.22,
                    "lying_aspect_ratio_threshold": 1.8,
                    "min_solidity": 0.0,
                    "min_extent": 0.0,
                    "max_extent": 1.0,
                    "use_workspace_mask": False,
                    "draw_workspace_limit": True,
                    "workspace_limit_shape": "polygon",
                    "workspace_limit_pixels": [],
                    "workspace_circle_pixel": [320.0, 240.0, 180.0],
                    "workspace_mask_margin_px": 0,
                },
                "horizontal": {
                    "opencv_mode": "bright",
                    "active_filters": ["detect_bright", "min_area", "max_area"],
                    "center_method": "moments",
                    "direction_method": "min_area_rect",
                    "direction_sense_method": "none",
                    "shape_filter": "any",
                    "min_area_px": 100.0,
                    "max_area_px": 0.0,
                    "min_aspect_ratio": 0.0,
                    "max_aspect_ratio": 0.0,
                    "morph_kernel": 7,
                    "morph_operation": 1,
                    "roi_margin_px": 0,
                    "brightness_threshold": 105,
                    "brightness_invert": False,
                    "blur_kernel": 5,
                    "canny_low": 45,
                    "canny_high": 130,
                    "edge_aperture_size": 3,
                    "edge_operation": 0,
                    "hsv_lower": [0, 40, 40],
                    "hsv_upper": [179, 255, 255],
                    "rgb_lower": [0, 0, 0],
                    "rgb_upper": [255, 255, 255],
                    "shape_epsilon_ratio": 0.035,
                    "circle_min_circularity": 0.72,
                    "square_aspect_tolerance": 0.22,
                    "lying_aspect_ratio_threshold": 1.8,
                    "min_solidity": 0.0,
                    "min_extent": 0.0,
                    "max_extent": 1.0,
                    "use_workspace_mask": False,
                    "draw_workspace_limit": True,
                    "workspace_limit_shape": "polygon",
                    "workspace_limit_pixels": [],
                    "workspace_circle_pixel": [320.0, 240.0, 180.0],
                    "workspace_mask_margin_px": 0,
                },
            },
            "merge": {"rotated_iou_threshold": 0.45, "center_distance_threshold": 0.05},
            "iteration": {"number": 1, "converged": False, "notes": "test"},
        }
        profile = _vision_profile_from_api_response(data)
        self.assertEqual(profile.schema_version, "1.0.0")
        self.assertEqual(profile.scene.expected_objects, 3)
        self.assertEqual(profile.profiles["vertical"].opencv_mode, "bright")


if __name__ == "__main__":
    unittest.main()
