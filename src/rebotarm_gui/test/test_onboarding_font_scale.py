"""Regression coverage for commissioning onboarding and DPI font scaling."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from rebotarm_gui.settings import (  # noqa: E402
    AppSettings,
    COMMISSIONING_CHECKLIST_KEYS,
    load_app_settings,
    save_app_settings,
)
from rebotarm_gui.styles.theme import build_stylesheet  # noqa: E402


class TestPersistentUiGuidanceSettings(unittest.TestCase):
    def test_font_scale_and_commissioning_progress_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "settings.json"
            settings = AppSettings(font_scale_percent=120)
            settings.commissioning_checklist["calibration_approved"] = True
            settings.commissioning_checklist["vision_validated"] = True

            save_app_settings(settings, path)
            loaded = load_app_settings(path)

            self.assertEqual(loaded.font_scale_percent, 120)
            self.assertTrue(
                loaded.commissioning_checklist["calibration_approved"]
            )
            self.assertTrue(loaded.commissioning_checklist["vision_validated"])
            self.assertFalse(
                loaded.commissioning_checklist["cell_scene_applied"]
            )
            self.assertEqual(
                set(loaded.commissioning_checklist),
                set(COMMISSIONING_CHECKLIST_KEYS),
            )

    def test_invalid_scale_is_clamped_and_unknown_steps_are_discarded(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "settings.json"
            save_app_settings(AppSettings(), path)
            data = json.loads(path.read_text(encoding="utf-8"))
            data["font_scale_percent"] = 999
            data["commissioning_checklist"] = {
                "calibration_approved": 1,
                "unknown_future_step": True,
            }
            path.write_text(json.dumps(data), encoding="utf-8")

            loaded = load_app_settings(path)

            self.assertEqual(loaded.font_scale_percent, 130)
            self.assertTrue(
                loaded.commissioning_checklist["calibration_approved"]
            )
            self.assertNotIn(
                "unknown_future_step", loaded.commissioning_checklist
            )

    def test_api_settings_are_discarded_when_legacy_file_is_saved(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "settings.json"
            path.write_text(
                json.dumps(
                    {
                        "operator_id": "operator",
                        "openai": {
                            "api_key": "legacy-secret",
                            "save_api_key": True,
                            "model": "legacy-model",
                            "base_url": "https://example.invalid",
                            "timeout_s": 90,
                        },
                    }
                ),
                encoding="utf-8",
            )

            settings = load_app_settings(path)
            save_app_settings(settings, path)
            persisted = json.loads(path.read_text(encoding="utf-8"))

            self.assertEqual(settings.operator_id, "operator")
            self.assertNotIn("openai", persisted)
            self.assertNotIn("legacy-secret", path.read_text(encoding="utf-8"))


class TestScaledStylesheet(unittest.TestCase):
    def test_all_point_sizes_scale_proportionally(self) -> None:
        normal = build_stylesheet("dark", 100)
        compact = build_stylesheet("dark", 90)
        large = build_stylesheet("dark", 130)

        self.assertIn("font-size: 10pt;", normal)
        self.assertIn("font-size: 9pt;", compact)
        self.assertIn("font-size: 13pt;", large)
        self.assertIn("font-size: 20.8pt;", large)  # 16pt page title
        self.assertNotEqual(normal, compact)
        self.assertNotEqual(normal, large)

    def test_scale_is_bounded_at_supported_limits(self) -> None:
        self.assertEqual(
            build_stylesheet("light", 10),
            build_stylesheet("light", 90),
        )
        self.assertEqual(
            build_stylesheet("light", 999),
            build_stylesheet("light", 130),
        )


if __name__ == "__main__":
    unittest.main()
