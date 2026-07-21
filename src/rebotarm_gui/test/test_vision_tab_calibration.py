"""Unit tests for the VisionTab calibration integration."""

from __future__ import annotations

import os
import sys
import json
import tempfile
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QObject, Signal  # noqa: E402
from PySide6.QtGui import QImage  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_vision.vision_profile import (  # noqa: E402
    CalibratedProfile,
    ProfileStatus,
    VisionProfile,
)

from rebotarm_gui.settings import AppSettings  # noqa: E402
from rebotarm_gui.tabs.vision_tab import VisionTab  # noqa: E402


class MockBridge(QObject):
    """Minimal bridge exposing only the signals VisionTab connects to."""

    vision_calibration_progress = Signal(str)
    vision_calibration_result = Signal(str, str)
    service_result = Signal(str, bool, str)

    def __init__(self) -> None:
        super().__init__()
        self.published_images: list[tuple[int, int, bytes]] = []
        self.process_calls = 0
        self.calibrate_calls: list[tuple[object, str, str]] = []
        self.vision_params_calls: list[dict[str, object]] = []
        self.vision_pose_params_calls: list[dict[str, object]] = []
        self.pick_scheduler_params_calls: list[dict[str, object]] = []

    def vision_set_parameters(self, params: dict[str, object]) -> None:
        self.vision_params_calls.append(dict(params))

    def vision_pose_set_parameters(self, params: dict[str, object]) -> None:
        self.vision_pose_params_calls.append(dict(params))

    def pick_scheduler_set_parameters(self, params: dict[str, object]) -> None:
        self.pick_scheduler_params_calls.append(dict(params))

    def publish_vision_test_image(self, width: int, height: int, data: bytes, _step: int) -> None:
        self.published_images.append((int(width), int(height), bytes(data)))

    def vision_process_frame(self) -> bool:
        self.process_calls += 1
        return True

    def vision_calibrate(self, image, prompt: str, api_key: str = "") -> bool:
        self.calibrate_calls.append((image, prompt, api_key))
        return True


class TestVisionTabCalibration(unittest.TestCase):
    """Tests for the calibration UI flow."""

    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.bridge = MockBridge()
        self.settings = AppSettings()
        self.tab = VisionTab(self.bridge, ros_log=None, settings=self.settings)

    def tearDown(self) -> None:
        self.tab.deleteLater()

    def test_qimage_to_ros_image_rgb(self) -> None:
        """Converting a RGB QImage must produce a valid sensor_msgs/Image."""
        image = QImage(10, 8, QImage.Format.Format_RGB888)
        image.fill(0)
        rgb = image.convertToFormat(QImage.Format.Format_RGB888)
        msg = VisionTab._qimage_to_ros_image(image)
        self.assertIsNotNone(msg)
        assert msg is not None
        self.assertEqual(msg.width, 10)
        self.assertEqual(msg.height, 8)
        self.assertEqual(msg.encoding, "rgb8")
        self.assertEqual(msg.step, rgb.bytesPerLine())
        self.assertEqual(len(msg.data), msg.height * msg.step)

    def test_calibration_buttons_initially_disabled(self) -> None:
        """Approval/Complement should be disabled before a calibration finishes."""
        self.assertFalse(self.tab._btn_approve_calibration.isEnabled())
        self.assertFalse(self.tab._btn_retry_calibration.isEnabled())
        self.assertFalse(self.tab._btn_complement_calibration.isEnabled())

    def test_approve_calibration_disables_button(self) -> None:
        """Approving a calibration must disable the approve button and update status."""
        self.tab._btn_approve_calibration.setEnabled(True)
        self.tab._approve_calibration()
        self.assertIn("aprovada", self.tab._autotune_status.text().lower())
        self.assertFalse(self.tab._btn_approve_calibration.isEnabled())

    def test_complement_calibration_updates_status(self) -> None:
        """Complement text should explain the next step to the operator."""
        self.tab._complement_calibration_text()
        self.assertIn("prompt", self.tab._autotune_status.text().lower())

    def test_calibration_result_populates_profiles(self) -> None:
        """A converged profile returned by the service must populate orientation profiles."""
        self.tab._test_image = None
        profile = VisionProfile(
            profiles={
                "vertical": CalibratedProfile(
                    opencv_mode="bright",
                    active_filters=["detect_bright"],
                    min_area_px=1000.0,
                ),
                "horizontal": CalibratedProfile(
                    opencv_mode="edge",
                    active_filters=["detect_edge"],
                    min_area_px=2000.0,
                ),
            },
            status=ProfileStatus(converged=True, reason="ok"),
        )
        self.tab._on_ai_calibrate_result(profile.to_json(), "report")
        self.assertIn("vertical", self.tab._orientation_profiles)
        self.assertIn("horizontal", self.tab._orientation_profiles)
        self.assertEqual(self.tab._orientation_profiles["vertical"]["min_area_px"], 1000.0)
        self.assertTrue(self.tab._btn_approve_calibration.isEnabled())
        self.assertTrue(self.tab._btn_retry_calibration.isEnabled())
        self.assertTrue(self.tab._btn_complement_calibration.isEnabled())

    def test_calibration_result_not_converged_keeps_approve_disabled(self) -> None:
        """A non-converged profile must not allow approval."""
        self.tab._test_image = None
        profile = VisionProfile(
            profiles={"vertical": CalibratedProfile(opencv_mode="bright")},
            status=ProfileStatus(converged=False, reason="low coverage"),
        )
        self.tab._on_ai_calibrate_result(profile.to_json(), "")
        self.assertFalse(self.tab._btn_approve_calibration.isEnabled())

    def test_calibration_result_republishes_original_calibration_image(self) -> None:
        """Changing the test image mid-calibration must not affect result processing."""
        calibration_image = QImage(2, 2, QImage.Format.Format_RGB888)
        calibration_image.fill(0x112233)
        changed_image = QImage(3, 3, QImage.Format.Format_RGB888)
        changed_image.fill(0x445566)
        self.tab._last_calibration_image = calibration_image
        self.tab._test_image = changed_image

        profile = VisionProfile(
            profiles={"vertical": CalibratedProfile(opencv_mode="bright")},
            status=ProfileStatus(converged=True, reason="ok"),
        )
        self.tab._on_ai_calibrate_result(profile.to_json(), "")

        self.assertTrue(self.bridge.published_images)
        width, height, _data = self.bridge.published_images[-1]
        self.assertEqual((width, height), (2, 2))

    def test_start_calibration_forwards_api_key_to_bridge(self) -> None:
        """The GUI-held API key must be sent with the calibration request."""
        self.settings.openai.api_key = "sk-test"
        self.tab._test_image = QImage(2, 2, QImage.Format.Format_RGB888)
        self.tab._test_image.fill(0x112233)

        self.tab._start_ai_autotune()

        self.assertTrue(self.bridge.calibrate_calls)
        _image, _prompt, api_key = self.bridge.calibrate_calls[-1]
        self.assertEqual(api_key, "sk-test")

    def test_external_profile_files_are_applied_with_workspace(self) -> None:
        """The new flow loads one JSON per orientation and applies both to ROS."""
        workspace = [10.0, 20.0, 300.0, 20.0, 300.0, 200.0, 10.0, 200.0]
        horizontal = {
            "opencv_mode": "edge",
            "active_filters": ["detect_edge"],
            "direction_sense_method": "positive_x",
            "min_area_px": 1000.0,
            "use_workspace_mask": True,
            "workspace_limit_shape": "polygon",
            "workspace_limit_pixels": workspace,
        }
        vertical = {
            "opencv_mode": "bright",
            "active_filters": ["detect_bright"],
            "direction_sense_method": "none",
            "min_area_px": 500.0,
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            horizontal_path = Path(tmpdir) / "config_perfil_A.json"
            vertical_path = Path(tmpdir) / "config_perfil_B.json"
            horizontal_path.write_text(json.dumps(horizontal), encoding="utf-8")
            vertical_path.write_text(json.dumps(vertical), encoding="utf-8")

            self.tab._load_orientation_profile(str(horizontal_path), "horizontal")
            self.tab._load_orientation_profile(str(vertical_path), "vertical")
            self.tab._apply_external_profiles()

        calls = [
            params for params in self.bridge.vision_params_calls
            if params.get("use_orientation_profiles") is True
        ]
        self.assertTrue(calls)
        params = calls[-1]
        self.assertTrue(params["use_orientation_profiles"])
        horizontal_params = json.loads(str(params["horizontal_profile_json"]))
        vertical_params = json.loads(str(params["vertical_profile_json"]))
        self.assertEqual(horizontal_params["opencv_mode"], "edge")
        self.assertEqual(vertical_params["opencv_mode"], "bright")
        self.assertEqual(horizontal_params["workspace_limit_pixels"], workspace)
        self.assertEqual(vertical_params["workspace_limit_pixels"], workspace)

    def test_unified_json_applies_yolo_workspace_and_pick_route(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            (root / "best.pt").write_bytes(b"placeholder")
            config_path = root / "vision_pick.json"
            config_path.write_text(
                json.dumps(
                    {
                        "schema_version": 3,
                        "detector": {
                            "backend": "yolo",
                            "model_path": "best.pt",
                            "confidence_percent": 73,
                            "image_size": 320,
                            "target_classes": ["bottle"],
                        },
                        "object": {
                            "orientation": "horizontal",
                            "auxiliary_attribute": "PET",
                        },
                        "workspace": {
                            "enabled": True,
                            "shape": "circle",
                            "circle": [200, 150, 100],
                            "clearance_px": 8,
                        },
                        "grasp": {
                            "clearance_px": 5,
                            "min_clearance_horizontal_px": 12,
                            "min_clearance_vertical_px": 50,
                        },
                        "pick_route": {
                            "strategy": "nearest_neighbor",
                            "max_objects": 6,
                        },
                    }
                ),
                encoding="utf-8",
            )
            self.tab._load_task_config(str(config_path))
            self.tab._apply_task_configuration()

        vision = next(
            params
            for params in reversed(self.bridge.vision_params_calls)
            if "backend" in params
        )
        self.assertEqual(vision["backend"], "yolo")
        self.assertEqual(vision["confidence_threshold"], 0.73)
        self.assertEqual(vision["target_classes"], ["bottle"])
        self.assertEqual(vision["workspace_limit_shape"], "circle")
        self.assertTrue(self.bridge.pick_scheduler_params_calls)
        scheduler = self.bridge.pick_scheduler_params_calls[-1]
        self.assertEqual(scheduler["strategy"], "nearest_neighbor")
        self.assertEqual(scheduler["allowed_orientations"], ["horizontal"])
        self.assertEqual(scheduler["min_clearance_horizontal_px"], 12.0)


if __name__ == "__main__":
    unittest.main()
