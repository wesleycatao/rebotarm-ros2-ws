"""Tests for the read-only ROS diagnostics dashboard."""

from __future__ import annotations

import csv
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

from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_gui.ros_bridge import RosBridge  # noqa: E402
from rebotarm_gui.tabs.diagnostics_tab import DiagnosticsTab  # noqa: E402


class _FakeBridge:
    @staticmethod
    def diagnostic_snapshot() -> dict[str, object]:
        return {
            "connected": False,
            "heartbeat": {"alive": False, "received": False},
            "topics": [],
            "services": [],
            "actions": [],
            "captured_at": 0.0,
        }


class TestDiagnosticsTab(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.tab = DiagnosticsTab(_FakeBridge())

    def tearDown(self) -> None:
        self.tab.deleteLater()
        self._app.processEvents()

    def test_snapshot_updates_heartbeat_topics_and_endpoints(self) -> None:
        snapshot = {
            "connected": True,
            "heartbeat": {
                "alive": True,
                "received": True,
                "hz": 19.95,
                "age_ms": 22.0,
                "latency_ms": 3.4,
            },
            "topics": [
                {
                    "name": "/rebotarm/arm_status",
                    "received": True,
                    "hz": 19.95,
                    "age_ms": 22.0,
                    "latency_ms": 3.4,
                },
                {
                    "name": "/camera/image_raw",
                    "received": False,
                    "hz": None,
                    "age_ms": None,
                    "latency_ms": None,
                },
            ],
            "services": [{"name": "/rebotarm/enable", "ready": True}],
            "actions": [{"name": "/rebotarm/move_to_pose", "ready": False}],
            "captured_at": 1.0,
        }

        self.tab.update_snapshot(snapshot)

        self.assertEqual(self.tab._heartbeat_value.text(), "OK · 19.9 Hz")
        self.assertEqual(self.tab._latency_value.text(), "3.4 ms")
        self.assertEqual(self.tab._topics_value.text(), "1 / 2")
        self.assertEqual(self.tab._endpoints_value.text(), "1 / 2")
        self.assertEqual(self.tab._topic_table.rowCount(), 2)
        self.assertEqual(self.tab._topic_table.item(0, 0).text(), "ATIVO")
        self.assertEqual(self.tab._endpoint_table.rowCount(), 2)
        self.assertEqual(self.tab._endpoint_table.item(1, 1).text(), "INDISPONÍVEL")

    def test_last_error_can_be_recorded_and_cleared(self) -> None:
        self.tab.record_error("Action move_to_pose", "goal abortado")
        self.assertIn("Action move_to_pose", self.tab._error_meta.text())
        self.assertEqual(self.tab._error_message.text(), "goal abortado")
        self.assertTrue(self.tab._btn_clear_error.isEnabled())

        self.tab.clear_error()
        self.assertEqual(self.tab._error_message.text(), "—")
        self.assertFalse(self.tab._btn_clear_error.isEnabled())

    def test_json_snapshot_export_preserves_metrics_and_last_error(self) -> None:
        snapshot = {
            "connected": True,
            "heartbeat": {
                "alive": True,
                "received": True,
                "hz": 20.0,
                "age_ms": 10.0,
                "latency_ms": 2.5,
                "timeout_ms": 1000.0,
            },
            "topics": [
                {
                    "name": "/rebotarm/arm_status",
                    "received": True,
                    "hz": 20.0,
                    "age_ms": 10.0,
                    "latency_ms": 2.5,
                }
            ],
            "services": [{"name": "/rebotarm/enable", "ready": True}],
            "actions": [],
            "captured_at": 123.0,
        }
        self.tab.update_snapshot(snapshot)
        self.tab.record_error("Bridge ROS", "telemetria inválida")
        payload = self.tab.build_export_payload()

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "snapshot.json"
            self.tab.write_snapshot(target, payload, export_format="json")
            exported = json.loads(target.read_text(encoding="utf-8"))

        self.assertEqual(exported["schema_version"], 1)
        self.assertEqual(exported["diagnostics"]["captured_at"], 123.0)
        self.assertEqual(
            exported["diagnostics"]["topics"][0]["hz"], 20.0
        )
        self.assertEqual(exported["last_error"]["source"], "Bridge ROS")
        self.assertEqual(
            exported["last_error"]["message"], "telemetria inválida"
        )

    def test_csv_snapshot_export_flattens_topics_and_endpoints(self) -> None:
        snapshot = {
            "connected": True,
            "heartbeat": {
                "alive": True,
                "received": True,
                "hz": 20.0,
                "age_ms": 10.0,
                "latency_ms": 2.5,
                "timeout_ms": 1000.0,
            },
            "topics": [
                {
                    "name": "/camera/image_raw",
                    "received": True,
                    "hz": 15.0,
                    "age_ms": 40.0,
                    "latency_ms": 4.0,
                }
            ],
            "services": [{"name": "/rebotarm/enable", "ready": True}],
            "actions": [{"name": "/rebotarm/move", "ready": False}],
            "captured_at": 123.0,
        }
        payload = self.tab.build_export_payload(snapshot)

        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "snapshot.csv"
            self.tab.write_snapshot(target, payload, export_format="csv")
            with target.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        topic = next(row for row in rows if row["name"] == "/camera/image_raw")
        service = next(row for row in rows if row["name"] == "/rebotarm/enable")
        action = next(row for row in rows if row["name"] == "/rebotarm/move")
        self.assertEqual(topic["hz"], "15.0")
        self.assertEqual(topic["state"], "ATIVO")
        self.assertEqual(service["state"], "DISPONÍVEL")
        self.assertEqual(action["state"], "INDISPONÍVEL")

    def test_snapshot_export_rejects_unknown_format(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "formato não suportado"):
                self.tab.write_snapshot(
                    Path(directory) / "snapshot.txt",
                    self.tab.build_export_payload(),
                    export_format="txt",
                )

    def test_disconnected_bridge_has_safe_empty_snapshot(self) -> None:
        snapshot = RosBridge().diagnostic_snapshot()
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["heartbeat"]["alive"])
        self.assertEqual(snapshot["topics"], [])


if __name__ == "__main__":
    unittest.main()
