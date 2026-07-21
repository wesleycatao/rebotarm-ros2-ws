"""Coverage for the persistent JSONL/CSV execution audit trail."""

from __future__ import annotations

import csv
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/rebotarm_gui_test_mpl")

from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from rebotarm_gui.audit import ExecutionAuditLog  # noqa: E402
from rebotarm_gui.motion import MoveStep, Program, pose  # noqa: E402
from rebotarm_gui.ros_bridge import RosBridge  # noqa: E402
from rebotarm_gui.settings import (  # noqa: E402
    AppSettings,
    load_app_settings,
    save_app_settings,
)
from rebotarm_gui.tabs.motion_tab import MotionTab  # noqa: E402


class TestExecutionAuditLog(unittest.TestCase):
    def test_operator_identifier_is_persisted_in_gui_settings(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "settings.json"
            settings = AppSettings(operator_id="TURNO-B-14")
            save_app_settings(settings, path)

            loaded = load_app_settings(path)

            self.assertEqual(loaded.operator_id, "TURNO-B-14")

    def test_jsonl_is_append_only_private_and_exports_structured_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = ExecutionAuditLog(Path(tmp) / "audit" / "execution.jsonl")
            self.assertTrue(
                log.append(
                    "step_end",
                    run_id="run-1",
                    operator="OP-07",
                    program="coleta",
                    parameters={"pose": {"x": 0.1}},
                    result="success",
                    duration_ms=12.5,
                )
            )
            self.assertEqual(log.path.stat().st_mode & 0o777, 0o600)
            record = next(iter(log.iter_records()))
            self.assertEqual(record["schema_version"], 1)
            self.assertEqual(record["operator"], "OP-07")
            self.assertEqual(record["parameters"]["pose"]["x"], 0.1)

            # A torn final line does not make earlier audit records unreadable.
            with log.path.open("a", encoding="utf-8") as stream:
                stream.write('{"event":"incomplete"')
            destination = Path(tmp) / "export.csv"
            self.assertEqual(log.export_csv(destination), 1)
            self.assertEqual(destination.stat().st_mode & 0o777, 0o600)
            with destination.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["run_id"], "run-1")
            self.assertEqual(json.loads(rows[0]["parameters"])["pose"]["x"], 0.1)
            with self.assertRaises(ValueError):
                log.export_csv(log.path)


class TestMotionAuditIntegration(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.log = ExecutionAuditLog(Path(self.tmp.name) / "execution.jsonl")
        self.bridge = RosBridge()
        self.tab = MotionTab(
            self.bridge,
            audit_log=self.log,
            operator_provider=lambda: "OPERADOR-TESTE",
        )
        self.tab.load_program(
            Program(
                name="auditado",
                steps=[MoveStep(label="A", pose=pose(0.1, 0.0, 0.2), duration=0.1)],
            )
        )
        self.tab._preflight_gate = lambda _program=None: True
        self.tab._check_execution_preconditions = lambda: (True, "")
        self.bridge.safety_set_step = lambda *_args, **_kwargs: True
        self.original_question = QMessageBox.question
        self.original_critical = QMessageBox.critical
        QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes

    def tearDown(self) -> None:
        QMessageBox.question = self.original_question
        QMessageBox.critical = self.original_critical
        self.tab.close()
        self.tab.deleteLater()
        self._app.processEvents()
        self.tmp.cleanup()

    def _records(self) -> list[dict]:
        return list(self.log.iter_records())

    def test_success_records_run_step_parameters_result_and_durations(self) -> None:
        self.tab._dry_run = True
        self.tab._chk_dry_run.setChecked(True)
        self.tab._run()
        deadline = time.monotonic() + 2.0
        while self.tab._running and time.monotonic() < deadline:
            self._app.processEvents()
            time.sleep(0.01)

        records = self._records()
        self.assertEqual(
            [record["event"] for record in records],
            ["run_start", "step_start", "step_end", "run_end"],
        )
        self.assertEqual(len({record["run_id"] for record in records}), 1)
        self.assertTrue(all(record["operator"] == "OPERADOR-TESTE" for record in records))
        self.assertEqual(records[1]["step_id"], "A_001")
        self.assertEqual(records[1]["parameters"]["pose"]["x"], 0.1)
        self.assertEqual(records[2]["result"], "success")
        self.assertGreaterEqual(records[2]["duration_ms"], 0.0)
        self.assertEqual(records[3]["result"], "success")
        self.assertIn("started_at_utc", records[3])
        self.assertIn("finished_at_utc", records[3])

    def test_operator_stop_records_cancel_and_canceled_end(self) -> None:
        self.tab._dry_run = True
        self.tab._chk_dry_run.setChecked(True)
        self.tab._program.steps[0].duration = 1.0
        self.tab._run()
        self.assertTrue(self.tab._running)
        self.tab._stop()

        records = self._records()
        self.assertIn("cancel_requested", [record["event"] for record in records])
        self.assertEqual(records[-2]["event"], "step_end")
        self.assertEqual(records[-2]["result"], "canceled")
        self.assertEqual(records[-1]["event"], "run_end")
        self.assertEqual(records[-1]["result"], "canceled")

    def test_action_failure_records_explicit_failure_event(self) -> None:
        sent_tags: list[str] = []
        self.bridge.move_to_pose_tagged = (
            lambda tag, _pose, _duration: sent_tags.append(tag) or True
        )
        self.tab._run()
        self.assertTrue(sent_tags)
        self.tab._on_pose_result(sent_tags[0], False, "goal rejeitado")

        records = self._records()
        failure = next(record for record in records if record["event"] == "failure")
        self.assertEqual(failure["step_id"], "A_001")
        self.assertIn("goal rejeitado", failure["message"])
        self.assertEqual(records[-1]["result"], "failure")

    def test_run_fails_closed_when_audit_start_cannot_be_written(self) -> None:
        blocker = Path(self.tmp.name) / "not-a-directory"
        blocker.write_text("blocked", encoding="utf-8")
        self.tab._audit_log = ExecutionAuditLog(blocker / "execution.jsonl")
        sent_tags: list[str] = []
        self.bridge.move_to_pose_tagged = (
            lambda tag, _pose, _duration: sent_tags.append(tag) or True
        )
        messages: list[str] = []
        QMessageBox.critical = (
            lambda _parent, _title, message, *args, **kwargs: messages.append(message)
        )

        self.tab._run()

        self.assertFalse(self.tab._running)
        self.assertEqual(sent_tags, [])
        self.assertTrue(messages)
        self.assertIn("registro de auditoria", messages[0])


if __name__ == "__main__":
    unittest.main()
