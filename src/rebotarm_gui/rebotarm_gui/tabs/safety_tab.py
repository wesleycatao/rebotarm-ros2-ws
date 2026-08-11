"""Safety tab for supervised torque profiles and collision alarms."""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import time
from typing import Callable, Optional

import yaml

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from rebotarm_gui.widgets.combo_box import RetractingComboBox
from rebotarmcontroller.torque_profile import build_profile_data, safe_step_id, write_csv

from ..widgets.layout_utils import fit_button_text, shrink_line_edit
from ..widgets.safety_event_dialog import SafetyEventDialog
from ..widgets.status_led import StatusLed
from ..widgets.step_overrides_editor import StepOverridesEditor
from ..widgets.torque_profile_plot import TorqueProfilePlot


def _collect_step_ids(paths: list[Path]) -> set[str]:
    """Read CSVs and collect the set of step IDs present."""
    step_ids: set[str] = set()
    for path in paths:
        try:
            with open(path, "r", encoding="utf-8", newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    step_id = row.get("step_id", "").strip()
                    if step_id:
                        step_ids.add(safe_step_id(step_id, "default"))
        except Exception:
            continue
    return step_ids


class SafetyTab(QWidget):
    """Configure and validate torque safety profiles.

    The page is split into two areas:
      * Operation — monitoring, current profile summary and fault recovery;
      * Configuration — profile creation, import/export/generation, and advanced
        parameters.  Sensitive configuration actions require at least Supervisor
        authentication and are audited.
    """

    alarm_changed = Signal(str, str)

    def __init__(
        self,
        bridge,
        parent: QWidget | None = None,
        *,
        authenticate_callback: Optional[Callable[[], bool]] = None,
        audit_callback: Optional[Callable[[dict], None]] = None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._authenticate_callback = authenticate_callback
        self._audit_callback = audit_callback
        self._config_authorized = False
        self._config_authorized_at = 0.0
        self._latest_joint_state = None
        self._recording = False
        self._record_rows: list[dict[str, object]] = []
        self._record_start_mono = 0.0
        self._record_sample = 0
        self._record_run_id = ""

        self._record_timer = QTimer(self)
        self._record_timer.setSingleShot(True)
        self._record_timer.timeout.connect(self._stop_recording)

        self._led = StatusLed("green", 18)
        self._status = QLabel("OK")
        self._state = QLabel("--")
        self._last_alarm = QLabel("--")
        self._last_alarm.setWordWrap(True)
        self._last_alarm.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._btn_event_details = QPushButton("Detalhes da última falha")
        self._btn_event_details.clicked.connect(self._show_last_safety_event)
        fit_button_text(self._btn_event_details)

        status_box = QGroupBox("Alarme de torque")
        status_grid = QGridLayout(status_box)
        status_grid.setColumnStretch(2, 1)
        status_grid.addWidget(QLabel("Status"), 0, 0)
        status_grid.addWidget(self._led, 0, 1)
        status_grid.addWidget(self._status, 0, 2)
        status_grid.addWidget(QLabel("Estado"), 1, 0)
        status_grid.addWidget(self._state, 1, 1, 1, 2)
        status_grid.addWidget(QLabel("Ultimo evento"), 2, 0)
        status_grid.addWidget(self._last_alarm, 2, 1, 1, 2)
        status_grid.addWidget(self._btn_event_details, 3, 1, 1, 2)

        self._profile_path = QLineEdit()
        self._profile_path.setMinimumWidth(240)
        self._profile_path.setMaximumWidth(520)
        shrink_line_edit(self._profile_path)
        self._btn_import = QPushButton("Importar YAML")
        self._btn_apply = QPushButton("Aplicar")
        self._btn_reset_profile = QPushButton("Resetar perfil")
        self._btn_reset_fault = QPushButton("Resetar falha")
        for b in (self._btn_import, self._btn_apply, self._btn_reset_profile, self._btn_reset_fault):
            fit_button_text(b)
        self._mode = RetractingComboBox()
        self._mode.addItems(["off", "warn_only", "stop_on_fail"])
        self._active_step = QLineEdit("default")
        self._violation_samples = QSpinBox()
        self._violation_samples.setRange(1, 50)
        self._violation_samples.setValue(3)
        self._derivative_limit = self._dspin(0.0, 10000.0, 0.0, " Nm/s", 10.0)
        self._derivative_limit.setToolTip(
            "Limite de taxa de variação do torque. 0 desativa. "
            "Picos súbitos acima deste valor geram WARN adicional."
        )
        self._btn_import.clicked.connect(self._import_profile)
        self._btn_apply.clicked.connect(self._apply_profile)
        self._btn_reset_profile.clicked.connect(self._reset_profile)
        self._btn_reset_fault.clicked.connect(self._reset_fault)
        self._profile_edit_controls = (
            self._profile_path,
            self._btn_import,
            self._btn_apply,
            self._btn_reset_profile,
            self._mode,
            self._active_step,
            self._violation_samples,
            self._derivative_limit,
        )

        profile_box = QGroupBox("Perfil ativo")
        profile_form = QFormLayout(profile_box)
        profile_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        row = QHBoxLayout()
        row.addWidget(self._profile_path, 1)
        row.addWidget(self._btn_import)
        profile_form.addRow("Arquivo", row)
        profile_form.addRow("Modo", self._mode)
        profile_form.addRow("Etapa ativa", self._active_step)
        profile_form.addRow("Amostras FAIL", self._violation_samples)
        profile_form.addRow("Limite derivada", self._derivative_limit)
        actions = QHBoxLayout()
        actions.addWidget(self._btn_apply)
        actions.addWidget(self._btn_reset_profile)
        actions.addWidget(self._btn_reset_fault)
        profile_form.addRow(actions)

        self._program = QLineEdit("programa")
        shrink_line_edit(self._program, max_width=220)
        self._record_step = QLineEdit("default")
        shrink_line_edit(self._record_step, max_width=180)
        self._record_label = QLineEdit("")
        shrink_line_edit(self._record_label, max_width=220)
        self._payload_name = QLineEdit("")
        shrink_line_edit(self._payload_name, max_width=220)
        self._payload_kg = self._dspin(0.0, 100.0, 0.0, " kg", 0.05)
        self._record_duration = self._dspin(0.1, 3600.0, 10.0, " s", 0.5)
        self._record_output = QLineEdit(str(Path.home() / "torque_runs" / "run.csv"))
        self._record_output.setMinimumWidth(240)
        self._record_output.setMaximumWidth(520)
        shrink_line_edit(self._record_output)
        self._btn_record_path = QPushButton("...")
        self._btn_start_record = QPushButton("Gravar perfil")
        self._btn_stop_record = QPushButton("Parar")
        fit_button_text(self._btn_start_record)
        fit_button_text(self._btn_stop_record)
        self._sample_count = QLabel("0 amostras")
        self._btn_record_path.clicked.connect(self._choose_record_output)
        self._btn_start_record.clicked.connect(self._start_recording)
        self._btn_stop_record.clicked.connect(self._stop_recording)
        self._btn_stop_record.setEnabled(False)

        record_box = QGroupBox("Gravacao supervisionada por etapa")
        record_form = QFormLayout(record_box)
        record_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        record_form.addRow("Programa", self._program)
        record_form.addRow("Step ID", self._record_step)
        record_form.addRow("Rotulo", self._record_label)
        record_form.addRow("Payload", self._payload_name)
        record_form.addRow("Massa", self._payload_kg)
        record_form.addRow("Duracao", self._record_duration)
        out_row = QHBoxLayout()
        out_row.addWidget(self._record_output, 1)
        out_row.addWidget(self._btn_record_path)
        record_form.addRow("CSV", out_row)
        rec_actions = QHBoxLayout()
        rec_actions.addWidget(self._btn_start_record)
        rec_actions.addWidget(self._btn_stop_record)
        rec_actions.addWidget(self._sample_count)
        record_form.addRow(rec_actions)

        self._csv_inputs = QLineEdit()
        self._csv_inputs.setMinimumWidth(240)
        self._csv_inputs.setMaximumWidth(520)
        shrink_line_edit(self._csv_inputs)
        self._profile_output = QLineEdit(
            str(Path.home() / "torque_profiles" / "profile.yaml")
        )
        self._profile_output.setMinimumWidth(240)
        self._profile_output.setMaximumWidth(520)
        shrink_line_edit(self._profile_output)
        self._warn_pct = self._dspin(0.0, 200.0, 10.0, " %", 1.0)
        self._fail_pct = self._dspin(0.0, 300.0, 20.0, " %", 1.0)
        self._min_margin = self._dspin(0.0, 1000.0, 0.25, "", 0.05)
        self._bins = QSpinBox()
        self._bins.setRange(2, 1001)
        self._bins.setValue(101)
        self._std_factor = self._dspin(0.0, 20.0, 3.0, "", 0.5)
        self._btn_pick_csv = QPushButton("Selecionar CSVs")
        self._btn_profile_output = QPushButton("...")
        self._btn_build = QPushButton("Gerar YAML")
        for b in (self._btn_pick_csv, self._btn_build):
            fit_button_text(b)
        self._btn_pick_csv.clicked.connect(self._pick_csvs)
        self._btn_profile_output.clicked.connect(self._choose_profile_output)
        self._btn_build.clicked.connect(self._build_profile)

        self._step_overrides = StepOverridesEditor()

        build_box = QGroupBox("Gerar perfil WARN/FAIL")
        build_form = QFormLayout(build_box)
        build_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        csv_row = QHBoxLayout()
        csv_row.addWidget(self._csv_inputs, 1)
        csv_row.addWidget(self._btn_pick_csv)
        build_form.addRow("CSVs", csv_row)
        profile_row = QHBoxLayout()
        profile_row.addWidget(self._profile_output, 1)
        profile_row.addWidget(self._btn_profile_output)
        build_form.addRow("YAML", profile_row)
        build_form.addRow("WARN", self._warn_pct)
        build_form.addRow("FAIL", self._fail_pct)
        build_form.addRow("Margem min.", self._min_margin)
        build_form.addRow("Bins", self._bins)
        build_form.addRow("Fator de desvio", self._std_factor)
        build_form.addRow(self._btn_build)

        overrides_box = QGroupBox("Tolerâncias por etapa")
        overrides_layout = QVBoxLayout(overrides_box)
        overrides_layout.setContentsMargins(8, 8, 8, 8)
        overrides_layout.addWidget(self._step_overrides, 1)

        self._check_phase = self._dspin(0.0, 1.0, 0.0, "", 0.05)
        self._check_phase.setAccessibleName("Fase para verificação de torque")
        self._btn_check = QPushButton("Verificar agora")
        fit_button_text(self._btn_check)
        self._check_summary = QLabel("--")
        self._check_summary.setWordWrap(True)
        self._check_table = QTableWidget(0, 7)
        self._check_table.setHorizontalHeaderLabels(
            ["Junta", "Torque", "WARN min", "WARN max", "FAIL min", "FAIL max", "Status"]
        )
        self._check_table.setAccessibleName("Resultado da verificação de torque por junta")
        self._check_table.setMinimumHeight(280)
        self._check_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._check_table.horizontalHeader().setStretchLastSection(True)
        self._btn_check.clicked.connect(self._check_now)
        check_box = QGroupBox("Verificacao")
        check_layout = QVBoxLayout(check_box)
        check_top = QHBoxLayout()
        check_top.addWidget(QLabel("Fase"))
        check_top.addWidget(self._check_phase)
        check_top.addWidget(self._btn_check)
        check_top.addWidget(self._check_summary, 1)
        check_layout.addLayout(check_top)
        check_layout.addWidget(self._check_table, 1)

        self._profile_plot = TorqueProfilePlot()
        plot_box = QGroupBox("Visualizacao do perfil")
        plot_layout = QVBoxLayout(plot_box)
        plot_layout.setContentsMargins(8, 8, 8, 8)
        plot_layout.addWidget(self._profile_plot, 1)

        operation = QWidget()
        operation_layout = QVBoxLayout(operation)
        operation_layout.setContentsMargins(0, 0, 0, 0)
        operation_layout.setSpacing(14)
        operation_layout.addWidget(status_box)
        operation_layout.addWidget(profile_box)
        operation_layout.addWidget(check_box, 1)
        operation_layout.addWidget(plot_box, 2)

        recording = QWidget()
        recording_layout = QVBoxLayout(recording)
        recording_layout.setContentsMargins(0, 0, 0, 0)
        recording_layout.setSpacing(14)
        recording_layout.addWidget(record_box)
        recording_layout.addStretch(1)

        build = QWidget()
        build_layout = QVBoxLayout(build)
        build_layout.setContentsMargins(0, 0, 0, 0)
        build_layout.setSpacing(14)
        build_layout.addWidget(build_box)
        build_layout.addWidget(overrides_box, 1)
        build_layout.addStretch(1)

        configuration = QWidget()
        configuration_layout = QVBoxLayout(configuration)
        configuration_layout.setContentsMargins(0, 0, 0, 0)
        configuration_layout.setSpacing(14)
        configuration_layout.addWidget(record_box)
        configuration_layout.addWidget(build_box)
        configuration_layout.addWidget(overrides_box, 1)
        configuration_layout.addStretch(1)

        self._sections = QTabWidget()
        self._sections.addTab(self._wrap_scroll(operation), "Operação")
        self._sections.addTab(self._wrap_scroll(configuration), "Configuração")

        self._access_note = QLabel("")
        self._access_note.setObjectName("ModeNotice")
        self._access_note.setWordWrap(True)
        self._access_note.setAccessibleName("Permissões do modo da interface")

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.addWidget(self._access_note)
        root.addWidget(self._sections, 1)

    def set_interface_mode(self, mode: str) -> None:
        """Adapt editing controls to the selected interface mode.

        This is a usability guard, not authentication. Controller-side safety
        validation remains the source of truth.
        """
        can_edit_profile = mode in {"supervisor", "engenharia"}
        for control in self._profile_edit_controls:
            control.setEnabled(can_edit_profile)
        self._sections.setTabEnabled(1, can_edit_profile)
        if not self._sections.isTabEnabled(self._sections.currentIndex()):
            self._sections.setCurrentIndex(0)

        if mode == "operador":
            text = (
                "Modo Operador: monitoramento e reset de falha disponíveis; "
                "alterações do perfil de torque estão bloqueadas."
            )
        elif mode == "supervisor":
            text = (
                "Modo Supervisor: o perfil ativo pode ser revisado e aplicado; "
                "gravação e geração de perfis exigem o modo Engenharia."
            )
        else:
            text = (
                "Modo Engenharia: edição, gravação e geração de perfis habilitadas. "
                "O modo da interface não substitui autenticação nem validação do controlador."
            )
        self._access_note.setText(text)

    # ------------------------------------------------------------------
    # Authentication and audit helpers
    # ------------------------------------------------------------------
    def _require_supervisor_or_engineer(self) -> bool:
        """Re-authenticate before any privileged safety mutation."""
        if self._authenticate_callback is not None:
            if not self._authenticate_callback():
                self._audit(
                    action="privilege_check",
                    success=False,
                    reason="autenticação privilegiada negada ou cancelada",
                )
                return False
        self._config_authorized = True
        self._config_authorized_at = time.monotonic()
        return True

    def _ensure_authorization_still_valid(self) -> bool:
        """Authorization is session-scoped; force re-check for risky actions."""
        if self._authenticate_callback is None:
            return True
        # In a real deployment the lease timeout is checked here; for the GUI
        # we re-prompt on every privileged action so a lost lease cannot be
        # exploited.
        return self._require_supervisor_or_engineer()

    def _audit(
        self,
        *,
        action: str,
        profile_before: str = "",
        profile_after: str = "",
        success: bool,
        reason: str = "",
    ) -> None:
        if self._audit_callback is None:
            return
        record = {
            "event": "safety_profile_change",
            "action": action,
            "profile_before": profile_before,
            "profile_after": profile_after,
            "success": success,
            "reason": reason,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
        }
        try:
            self._audit_callback(record)
        except Exception:
            pass

    # ------------------------------------------------------------------
    def update_joint_state(self, msg) -> None:
        self._latest_joint_state = msg
        if self._recording:
            self._append_record_sample(msg)
        # Update live torque marker on the profile plot if a joint is selected.
        plot_joint = self._profile_plot._current_joint
        if plot_joint and self._profile_plot._profile_data is not None:
            names = list(getattr(msg, "name", []))
            efforts = list(getattr(msg, "effort", []))
            if plot_joint in names:
                idx = names.index(plot_joint)
                if idx < len(efforts):
                    # Phase is unknown on the GUI side; pass None to keep marker off.
                    self._profile_plot.update_current_state(
                        plot_joint, float(efforts[idx]), -1.0
                    )

    def update_arm_status(self, status) -> None:
        state = str(getattr(status, "state_machine", "--"))
        errors = list(getattr(status, "error_codes", []))
        self._state.setText(state)
        if state == "PROTECTIVE_STOP":
            self._led.set_color("red")
            self._status.setText("FAIL")
        elif any("TORQUE" in str(err) for err in errors):
            self._led.set_color("amber")
            self._status.setText("WARN")
        else:
            self._led.set_color("green")
            self._status.setText("OK")
        if errors:
            message = ", ".join(str(err) for err in errors)
            self._last_alarm.setText(message)
            severity = "FAIL" if state == "PROTECTIVE_STOP" else "WARN"
            self.alarm_changed.emit(severity, message)

    def update_check_result(self, result) -> None:
        ok = bool(getattr(result, "success", False))
        severity = str(getattr(result, "severity", "FAIL" if not ok else "OK"))
        message = str(getattr(result, "message", ""))
        self._check_summary.setText(f"{severity}: {message}")
        self._last_alarm.setText(message or severity)
        self._led.set_color("red" if severity == "FAIL" else "amber" if severity == "WARN" else "green")
        self._status.setText(severity)
        if severity in {"WARN", "FAIL"}:
            self.alarm_changed.emit(severity, message or "Verificação de torque fora do perfil.")

        names = list(getattr(result, "joint_names", []))
        torque = list(getattr(result, "torque", []))
        warn_min = list(getattr(result, "warn_min", []))
        warn_max = list(getattr(result, "warn_max", []))
        fail_min = list(getattr(result, "fail_min", []))
        fail_max = list(getattr(result, "fail_max", []))
        status = list(getattr(result, "status", []))
        self._check_table.setRowCount(len(names))
        for row, name in enumerate(names):
            values = [
                name,
                f"{torque[row]:+.4f}",
                f"{warn_min[row]:+.4f}",
                f"{warn_max[row]:+.4f}",
                f"{fail_min[row]:+.4f}",
                f"{fail_max[row]:+.4f}",
                status[row],
            ]
            for col, text in enumerate(values):
                self._check_table.setItem(row, col, QTableWidgetItem(str(text)))

    @staticmethod
    def _dspin(minimum, maximum, value, suffix="", step=0.1) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(float(minimum), float(maximum))
        spin.setValue(float(value))
        spin.setSingleStep(float(step))
        spin.setDecimals(3)
        if suffix:
            spin.setSuffix(suffix)
        return spin

    @staticmethod
    def _wrap_scroll(content: QWidget) -> QScrollArea:
        content.setMinimumWidth(680)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(content)
        return scroll

    def _import_profile(self) -> None:
        if not self._require_supervisor_or_engineer():
            QMessageBox.warning(
                self, "Permissão necessária",
                "Importar perfil exige autenticação de Supervisor ou Engenharia."
            )
            return
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Importar perfil de torque",
            str(Path.home()),
            "YAML (*.yaml *.yml)",
        )
        if path:
            old = self._profile_path.text()
            self._profile_path.setText(path)
            self._profile_plot.load_profile(path)
            self._audit(
                action="import_profile",
                profile_before=old,
                profile_after=path,
                success=True,
            )

    def _apply_profile(self) -> None:
        if not self._ensure_authorization_still_valid():
            QMessageBox.warning(
                self, "Permissão necessária",
                "Aplicar perfil exige autenticação de Supervisor ou Engenharia."
            )
            return
        old = self._profile_path.text()
        new_mode = self._mode.currentText()
        new_violations = int(self._violation_samples.value())
        new_step = self._active_step.text().strip()
        new_derivative = float(self._derivative_limit.value())
        try:
            self._bridge.safety_set_profile(
                self._profile_path.text().strip(),
                new_mode,
                new_violations,
                new_step,
                new_derivative,
            )
            self._audit(
                action="apply_profile",
                profile_before=old,
                profile_after=self._profile_path.text().strip(),
                success=True,
            )
        except Exception as exc:
            self._audit(
                action="apply_profile",
                profile_before=old,
                profile_after=self._profile_path.text().strip(),
                success=False,
                reason=str(exc),
            )
            raise

    def _reset_profile(self) -> None:
        if not self._ensure_authorization_still_valid():
            QMessageBox.warning(
                self, "Permissão necessária",
                "Resetar perfil exige autenticação de Supervisor ou Engenharia."
            )
            return
        old = self._profile_path.text()
        self._profile_path.clear()
        self._mode.setCurrentText("off")
        self._profile_plot.clear_profile()
        try:
            self._bridge.safety_set_profile(
                "", "off", int(self._violation_samples.value()), "", 0.0
            )
            self._audit(
                action="reset_profile",
                profile_before=old,
                profile_after="",
                success=True,
            )
        except Exception as exc:
            self._audit(
                action="reset_profile",
                profile_before=old,
                profile_after="",
                success=False,
                reason=str(exc),
            )
            raise

    def _reset_fault(self) -> None:
        answer = QMessageBox.question(
            self,
            "Confirmar reset de segurança",
            "Confirme que a causa da colisão foi removida, a área está livre "
            "e o braço está parado. O controlador ainda verificará torque, "
            "velocidade e falhas dos atuadores antes de liberar o movimento.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self._bridge.safety_reset_fault()
            self._audit(action="reset_fault", success=True)

    def _choose_record_output(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Salvar CSV de torque",
            self._record_output.text(),
            "CSV (*.csv)",
        )
        if path:
            self._record_output.setText(path)

    def _start_recording(self) -> None:
        if not self._require_supervisor_or_engineer():
            QMessageBox.warning(
                self, "Permissão necessária",
                "Gravação supervisionada de torque exige autenticação de Supervisor ou Engenharia."
            )
            return
        if self._latest_joint_state is None:
            QMessageBox.warning(self, "Sem telemetria", "Aguardando /joint_states.")
            self._audit(
                action="start_recording",
                success=False,
                reason="sem telemetria disponível",
            )
            return
        self._record_rows = []
        self._record_sample = 0
        self._record_run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._record_start_mono = time.monotonic()
        self._recording = True
        self._btn_start_record.setEnabled(False)
        self._btn_stop_record.setEnabled(True)
        self._sample_count.setText("0 amostras")
        self._record_timer.start(int(float(self._record_duration.value()) * 1000))
        self._audit(action="start_recording", success=True)

    def _stop_recording(self) -> None:
        if not self._recording:
            return
        self._recording = False
        self._record_timer.stop()
        self._btn_start_record.setEnabled(True)
        self._btn_stop_record.setEnabled(False)
        try:
            output = Path(self._record_output.text().strip())
            output.parent.mkdir(parents=True, exist_ok=True)
            tmp = output.with_suffix(output.suffix + ".tmp")
            write_csv(str(tmp), self._record_rows)
            tmp.replace(output)
            output.chmod(0o600)
            self._audit(
                action="stop_recording",
                success=True,
                reason=f"{len(self._record_rows)} linhas salvas em {output}",
            )
        except Exception as exc:
            self._audit(action="stop_recording", success=False, reason=str(exc))
            QMessageBox.critical(self, "Erro ao salvar CSV", str(exc))
            return
        self._sample_count.setText(f"{len(self._record_rows)} linhas salvas")

    def _append_record_sample(self, msg) -> None:
        elapsed = time.monotonic() - self._record_start_mono
        stamp = getattr(msg, "header", None).stamp if getattr(msg, "header", None) else None
        stamp_sec = (
            float(stamp.sec) + float(stamp.nanosec) * 1e-9 if stamp is not None else 0.0
        )
        step_id = safe_step_id(self._record_step.text(), "default")
        for index, name in enumerate(msg.name):
            if str(name).startswith("gripper"):
                continue
            self._record_rows.append(
                {
                    "run_id": self._record_run_id,
                    "program": self._program.text().strip(),
                    "step_id": step_id,
                    "step_label": self._record_label.text().strip(),
                    "payload_name": self._payload_name.text().strip(),
                    "payload_kg": f"{float(self._payload_kg.value()):.6g}",
                    "sample_index": self._record_sample,
                    "stamp_sec": f"{stamp_sec:.9f}",
                    "elapsed_sec": f"{elapsed:.9f}",
                    "joint_name": str(name),
                    "position": f"{float(msg.position[index] if index < len(msg.position) else 0.0):.9f}",
                    "velocity": f"{float(msg.velocity[index] if index < len(msg.velocity) else 0.0):.9f}",
                    "torque": f"{float(msg.effort[index] if index < len(msg.effort) else 0.0):.9f}",
                }
            )
        self._record_sample += 1
        self._sample_count.setText(f"{len(self._record_rows)} linhas")

    def _pick_csvs(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Selecionar CSVs",
            str(Path.home()),
            "CSV (*.csv)",
        )
        if paths:
            self._csv_inputs.setText(";".join(paths))
            self._refresh_step_overrides()

    def _refresh_step_overrides(self) -> None:
        paths = [Path(p) for p in self._csv_inputs.text().split(";") if p.strip()]
        step_ids = _collect_step_ids(paths)
        defaults = {
            "warn_tolerance_percent": float(self._warn_pct.value()),
            "fail_tolerance_percent": float(self._fail_pct.value()),
            "min_margin": float(self._min_margin.value()),
        }
        self._step_overrides.set_steps(list(step_ids), defaults)

    def _choose_profile_output(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Salvar perfil YAML",
            self._profile_output.text(),
            "YAML (*.yaml *.yml)",
        )
        if path:
            self._profile_output.setText(path)

    def _build_profile(self) -> None:
        if not self._ensure_authorization_still_valid():
            QMessageBox.warning(
                self, "Permissão necessária",
                "Gerar perfil de torque exige autenticação de Supervisor ou Engenharia."
            )
            return
        paths = [Path(p) for p in self._csv_inputs.text().split(";") if p.strip()]
        if not paths:
            QMessageBox.warning(self, "Sem CSV", "Selecione ao menos um CSV.")
            return
        try:
            data = build_profile_data(
                paths,
                bins=int(self._bins.value()),
                warn_tolerance_percent=float(self._warn_pct.value()),
                fail_tolerance_percent=float(self._fail_pct.value()),
                min_margin=float(self._min_margin.value()),
                std_factor=float(self._std_factor.value()),
                violation_samples=int(self._violation_samples.value()),
                step_overrides=self._step_overrides.overrides(),
            )
            output = Path(self._profile_output.text().strip()).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            tmp = output.with_suffix(output.suffix + ".tmp")
            with open(tmp, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, sort_keys=False)
            tmp.replace(output)
            output.chmod(0o600)
            self._audit(
                action="build_profile",
                profile_after=str(output),
                success=True,
            )
        except Exception as exc:
            self._audit(
                action="build_profile",
                success=False,
                reason=str(exc),
            )
            QMessageBox.critical(self, "Erro ao gerar perfil", str(exc))
            return
        self._profile_path.setText(str(output))
        self._profile_plot.load_profile(str(output))
        QMessageBox.information(self, "Perfil gerado", str(output))

    def _check_now(self) -> None:
        self._bridge.safety_check(
            self._active_step.text().strip(),
            float(self._check_phase.value()),
        )

    def _show_last_safety_event(self) -> None:
        dialog = SafetyEventDialog(parent=self)
        dialog.exec()
