"""In-app, anonymised task timer for moderated operator usability sessions.

This module provides both the interactive panel and the data model for a
human usability study.  It does not fabricate participants or results: the
aggregation and synthetic-demo mode are explicitly separated so a real session
can never be silently mixed with generated data.
"""

from __future__ import annotations

import json
import os
import platform
import re
import statistics
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

from PySide6.QtCore import QElapsedTimer, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ..accessibility import announce


@dataclass(frozen=True)
class OperatorTask:
    title: str
    instruction: str
    success_criterion: str


@dataclass
class OperatorTaskResult:
    task: str
    success: bool
    elapsed_s: float
    errors: int
    assistance: bool
    cancelled: bool
    false_command: bool
    notes: str


@dataclass
class UsabilityEnvironment:
    """Non-identifiable session context recorded with every result."""

    gui_version: str = "unknown"
    qt_version: str = "unknown"
    pyside_version: str = "unknown"
    os_name: str = "unknown"
    resolution: str = "unknown"
    scale: str = "unknown"
    theme: str = "unknown"
    input_mode: str = "teclado+mouse"

    def to_dict(self) -> Dict[str, str]:
        return asdict(self)


@dataclass
class UsabilitySession:
    """Complete, anonymised record of one participant session."""

    participant_code: str
    experience: str
    tasks: List[OperatorTaskResult] = field(default_factory=list)
    environment: UsabilityEnvironment = field(default_factory=UsabilityEnvironment)
    synthetic: bool = False
    started_at: str = ""
    finished_at: str = ""
    facilitator_notes: str = ""

    def schema_version(self) -> int:
        return 2

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version(),
            "data_class": "DADOS SINTÉTICOS" if self.synthetic else "DADOS REAIS",
            "participant_code": self.participant_code,
            "experience": self.experience,
            "environment": self.environment.to_dict(),
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "completed": len(self.tasks) == len(DEFAULT_TASKS),
            "results": [asdict(r) for r in self.tasks],
            "facilitator_notes": self.facilitator_notes,
        }


@dataclass
class AggregateMetrics:
    """Aggregated, non-personal metrics across sessions."""

    sessions: int = 0
    real_sessions: int = 0
    synthetic_sessions: int = 0
    task_completion_rates: Dict[str, float] = field(default_factory=dict)
    task_median_seconds: Dict[str, float] = field(default_factory=dict)
    task_min_seconds: Dict[str, float] = field(default_factory=dict)
    task_max_seconds: Dict[str, float] = field(default_factory=dict)
    task_total_errors: Dict[str, int] = field(default_factory=dict)
    task_total_assistance: Dict[str, int] = field(default_factory=dict)
    task_total_false_commands: Dict[str, int] = field(default_factory=dict)
    task_total_incidents: Dict[str, int] = field(default_factory=dict)
    notes: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class UsabilityAggregator:
    """Aggregate anonymised session data without mixing real and synthetic."""

    @staticmethod
    def aggregate(sessions: List[UsabilitySession]) -> AggregateMetrics:
        real = [s for s in sessions if not s.synthetic]
        synthetic = [s for s in sessions if s.synthetic]
        metrics = AggregateMetrics(
            sessions=len(sessions),
            real_sessions=len(real),
            synthetic_sessions=len(synthetic),
            notes=(
                "Agregação separa dados reais de sintéticos. "
                "Não declare validação humana sem sessões reais."
            ),
        )
        # Aggregate real sessions only; synthetic sessions are reported separately.
        UsabilityAggregator._aggregate_group(real, metrics)
        return metrics

    @staticmethod
    def _aggregate_group(sessions: List[UsabilitySession], metrics: AggregateMetrics) -> None:
        by_task: Dict[str, List[OperatorTaskResult]] = {}
        for session in sessions:
            for result in session.tasks:
                by_task.setdefault(result.task, []).append(result)
        for task, results in by_task.items():
            times = [r.elapsed_s for r in results]
            metrics.task_completion_rates[task] = round(
                sum(1 for r in results if r.success) / len(results), 2
            )
            metrics.task_median_seconds[task] = round(statistics.median(times), 3)
            metrics.task_min_seconds[task] = round(min(times), 3)
            metrics.task_max_seconds[task] = round(max(times), 3)
            metrics.task_total_errors[task] = sum(r.errors for r in results)
            metrics.task_total_assistance[task] = sum(1 for r in results if r.assistance)
            metrics.task_total_false_commands[task] = sum(
                1 for r in results if r.false_command
            )
            metrics.task_total_incidents[task] = sum(
                1 for r in results if r.cancelled or r.false_command
            )


DEFAULT_TASKS = (
    OperatorTask(
        "Identificar o estado da célula",
        "Partindo de Início, informe se ROS e braço estão disponíveis e abra o diagnóstico ROS.",
        "Abre Diagnóstico ROS e interpreta heartbeat e endpoints corretamente.",
    ),
    OperatorTask(
        "Processar visão",
        "Abra Visão, processe um frame e identifique quantos objetos válidos foram encontrados.",
        "Executa o processamento e encontra a contagem sem ajuda.",
    ),
    OperatorTask(
        "Gerar uma pega",
        "A partir do objeto detectado, revise a melhor estratégia e envie-a ao Programa.",
        "Chega ao editor com um programa de coleta gerado.",
    ),
    OperatorTask(
        "Simular o programa",
        "Ative Simulação, valide e execute o programa sem comandar movimento real.",
        "Conclui o dry-run e localiza o resultado da execução.",
    ),
    OperatorTask(
        "Solicitar Home seguro",
        "Localize Home seguro, leia a confirmação e cancele antes de enviar o movimento.",
        "Encontra o comando, compreende o aviso e escolhe Cancelar.",
    ),
    OperatorTask(
        "Localizar e interpretar uma falha",
        "Após o facilitador injetar uma falha, localize origem, horário e ação recomendada.",
        "Usa o banner/centro de alarmes e relata a ação recomendada correta.",
    ),
)


_EXPERIENCE_OPTIONS = (
    "Primeiro uso (não identificável)",
    "Ocasional (não identificável)",
    "Experiente (não identificável)",
)


# Simple sanitiser: remove e-mail addresses and phone-like numbers.
_PERSONAL_DATA_PATTERNS = (
    re.compile(r"[\w.-]+@[\w.-]+\.\w+"),
    re.compile(r"\b\d{3}[\s.-]?\d{3}[\s.-]?\d{4}\b"),
)


def _sanitize_notes(text: str) -> str:
    for pattern in _PERSONAL_DATA_PATTERNS:
        text = pattern.sub("[REMOVIDO]", text)
    return text.strip()


def _environment() -> UsabilityEnvironment:
    try:
        import PySide6
        pyside_version = PySide6.__version__
    except Exception:
        pyside_version = "unknown"
    try:
        from PySide6.QtCore import qVersion
        qt_version = qVersion()
    except Exception:
        qt_version = "unknown"
    return UsabilityEnvironment(
        gui_version="2.0",
        qt_version=qt_version,
        pyside_version=pyside_version,
        os_name=platform.platform(),
        resolution=os.environ.get("QT_SCREEN_SIZE", "unknown"),
        scale=os.environ.get("QT_SCALE_FACTOR", "1.0"),
        theme=os.environ.get("REBOTARM_THEME", "dark"),
        input_mode="teclado+mouse",
    )


class OperatorTestPanel(QWidget):
    """Guided stopwatch that records one result per predefined task.

    The panel supports start, pause, resume, finish-success, finish-failure and
    cancel for each task.  It stores only an anonymous participant code, an
    experience category and objective metrics; no name, e-mail or image is
    recorded.  Results can be exported as JSON and aggregated across sessions.
    """

    session_completed = Signal(dict)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAccessibleName("Avaliação de usabilidade com operador")
        self._tasks = list(DEFAULT_TASKS)
        self._results: List[OperatorTaskResult] = []
        self._task_index = 0
        self._running = False
        self._paused = False
        self._elapsed = QElapsedTimer()
        self._session = UsabilitySession(
            participant_code="",
            experience=_EXPERIENCE_OPTIONS[0],
            environment=_environment(),
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        intro = QLabel(
            "Sessão moderada e anonimizada. Use o mock ou uma célula já validada; "
            "o facilitador não deve orientar durante a tarefa. "
            "Não registre nome, e-mail, voz, imagem ou outro dado pessoal."
        )
        intro.setWordWrap(True)
        intro.setObjectName("WarningText")
        root.addWidget(intro)

        form = QFormLayout()
        self.participant = QLineEdit()
        self.participant.setPlaceholderText("Código anônimo, ex.: OP-01")
        self.participant.setMaxLength(16)
        self.experience = QComboBox()
        self.experience.addItems(_EXPERIENCE_OPTIONS)
        self.input_mode = QComboBox()
        self.input_mode.addItems(["teclado+mouse", "somente teclado", "somente mouse"])
        self.input_mode.setAccessibleName("Modo de entrada")
        form.addRow("Participante", self.participant)
        form.addRow("Experiência", self.experience)
        form.addRow("Modo de entrada", self.input_mode)
        root.addLayout(form)

        self.progress = QLabel()
        self.progress.setObjectName("MetricValue")
        self.task_title = QLabel()
        self.task_title.setObjectName("PageTitle")
        self.task_instruction = QLabel()
        self.task_instruction.setWordWrap(True)
        self.success_criterion = QLabel()
        self.success_criterion.setWordWrap(True)
        self.success_criterion.setObjectName("MutedLabel")
        root.addWidget(self.progress)
        root.addWidget(self.task_title)
        root.addWidget(self.task_instruction)
        root.addWidget(self.success_criterion)

        self.timer_label = QLabel("00:00.0")
        self.timer_label.setObjectName("StatusValue")
        self.timer_label.setAccessibleName("Cronômetro da tarefa")
        root.addWidget(self.timer_label)

        metrics = QFormLayout()
        self.errors = QSpinBox()
        self.errors.setRange(0, 99)
        self.assistance = QCheckBox("Precisou de ajuda do facilitador")
        self.false_command = QCheckBox("Acionamento indevido de comando crítico")
        self.cancelled = QCheckBox("Tarefa cancelada pelo participante")
        self.notes = QTextEdit()
        self.notes.setPlaceholderText("Observações objetivas; não registre dados pessoais.")
        self.notes.setMaximumHeight(90)
        metrics.addRow("Erros observados", self.errors)
        metrics.addRow("", self.assistance)
        metrics.addRow("", self.false_command)
        metrics.addRow("", self.cancelled)
        metrics.addRow("Notas", self.notes)
        root.addLayout(metrics)

        buttons = QHBoxLayout()
        self.btn_start = QPushButton("Iniciar tarefa")
        self.btn_start.setObjectName("PrimaryButton")
        self.btn_pause = QPushButton("Pausar")
        self.btn_resume = QPushButton("Retomar")
        self.btn_success = QPushButton("Concluir com sucesso")
        self.btn_fail = QPushButton("Encerrar com falha")
        self.btn_fail.setObjectName("WarningButton")
        self.btn_cancel = QPushButton("Cancelar tarefa")
        self.btn_cancel.setObjectName("DangerButton")
        for b in (self.btn_start, self.btn_pause, self.btn_resume,
                  self.btn_success, self.btn_fail, self.btn_cancel):
            buttons.addWidget(b)
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(False)
        self.btn_success.setEnabled(False)
        self.btn_fail.setEnabled(False)
        self.btn_cancel.setEnabled(False)
        root.addLayout(buttons)

        self.btn_save = QPushButton("Exportar resultado JSON…")
        self.btn_save.setEnabled(False)
        self.btn_demo = QPushButton("Gerar dados demonstrativos")
        self.btn_demo.setToolTip(
            "Cria uma sessão sintética marcada como DADOS SINTÉTICOS; "
            "não pode ser combinada silenciosamente com dados reais."
        )
        self.btn_demo.setObjectName("AdvancedButton")
        export_row = QHBoxLayout()
        export_row.addWidget(self.btn_save)
        export_row.addWidget(self.btn_demo)
        root.addLayout(export_row)
        root.addStretch(1)

        self._tick = QTimer(self)
        self._tick.setInterval(100)
        self._tick.timeout.connect(self._update_timer)
        self.btn_start.clicked.connect(self.start_current_task)
        self.btn_pause.clicked.connect(self.pause_current_task)
        self.btn_resume.clicked.connect(self.resume_current_task)
        self.btn_success.clicked.connect(lambda: self.finish_current_task(True))
        self.btn_fail.clicked.connect(lambda: self.finish_current_task(False))
        self.btn_cancel.clicked.connect(self.cancel_current_task)
        self.btn_save.clicked.connect(self._save_dialog)
        self.btn_demo.clicked.connect(self._generate_demo_session)
        self._render_task()

    # ------------------------------------------------------------------
    # Session control
    # ------------------------------------------------------------------
    def start_current_task(self) -> None:
        if self._running or self._task_index >= len(self._tasks):
            return
        if not self._validate_participant():
            return
        self._running = True
        self._paused = False
        self._elapsed.start()
        self._tick.start()
        self._set_editing_locked(True)
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)
        self.btn_resume.setEnabled(False)
        self.btn_success.setEnabled(True)
        self.btn_fail.setEnabled(True)
        self.btn_cancel.setEnabled(True)
        self.btn_save.setEnabled(False)
        announce(self.task_title, f"Cronômetro iniciado. {self.task_title.text()}")

    def pause_current_task(self) -> None:
        if not self._running or self._paused:
            return
        self._paused = True
        self._tick.stop()
        self._elapsed.invalidate()
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(True)
        announce(self.task_title, "Tarefa pausada. O tempo parou de contar.")

    def resume_current_task(self) -> None:
        if not self._running or not self._paused:
            return
        self._paused = False
        self._elapsed.restart()
        self._tick.start()
        self.btn_pause.setEnabled(True)
        self.btn_resume.setEnabled(False)
        announce(self.task_title, "Tarefa retomada.")

    def cancel_current_task(self) -> None:
        if not self._running:
            return
        self._tick.stop()
        self._running = False
        self._paused = False
        elapsed_s = max(0.0, self._elapsed.elapsed() / 1000.0)
        task = self._tasks[self._task_index]
        self._results.append(
            OperatorTaskResult(
                task=task.title,
                success=False,
                elapsed_s=round(elapsed_s, 3),
                errors=int(self.errors.value()),
                assistance=bool(self.assistance.isChecked()),
                cancelled=True,
                false_command=bool(self.false_command.isChecked()),
                notes=_sanitize_notes(self.notes.toPlainText()),
            )
        )
        self._advance_task()

    def finish_current_task(self, success: bool) -> None:
        if not self._running:
            return
        self._tick.stop()
        elapsed_s = max(0.0, self._elapsed.elapsed() / 1000.0)
        task = self._tasks[self._task_index]
        self._results.append(
            OperatorTaskResult(
                task=task.title,
                success=bool(success),
                elapsed_s=round(elapsed_s, 3),
                errors=int(self.errors.value()),
                assistance=bool(self.assistance.isChecked()),
                cancelled=False,
                false_command=bool(self.false_command.isChecked()),
                notes=_sanitize_notes(self.notes.toPlainText()),
            )
        )
        self._running = False
        self._paused = False
        self._advance_task()

    def _advance_task(self) -> None:
        self._task_index += 1
        self.errors.setValue(0)
        self.assistance.setChecked(False)
        self.false_command.setChecked(False)
        self.cancelled.setChecked(False)
        self.notes.clear()
        self._render_task()

    def _validate_participant(self) -> bool:
        code = self.participant.text().strip()
        if not code:
            self.participant.setFocus()
            QMessageBox.warning(
                self, "Identificação necessária",
                "Informe apenas um código anônimo do participante."
            )
            return False
        if len(code) > 16 or not re.match(r"^[A-Za-z0-9._-]+$", code):
            QMessageBox.warning(
                self, "Código inválido",
                "Use até 16 caracteres alfanuméricos, ponto, hífen ou underscore."
            )
            return False
        self._session.participant_code = code
        self._session.experience = self.experience.currentText()
        self._session.environment.input_mode = self.input_mode.currentText()
        if not self._session.started_at:
            self._session.started_at = datetime.now().astimezone().isoformat(timespec="seconds")
        return True

    def _set_editing_locked(self, locked: bool) -> None:
        self.participant.setEnabled(not locked)
        self.experience.setEnabled(not locked)
        self.input_mode.setEnabled(not locked)
        self.errors.setEnabled(locked)
        self.assistance.setEnabled(locked)
        self.false_command.setEnabled(locked)
        self.cancelled.setEnabled(locked)
        self.notes.setEnabled(locked)

    def session_payload(self) -> dict[str, object]:
        """Return the current session as an anonymised dictionary."""
        self._session.tasks = list(self._results)
        self._session.finished_at = datetime.now().astimezone().isoformat(timespec="seconds")
        self._session.facilitator_notes = _sanitize_notes(self.notes.toPlainText())
        return self._session.to_dict()

    def save_results(self, path: str | Path) -> Path:
        """Atomically write the individual session JSON."""
        target = Path(path)
        payload = self.session_payload()
        payload["exported_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_suffix(target.suffix + ".tmp")
        temp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temp.replace(target)
        return target

    def _save_dialog(self) -> None:
        default = Path.home() / "rebotarm_usability" / (
            f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        path, _selected = QFileDialog.getSaveFileName(
            self, "Exportar avaliação", str(default), "JSON (*.json)"
        )
        if not path:
            return
        target = self.save_results(path)
        QMessageBox.information(self, "Avaliação exportada", f"Resultado salvo em:\n{target}")

    def _generate_demo_session(self) -> None:
        """Create a synthetic session marked as DADOS SINTÉTICOS."""
        reply = QMessageBox.question(
            self,
            "Dados sintéticos",
            "Isso gera uma sessão de demonstração marcada como DADOS SINTÉTICOS. "
            "Deseja continuar?",
        )
        if reply != QMessageBox.Yes:
            return
        demo = UsabilitySession(
            participant_code="DEMO-00",
            experience=_EXPERIENCE_OPTIONS[0],
            environment=_environment(),
            synthetic=True,
            started_at=datetime.now().astimezone().isoformat(timespec="seconds"),
            finished_at=datetime.now().astimezone().isoformat(timespec="seconds"),
        )
        for index, task in enumerate(self._tasks):
            demo.tasks.append(
                OperatorTaskResult(
                    task=task.title,
                    success=True,
                    elapsed_s=round(10.0 + index * 5.0, 3),
                    errors=0,
                    assistance=False,
                    cancelled=False,
                    false_command=False,
                    notes="Dados sintéticos — não usar como evidência humana.",
                )
            )
        target = Path.home() / "rebotarm_usability" / "demo_session.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_suffix(target.suffix + ".tmp")
        temp.write_text(
            json.dumps(demo.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temp.replace(target)
        QMessageBox.information(
            self,
            "Dados demonstrativos",
            f"Sessão sintética salva em:\n{target}\n\n"
            "Ela está marcada como DADOS SINTÉTICOS e não deve ser misturada com dados reais.",
        )

    def _render_task(self) -> None:
        done = self._task_index >= len(self._tasks)
        if done:
            self._session.finished_at = datetime.now().astimezone().isoformat(timespec="seconds")
            self.progress.setText(f"{len(self._tasks)} de {len(self._tasks)} tarefas")
            self.task_title.setText("Sessão concluída")
            self.task_instruction.setText("Revise as anotações e exporte o resultado.")
            self.success_criterion.setText("")
            self.timer_label.setText("Concluído")
            self.btn_start.setEnabled(False)
            self.btn_pause.setEnabled(False)
            self.btn_resume.setEnabled(False)
            self.btn_success.setEnabled(False)
            self.btn_fail.setEnabled(False)
            self.btn_cancel.setEnabled(False)
            self.btn_save.setEnabled(True)
            self._set_editing_locked(False)
            announce(self.task_title, "Sessão concluída. Exporte o resultado.")
            self.session_completed.emit(self.session_payload())
            return
        task = self._tasks[self._task_index]
        self.progress.setText(f"Tarefa {self._task_index + 1} de {len(self._tasks)}")
        self.task_title.setText(task.title)
        self.task_instruction.setText(task.instruction)
        self.success_criterion.setText(f"Critério: {task.success_criterion}")
        self.timer_label.setText("00:00.0")
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(False)
        self.btn_success.setEnabled(False)
        self.btn_fail.setEnabled(False)
        self.btn_cancel.setEnabled(False)
        self.btn_save.setEnabled(False)
        self._set_editing_locked(False)
        if self._task_index > 0:
            announce(
                self.task_title,
                f"Próxima tarefa: {task.title}. {task.instruction}",
            )

    def _update_timer(self) -> None:
        elapsed = max(0, self._elapsed.elapsed())
        minutes, remainder = divmod(elapsed, 60_000)
        seconds = remainder / 1000.0
        self.timer_label.setText(f"{minutes:02d}:{seconds:04.1f}")


__all__ = [
    "OperatorTask",
    "OperatorTaskResult",
    "UsabilitySession",
    "UsabilityEnvironment",
    "UsabilityAggregator",
    "AggregateMetrics",
    "OperatorTestPanel",
]
