"""In-app, anonymised task timer for moderated operator usability sessions."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

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
    notes: str


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


class OperatorTestPanel(QWidget):
    """Guided stopwatch that records one result per predefined task."""

    session_completed = Signal(dict)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAccessibleName("Avaliação de usabilidade com operador")
        self._tasks = list(DEFAULT_TASKS)
        self._results: list[OperatorTaskResult] = []
        self._task_index = 0
        self._running = False
        self._elapsed = QElapsedTimer()

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        intro = QLabel(
            "Sessão moderada e anonimizada. Use o mock ou uma célula já validada; "
            "o facilitador não deve orientar durante a tarefa."
        )
        intro.setWordWrap(True)
        intro.setObjectName("WarningText")
        root.addWidget(intro)

        form = QFormLayout()
        self.participant = QLineEdit()
        self.participant.setPlaceholderText("Código anônimo, ex.: OP-01")
        self.experience = QComboBox()
        self.experience.addItems(["Primeiro uso", "Ocasional", "Experiente"])
        form.addRow("Participante", self.participant)
        form.addRow("Experiência", self.experience)
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
        self.notes = QTextEdit()
        self.notes.setPlaceholderText("Observações objetivas; não registre dados pessoais.")
        self.notes.setMaximumHeight(90)
        metrics.addRow("Erros observados", self.errors)
        metrics.addRow("", self.assistance)
        metrics.addRow("Notas", self.notes)
        root.addLayout(metrics)

        buttons = QHBoxLayout()
        self.btn_start = QPushButton("Iniciar tarefa")
        self.btn_start.setObjectName("PrimaryButton")
        self.btn_success = QPushButton("Concluir com sucesso")
        self.btn_fail = QPushButton("Encerrar com falha")
        self.btn_fail.setObjectName("WarningButton")
        self.btn_success.setEnabled(False)
        self.btn_fail.setEnabled(False)
        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_success)
        buttons.addWidget(self.btn_fail)
        root.addLayout(buttons)

        self.btn_save = QPushButton("Exportar resultado JSON…")
        self.btn_save.setEnabled(False)
        root.addWidget(self.btn_save)
        root.addStretch(1)

        self._tick = QTimer(self)
        self._tick.setInterval(100)
        self._tick.timeout.connect(self._update_timer)
        self.btn_start.clicked.connect(self.start_current_task)
        self.btn_success.clicked.connect(lambda: self.finish_current_task(True))
        self.btn_fail.clicked.connect(lambda: self.finish_current_task(False))
        self.btn_save.clicked.connect(self._save_dialog)
        self._render_task()

    def start_current_task(self) -> None:
        if self._running or self._task_index >= len(self._tasks):
            return
        if not self.participant.text().strip():
            self.participant.setFocus()
            QMessageBox.warning(
                self, "Identificação necessária", "Informe apenas um código anônimo do participante."
            )
            return
        self._running = True
        self.participant.setEnabled(False)
        self.experience.setEnabled(False)
        self._elapsed.start()
        self._tick.start()
        self.btn_start.setEnabled(False)
        self.btn_success.setEnabled(True)
        self.btn_fail.setEnabled(True)
        self.errors.setEnabled(True)
        self.assistance.setEnabled(True)
        self.notes.setEnabled(True)
        announce(self.task_title, f"Cronômetro iniciado. {self.task_title.text()}")

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
                notes=self.notes.toPlainText().strip(),
            )
        )
        self._running = False
        self._task_index += 1
        self.errors.setValue(0)
        self.assistance.setChecked(False)
        self.notes.clear()
        self._render_task()

    def session_payload(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "participant_code": self.participant.text().strip(),
            "experience": self.experience.currentText(),
            "completed": len(self._results) == len(self._tasks),
            "results": [asdict(result) for result in self._results],
        }

    def save_results(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.session_payload(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
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

    def _render_task(self) -> None:
        done = self._task_index >= len(self._tasks)
        if done:
            self.progress.setText(f"{len(self._tasks)} de {len(self._tasks)} tarefas")
            self.task_title.setText("Sessão concluída")
            self.task_instruction.setText("Revise as anotações e exporte o resultado.")
            self.success_criterion.setText("")
            self.timer_label.setText("Concluído")
            self.btn_start.setEnabled(False)
            self.btn_success.setEnabled(False)
            self.btn_fail.setEnabled(False)
            self.btn_save.setEnabled(True)
            self.errors.setEnabled(False)
            self.assistance.setEnabled(False)
            self.notes.setEnabled(False)
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
        self.btn_success.setEnabled(False)
        self.btn_fail.setEnabled(False)
        self.errors.setEnabled(False)
        self.assistance.setEnabled(False)
        self.notes.setEnabled(False)
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
