"""Dialog to display the last torque safety event/replay."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from .layout_utils import fit_button_text


DEFAULT_EVENT_PATH = Path.home() / ".config" / "rebotarmcontroller" / "last_safety_event.json"


class SafetyEventDialog(QDialog):
    """Show the last saved safety event with samples and a torque plot."""

    def __init__(self, path: Path | str | None = None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._path = Path(path) if path else DEFAULT_EVENT_PATH
        self.setWindowTitle("Detalhes da última falha de segurança")
        self.resize(900, 600)

        self._summary = QLabel("Nenhum evento registrado.")
        self._summary.setWordWrap(True)

        self._table = QTableWidget(0, 5)
        self._table.setHorizontalHeaderLabels(
            ["Amostra", "Fase", "Junta", "Torque", "Timestamp"]
        )
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self._figure = Figure(figsize=(5, 3), dpi=100)
        self._canvas = FigureCanvas(self._figure)
        self._ax = self._figure.add_subplot(111)
        self._ax.set_xlabel("Amostra")
        self._ax.set_ylabel("Torque")
        self._ax.set_title("Torque nas amostras antes da falha")
        self._ax.grid(True, linestyle=":", alpha=0.6)

        btn_close = QPushButton("Fechar")
        btn_close.clicked.connect(self.accept)
        btn_refresh = QPushButton("Atualizar")
        btn_refresh.clicked.connect(self._load)
        fit_button_text(btn_close)
        fit_button_text(btn_refresh)

        buttons = QHBoxLayout()
        buttons.addWidget(btn_refresh)
        buttons.addStretch(1)
        buttons.addWidget(btn_close)

        layout = QVBoxLayout(self)
        layout.addWidget(self._summary)
        layout.addWidget(self._table, 1)
        layout.addWidget(self._canvas, 1)
        layout.addLayout(buttons)

        self._load()

    def _load(self) -> None:
        try:
            import json

            with open(self._path, "r", encoding="utf-8") as f:
                event = json.load(f)
        except Exception as exc:
            self._summary.setText(f"Não foi possível carregar o evento: {exc}")
            self._table.setRowCount(0)
            self._ax.clear()
            self._canvas.draw()
            return

        self._populate(event)

    def _populate(self, event: dict[str, Any]) -> None:
        severity = event.get("severity", "?")
        message = event.get("message", "")
        phase = event.get("phase", 0.0)
        step_id = event.get("step_id", "")
        saved_at = event.get("saved_at", "?")
        self._summary.setText(
            f"<b>Severidade:</b> {severity}<br>"
            f"<b>Mensagem:</b> {message}<br>"
            f"<b>Fase:</b> {phase:.4f}<br>"
            f"<b>Step:</b> {step_id}<br>"
            f"<b>Salvo em:</b> {saved_at}"
        )

        samples = event.get("samples", [])
        rows: list[tuple[int, float, str, float, str]] = []
        for index, sample in enumerate(samples):
            phase = sample.get("phase", 0.0)
            timestamp = sample.get("timestamp", "")
            names = sample.get("joint_names", [])
            torques = sample.get("torques", [])
            for name, torque in zip(names, torques):
                rows.append((index, float(phase), str(name), float(torque), timestamp))

        self._table.setRowCount(len(rows))
        for row, (idx, phase, name, torque, timestamp) in enumerate(rows):
            values = [str(idx), f"{phase:.4f}", name, f"{torque:+.4f}", timestamp]
            for col, text in enumerate(values):
                self._table.setItem(row, col, QTableWidgetItem(text))

        self._plot(samples)

    def _plot(self, samples: list[dict[str, Any]]) -> None:
        self._ax.clear()
        self._ax.set_xlabel("Amostra")
        self._ax.set_ylabel("Torque")
        self._ax.set_title("Torque nas amostras antes da falha")
        self._ax.grid(True, linestyle=":", alpha=0.6)

        if not samples:
            self._canvas.draw()
            return

        # Plot torque for each joint over sample index.
        names = samples[-1].get("joint_names", []) if samples else []
        for name in names:
            index_to_torque: dict[int, float] = {}
            for idx, sample in enumerate(samples):
                try:
                    j_idx = sample["joint_names"].index(name)
                    index_to_torque[idx] = float(sample["torques"][j_idx])
                except (ValueError, IndexError):
                    continue
            if index_to_torque:
                xs = sorted(index_to_torque.keys())
                ys = [index_to_torque[x] for x in xs]
                self._ax.plot(xs, ys, label=name, linewidth=1.2)

        self._ax.axvline(
            len(samples) - 1, color="red", linestyle="--", alpha=0.7, label="falha"
        )
        self._ax.legend(loc="best", fontsize="small")
        self._canvas.draw()
