"""MIT raw command tab: per-joint MIT / pos-vel command publisher."""

from __future__ import annotations

from typing import Dict

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..widgets.layout_utils import fit_button_text


class _JointMitRow(QWidget):
    """One row of MIT/pos-vel inputs for a single joint."""

    def __init__(self, joint_name: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.joint_name = joint_name

        self.lbl = QLabel(f"<b>{joint_name}</b>")
        self.lbl.setMinimumWidth(90)

        self.pos = self._spin(-12.6, 12.6, 0.0, " rad")
        self.vel = self._spin(-30.0, 30.0, 0.0, " rad/s")
        self.kp = self._spin(0.0, 500.0, 0.0, "")
        self.kd = self._spin(0.0, 50.0, 0.0, "")
        self.tau = self._spin(-30.0, 30.0, 0.0, " Nm")
        self.vlim = self._spin(0.0, 30.0, 0.0, " rad/s")

        lay = QHBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(10)
        lay.addWidget(self.lbl)
        lay.addWidget(self._wrap("pos", self.pos))
        lay.addWidget(self._wrap("vel", self.vel))
        lay.addWidget(self._wrap("kp", self.kp))
        lay.addWidget(self._wrap("kd", self.kd))
        lay.addWidget(self._wrap("tau", self.tau))
        lay.addWidget(self._wrap("vlim", self.vlim))
        self.setMinimumHeight(76)

    def _spin(self, mn, mx, val, suffix) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(mn, mx)
        s.setValue(val)
        s.setDecimals(2)
        s.setSingleStep(0.5)
        s.setSuffix(suffix)
        s.setAlignment(Qt.AlignRight)
        s.setMinimumWidth(122)
        return s

    def _wrap(self, name, widget):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lab = QLabel(name)
        lab.setAlignment(Qt.AlignCenter)
        lab.setObjectName("MutedLabel")
        lay.addWidget(lab)
        lay.addWidget(widget)
        return box


class MitTab(QWidget):
    """Tab to publish raw MIT or pos-vel commands per joint."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._rows: Dict[str, _JointMitRow] = {}

        # ---- Mode selector ----
        self._mode_mit = QRadioButton("MIT (kp/kd/tau)")
        self._mode_mit.setChecked(True)
        self._mode_posvel = QRadioButton("Pos+Vel (pos/vlim)")
        grp = QButtonGroup(self)
        grp.addButton(self._mode_mit)
        grp.addButton(self._mode_posvel)

        # ---- Rows container ----
        self._host = QWidget()
        self._rows_lay = QVBoxLayout(self._host)
        self._rows_lay.setContentsMargins(8, 8, 8, 8)
        self._rows_lay.setSpacing(10)
        self._rows_lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._host)

        # ---- Buttons ----
        self._btn_send_all = QPushButton("Enviar todos")
        self._btn_send_all.setObjectName("PrimaryButton")
        self._btn_send_all.setMinimumHeight(40)
        self._btn_send_all.clicked.connect(self._send_all)
        self._btn_send_all.setEnabled(False)

        self._btn_stop = QPushButton("Zerar todos")
        self._btn_stop.setObjectName("WarningButton")
        self._btn_stop.setMinimumHeight(40)
        self._btn_stop.clicked.connect(self._zero_all)
        self._btn_stop.setEnabled(False)
        fit_button_text(self._btn_send_all)
        fit_button_text(self._btn_stop)

        top = QHBoxLayout()
        top.setSpacing(18)
        top.addWidget(self._mode_mit)
        top.addWidget(self._mode_posvel)
        top.addStretch(1)
        top.addWidget(self._btn_stop)
        top.addWidget(self._btn_send_all)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(12)
        lay.addLayout(top)
        warning = QLabel("Atenção: comandos brutos são publicados diretamente. Risco de movimento.")
        warning.setObjectName("WarningText")
        lay.addWidget(warning)
        lay.addWidget(scroll, 1)

    # ------------------------------------------------------------------
    def update_joint_state(self, msg) -> None:
        names = list(msg.name)
        if not names:
            return
        # Build rows once
        if not self._rows:
            insert_at = self._rows_lay.count() - 1
            for name in names:
                row = _JointMitRow(name)
                self._rows[name] = row
                self._rows_lay.insertWidget(insert_at, row)
                insert_at += 1

    def _send_all(self) -> None:
        if not self._btn_send_all.isEnabled():
            return
        mit = self._mode_mit.isChecked()
        for name, row in self._rows.items():
            if mit:
                self._bridge.publish_mit(
                    name,
                    float(row.pos.value()),
                    float(row.vel.value()),
                    float(row.kp.value()),
                    float(row.kd.value()),
                    float(row.tau.value()),
                )
            else:
                self._bridge.publish_posvel(
                    name, float(row.pos.value()), float(row.vlim.value())
                )

    def _zero_all(self) -> None:
        if not self._btn_stop.isEnabled():
            return
        for row in self._rows.values():
            row.pos.setValue(0.0)
            row.vel.setValue(0.0)
            row.kp.setValue(0.0)
            row.kd.setValue(0.0)
            row.tau.setValue(0.0)
            row.vlim.setValue(0.0)
        self._send_all()

    def set_robot_ready(self, ready: bool) -> None:
        """Gate raw publishers when the controller heartbeat is unavailable."""
        self._btn_send_all.setEnabled(bool(ready))
        self._btn_stop.setEnabled(bool(ready))
