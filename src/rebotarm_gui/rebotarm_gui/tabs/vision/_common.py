# ruff: noqa: F401
"""Computer-vision configuration tab."""

from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import Image as SensorImage
from rebotarm_vision.task_config import VisionTaskConfig

from ...calibration import (
    DEFAULT_MAX_ERROR_M,
    DEFAULT_MAX_RMS_ERROR_M,
    MIN_APPROVAL_POINTS,
    CalibrationPoint,
    CalibrationProfile,
    CalibrationStore,
    STATUS_APPROVED,
)
from ...security import CAP_CALIBRATION_APPROVE, AuthorizationContext
from ...settings import AppSettings
from ...widgets.combo_box import RetractingComboBox
from ...widgets.factory import make_button
from ...widgets.layout_utils import fit_button_text


VISION_SIDE_PANEL_WIDTH = 600
DEFAULT_TEST_IMAGE = "default_vision_test.png"
FALLBACK_TEST_IMAGE = "garrafas.png"
DEFAULT_AI_SCENARIO_PROMPT = (
    "25 garrafas PET transparentes e vazias. "
    "Detectar todas as unidades individuais na imagem. "
    "A zona de pega segura fica no centro do corpo de cada garrafa."
)


class VisionPreviewLabel(QLabel):
    image_clicked = Signal(float, float)

    def __init__(self, text: str = "", parent: QWidget | None = None) -> None:
        super().__init__(text, parent)
        self._source_width = 0
        self._source_height = 0
        self._overlay_text = ""
        self.setAccessibleName("Pré-visualização da câmera e área de trabalho")
        self.setAccessibleDescription(
            "Imagem usada para revisar detecções e, quando habilitado, marcar limites operacionais."
        )

    def set_source_size(self, width: int, height: int) -> None:
        self._source_width = int(width)
        self._source_height = int(height)

    def set_overlay_text(self, text: str) -> None:
        self._overlay_text = str(text)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        super().paintEvent(event)
        pixmap = self.pixmap()
        if pixmap is None or not self._overlay_text:
            return
        shown_w = float(pixmap.width())
        shown_h = float(pixmap.height())
        offset_x = max(0.0, (float(self.width()) - shown_w) * 0.5)
        offset_y = max(0.0, (float(self.height()) - shown_h) * 0.5)
        margin = 10
        pad_x = 8
        pad_y = 5
        max_width = max(40, int(shown_w) - margin * 2)
        painter = QPainter(self)
        metrics = painter.fontMetrics()
        text = metrics.elidedText(self._overlay_text, Qt.TextElideMode.ElideRight, max_width - pad_x * 2)
        text_width = metrics.horizontalAdvance(text)
        box_width = min(max_width, text_width + pad_x * 2)
        box_height = metrics.height() + pad_y * 2
        x = int(round(offset_x)) + margin
        y = int(round(offset_y + shown_h)) - margin - box_height
        painter.fillRect(x, y, box_width, box_height, QColor(0, 0, 0, 220))
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x + pad_x, y + pad_y + metrics.ascent(), text)
        painter.end()

    def mousePressEvent(self, event) -> None:  # noqa: N802
        pixmap = self.pixmap()
        if pixmap is None or self._source_width <= 0 or self._source_height <= 0:
            super().mousePressEvent(event)
            return
        pos = event.position() if hasattr(event, "position") else event.pos()
        px = float(pos.x())
        py = float(pos.y())
        shown_w = float(pixmap.width())
        shown_h = float(pixmap.height())
        offset_x = max(0.0, (float(self.width()) - shown_w) * 0.5)
        offset_y = max(0.0, (float(self.height()) - shown_h) * 0.5)
        if px < offset_x or py < offset_y or px > offset_x + shown_w or py > offset_y + shown_h:
            return
        u = (px - offset_x) * float(self._source_width) / max(shown_w, 1.0)
        v = (py - offset_y) * float(self._source_height) / max(shown_h, 1.0)
        self.image_clicked.emit(u, v)


class PreviewStatusLabel(QLabel):
    def __init__(self, preview: VisionPreviewLabel, text: str = "") -> None:
        super().__init__(text)
        self._preview = preview
        self._preview.set_overlay_text(text)

    def setText(self, text: str) -> None:  # noqa: N802
        super().setText(text)
        self._preview.set_overlay_text(text)


class IntComboBox(RetractingComboBox):
    def __init__(self, minimum: int, maximum: int, value: int) -> None:
        super().__init__()
        self.addItems([str(number) for number in range(minimum, maximum + 1)])
        self.setCurrentText(str(value))

    def value(self) -> int:
        try:
            return int(self.currentText())
        except ValueError:
            return 0

    def setValue(self, value: int | float) -> None:  # noqa: N802
        self.setCurrentText(str(int(round(float(value)))))


