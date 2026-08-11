"""Shared keyboard and assistive-technology helpers for the GUI.

Qt exposes most native controls to accessibility APIs automatically, but
custom layouts still need explicit label associations and meaningful names.
This module keeps those rules in one place so every tab gets the same baseline.
"""

from __future__ import annotations

import json
import os
import platform
import re
from collections.abc import Iterator
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Tuple

from PySide6.QtCore import Qt
from PySide6.QtGui import QAccessible, QAccessibleAnnouncementEvent
from PySide6.QtWidgets import (
    QAbstractButton,
    QAbstractItemView,
    QAbstractSlider,
    QAbstractSpinBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QLabel,
    QLayout,
    QLineEdit,
    QTabBar,
    QTextEdit,
    QWidget,
)


_INTERACTIVE_TYPES = (
    QAbstractButton,
    QAbstractItemView,
    QAbstractSlider,
    QAbstractSpinBox,
    QComboBox,
    QLineEdit,
    QTabBar,
    QTextEdit,
)
_SYMBOL_ONLY = re.compile(r"^[\s×+−▲▼◀▶…._/-]+$")


def _plain(text: str) -> str:
    """Return UI text without mnemonic markers or trailing punctuation."""
    return " ".join(str(text).replace("&&", "&").replace("&", "").split()).rstrip(":")


def _layout_tree(layout: QLayout | None) -> Iterator[QLayout]:
    if layout is None:
        return
    yield layout
    for index in range(layout.count()):
        item = layout.itemAt(index)
        child_layout = item.layout()
        if child_layout is not None:
            yield from _layout_tree(child_layout)
        widget = item.widget()
        if widget is not None and widget.layout() is not None:
            yield from _layout_tree(widget.layout())


def _first_interactive_in_item(item) -> QWidget | None:
    widget = item.widget() if item is not None else None
    if isinstance(widget, _INTERACTIVE_TYPES):
        return widget
    if widget is not None:
        children = widget.findChildren(QWidget)
        return next((child for child in children if isinstance(child, _INTERACTIVE_TYPES)), None)
    layout = item.layout() if item is not None else None
    if layout is not None:
        for index in range(layout.count()):
            found = _first_interactive_in_item(layout.itemAt(index))
            if found is not None:
                return found
    return None


def _associate_form_labels(root: QWidget) -> None:
    """Set QLabel buddies and field names for every QFormLayout under ``root``."""
    layouts = list(root.findChildren(QFormLayout))
    if isinstance(root.layout(), QFormLayout):
        layouts.insert(0, root.layout())
    for layout in layouts:
        for row in range(layout.rowCount()):
            label_item = layout.itemAt(row, QFormLayout.LabelRole)
            field_item = layout.itemAt(row, QFormLayout.FieldRole)
            label = label_item.widget() if label_item is not None else None
            field = _first_interactive_in_item(field_item)
            if not isinstance(label, QLabel) or field is None:
                continue
            name = _plain(label.text())
            if not name:
                continue
            label.setBuddy(field)
            if not field.accessibleName():
                field.setAccessibleName(name)


def _associate_grid_labels(root: QWidget) -> None:
    """Associate explicit ``FieldLabel`` cells with the next control in a row."""
    layouts = list(root.findChildren(QGridLayout))
    if isinstance(root.layout(), QGridLayout):
        layouts.insert(0, root.layout())
    for layout in layouts:
        for index in range(layout.count()):
            item = layout.itemAt(index)
            label = item.widget()
            if not isinstance(label, QLabel) or label.objectName() != "FieldLabel":
                continue
            row, column, _row_span, _column_span = layout.getItemPosition(index)
            field = None
            for candidate_column in range(column + 1, layout.columnCount()):
                field = _first_interactive_in_item(
                    layout.itemAtPosition(row, candidate_column)
                )
                if field is not None:
                    break
            name = _plain(label.text())
            if field is not None and name:
                label.setBuddy(field)
                if not field.accessibleName():
                    field.setAccessibleName(name)


def effective_accessible_name(widget: QWidget) -> str:
    """Return the explicit or native text name exposed for a widget."""
    explicit = _plain(widget.accessibleName())
    if explicit:
        return explicit
    if isinstance(widget, QAbstractButton):
        text = _plain(widget.text())
        if text and not _SYMBOL_ONLY.match(text):
            return text
    if isinstance(widget, QLineEdit):
        placeholder = _plain(widget.placeholderText())
        if placeholder:
            return placeholder
    return ""


def configure_accessibility(root: QWidget) -> None:
    """Apply the common accessible-name, description and buddy baseline."""
    _associate_form_labels(root)
    _associate_grid_labels(root)
    widgets = [root, *root.findChildren(QWidget)]
    for widget in widgets:
        if not isinstance(widget, _INTERACTIVE_TYPES):
            continue
        tip = _plain(widget.toolTip())
        name = effective_accessible_name(widget)
        if not name and isinstance(widget, QAbstractButton) and tip:
            name = tip.split("(", 1)[0].strip().rstrip(".")
        if not name and tip:
            name = tip.split("\n", 1)[0].rstrip(".")
        if not name and widget.objectName():
            name = _plain(widget.objectName().replace("_", " "))
        if name and not widget.accessibleName():
            widget.setAccessibleName(name)
        if tip and not widget.accessibleDescription():
            widget.setAccessibleDescription(tip)


def first_focusable(root: QWidget) -> QWidget | None:
    """Find the first enabled, visible control in Qt's normal tab focus set."""
    candidates = [root, *root.findChildren(QWidget)]
    for widget in candidates:
        if not isinstance(widget, _INTERACTIVE_TYPES):
            continue
        if not widget.isEnabled() or not widget.isVisibleTo(root):
            continue
        if widget.focusPolicy() & Qt.TabFocus:
            return widget
    return None


def focus_first(root: QWidget) -> QWidget | None:
    """Move keyboard focus to the first operable control under ``root``."""
    target = first_focusable(root)
    if target is not None:
        target.setFocus(Qt.ShortcutFocusReason)
    return target


def announce(widget: QWidget, message: str, *, assertive: bool = False) -> None:
    """Send an explicit live announcement to the platform accessibility API."""
    text = _plain(message)
    if not text:
        return
    widget.setAccessibleDescription(text)
    event = QAccessibleAnnouncementEvent(widget, text)
    event.setPoliteness(
        QAccessible.AnnouncementPoliteness.Assertive
        if assertive
        else QAccessible.AnnouncementPoliteness.Polite
    )
    QAccessible.updateAccessibility(event)


# ------------------------------------------------------------------------------
# Accessibility audit helper (#5)
# ------------------------------------------------------------------------------


@dataclass
class AccessibilityIssue:
    page: str
    widget_type: str
    object_name: str
    issue: str
    severity: str  # error | warning | info
    accessible_name: str = ""
    suggested_fix: str = ""


class AccessibilityAuditor:
    """Automated checks that support (but do not replace) a screen-reader session.

    The auditor walks the widgets of every page, records every interactive
    control, and reports missing or duplicate names, missing descriptions,
    missing label buddies, tab-focus without a name, and suspicious generic
    names.  The JSON/Markdown reports are meant to be handed to a human tester
    running Orca/NVDA; they do not certify conformance.
    """

    GENERIC_NAMES = {
        "ok", "cancel", "apply", "close", "button", "btn", "pushbutton",
        "lineedit", "combobox", "spinbox", "slider", "checkbox", "listwidget",
        "tablewidget", "widget", "qwidget", "frame", "qframe", "tab",
    }

    def __init__(self) -> None:
        self.issues: List[AccessibilityIssue] = []
        self.controls: List[Dict[str, object]] = []

    def audit_page(self, page_key: str, page: QWidget) -> None:
        """Record every interactive control under ``page`` and flag problems."""
        seen_names: Dict[str, List[str]] = {}
        for widget in [page, *page.findChildren(QWidget)]:
            if not isinstance(widget, _INTERACTIVE_TYPES):
                continue
            visible = widget.isVisibleTo(page)
            name = effective_accessible_name(widget)
            description = _plain(widget.accessibleDescription())
            buddy = getattr(widget, "buddy", lambda: None)()
            has_tab_focus = bool(widget.focusPolicy() & Qt.TabFocus)
            interface = QAccessible.queryAccessibleInterface(widget)
            role = interface.role() if interface is not None else "Unknown"
            enabled = widget.isEnabled()
            record: Dict[str, object] = {
                "page": page_key,
                "type": type(widget).__name__,
                "object_name": widget.objectName(),
                "accessible_name": name,
                "accessible_description": description,
                "has_tab_focus": has_tab_focus,
                "enabled": enabled,
                "visible": visible,
                "role": str(role),
                "buddy_label": buddy.objectName() if buddy else None,
            }
            self.controls.append(record)

            if not visible:
                continue

            context = f"{page_key}:{type(widget).__name__}:{widget.objectName()}"
            if has_tab_focus and not name:
                self.issues.append(
                    AccessibilityIssue(
                        page=page_key,
                        widget_type=type(widget).__name__,
                        object_name=widget.objectName(),
                        issue="Controle alcançável por Tab não possui nome acessível.",
                        severity="error",
                        accessible_name=name,
                        suggested_fix="Adicionar accessibleName ou rótulo associado via buddy.",
                    )
                )
            elif name:
                plain_name = name.lower()
                if plain_name in self.GENERIC_NAMES or len(plain_name) < 2:
                    self.issues.append(
                        AccessibilityIssue(
                            page=page_key,
                            widget_type=type(widget).__name__,
                            object_name=widget.objectName(),
                            issue="Nome acessível é genérico ou muito curto.",
                            severity="warning",
                            accessible_name=name,
                            suggested_fix="Usar um nome que descreva a ação ou o campo.",
                        )
                    )
                seen_names.setdefault(name, []).append(context)

            if isinstance(widget, QLineEdit) and not buddy and not description:
                self.issues.append(
                    AccessibilityIssue(
                        page=page_key,
                        widget_type=type(widget).__name__,
                        object_name=widget.objectName(),
                        issue="Campo de texto sem rótulo associado (buddy) nem descrição.",
                        severity="warning",
                        accessible_name=name,
                        suggested_fix="Associar um QLabel como buddy ou adicionar accessibleDescription.",
                    )
                )

            if not description and name and has_tab_focus:
                self.issues.append(
                    AccessibilityIssue(
                        page=page_key,
                        widget_type=type(widget).__name__,
                        object_name=widget.objectName(),
                        issue="Controle interativo sem accessibleDescription.",
                        severity="info",
                        accessible_name=name,
                        suggested_fix="Adicionar accessibleDescription com contexto de uso.",
                    )
                )

        for name, contexts in seen_names.items():
            if len(contexts) > 1:
                for context in contexts:
                    page_key, widget_type, object_name = context.split(":", 2)
                    self.issues.append(
                        AccessibilityIssue(
                            page=page_key,
                            widget_type=widget_type,
                            object_name=object_name,
                            issue=f"Nome acessível duplicado dentro da página: {name!r}.",
                            severity="warning",
                            accessible_name=name,
                            suggested_fix="Tornar o nome único ou incluir contexto (ex: 'Velocidade X').",
                        )
                    )

    def _environment(self) -> Dict[str, object]:
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
        return {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "qt_version": qt_version,
            "pyside_version": pyside_version,
            "session_graphic": "offscreen" if os.environ.get("QT_QPA_PLATFORM") == "offscreen" else "native",
            "scale_factor": os.environ.get("QT_SCALE_FACTOR", "1.0"),
            "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        }

    def to_json(self, pages: List[Tuple[str, QWidget]]) -> str:
        """Run the audit and return a JSON report."""
        self.issues.clear()
        self.controls.clear()
        for key, page in pages:
            self.audit_page(key, page)
        return json.dumps(
            {
                "schema_version": 1,
                "environment": self._environment(),
                "summary": {
                    "pages": len(pages),
                    "controls": len(self.controls),
                    "errors": sum(1 for i in self.issues if i.severity == "error"),
                    "warnings": sum(1 for i in self.issues if i.severity == "warning"),
                    "infos": sum(1 for i in self.issues if i.severity == "info"),
                },
                "issues": [field.asdict(i) for i in self.issues],
                "controls": self.controls,
                "manual_checklist": self._manual_checklist(),
            },
            ensure_ascii=False,
            indent=2,
            default=str,
        )

    def to_markdown(self, pages: List[Tuple[str, QWidget]]) -> str:
        """Run the audit and return a Markdown report for a human tester."""
        self.issues.clear()
        self.controls.clear()
        for key, page in pages:
            self.audit_page(key, page)
        env = self._environment()
        lines = [
            "# Relatório de auditoria de acessibilidade — reBotArm GUI",
            "",
            "> Este relatório é produzido por verificação automatizada. **Não substitui** uma sessão real com Orca, NVDA ou outro leitor de tela no ambiente de implantação.",
            "",
            "## Ambiente",
            "",
        ]
        for k, v in env.items():
            lines.append(f"- **{k}:** {v}")
        lines.extend([
            "",
            "## Resumo",
            "",
            f"- Páginas: {len(pages)}",
            f"- Controles interativos: {len(self.controls)}",
            f"- Erros: {sum(1 for i in self.issues if i.severity == 'error')}",
            f"- Avisos: {sum(1 for i in self.issues if i.severity == 'warning')}",
            f"- Infos: {sum(1 for i in self.issues if i.severity == 'info')}",
            "",
            "## Problemas",
            "",
        ])
        if not self.issues:
            lines.append("Nenhum problema automatizado encontrado.")
        else:
            lines.append("| Página | Tipo | Objeto | Severidade | Problema | Nome | Sugestão |")
            lines.append("|---|---|---|---|---|---|---|")
            for issue in self.issues:
                lines.append(
                    f"| {issue.page} | {issue.widget_type} | {issue.object_name} | "
                    f"{issue.severity} | {issue.issue} | {issue.accessible_name} | {issue.suggested_fix} |"
                )
        lines.extend(["", "## Roteiro manual", ""])
        for item in self._manual_checklist():
            lines.append(f"- [ ] {item}")
        lines.append("")
        return "\n".join(lines)

    @staticmethod
    def _manual_checklist() -> List[str]:
        return [
            "Linux + Orca: navegar por Tab/Shift+Tab em todas as páginas; confirmar que cada controle anuncia nome e papel.",
            "Linux + Orca: verificar que alarmes críticos são anunciados uma única vez (sem repetição excessiva).",
            "Linux + Orca: verificar que F6/Shift+F6 alterna entre regiões sem perder foco.",
            "Windows + NVDA (ou leitor adotado): repetir as verificações acima; se usar WSLg, registrar a ponte exata.",
            "Confirmar que barra de progresso e valores de velocidade têm descrição compreensível (ex: 'progresso do movimento 45%').",
            "Confirmar que botões de jog/move incluem direção, eixo ou junta no nome (ex: 'Jog junta 1 positivo').",
            "Confirmar que foco não é deslocado por toasts, alarmes ou atualizações periódicas de telemetria.",
            "Registrar resultado: não executado / aprovado / falhou / não aplicável para cada verificação.",
        ]


__all__ = [
    "configure_accessibility",
    "effective_accessible_name",
    "first_focusable",
    "focus_first",
    "announce",
    "AccessibilityAuditor",
    "AccessibilityIssue",
]
