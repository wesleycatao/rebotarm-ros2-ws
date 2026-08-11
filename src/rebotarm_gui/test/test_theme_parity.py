"""Theme parity, semantic-status colour and i18n regression coverage.

These tests guard three properties that previously regressed silently:

* the two base stylesheets define the same selectors (so adding a rule to one
  theme can no longer be forgotten in the other — the dark/light drift that
  left the light theme missing the breadcrumb separator and the collapsed
  sidebar rules);
* every semantic status colour stays WCAG-AA (>= 4.5:1) on the theme it
  belongs to, so the inline ``#3fb950``-style colours that broke contrast under
  the light theme cannot come back;
* the i18n foundation actually loads a translation table and ``tr`` returns it,
  while the default (Portuguese) is unchanged.
"""

from __future__ import annotations

import os
import re
import sys
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QLabel  # noqa: E402

from rebotarm_gui.i18n import load_translations, reset, tr  # noqa: E402
import rebotarm_gui  # noqa: E402
from rebotarm_gui.styles import theme, theme_tokens  # noqa: E402
from rebotarm_gui.styles.styling import (  # noqa: E402
    STATUS_FAIL,
    STATUS_OK,
    STATUS_WARN,
    apply_status,
    clear_status,
)

_STYLES_DIR = Path(theme.__file__).resolve().parent
_DARK_QSS = _STYLES_DIR / "dark.qss"
_LIGHT_QSS = _STYLES_DIR / "light.qss"
_LOCALE_EN = Path(rebotarm_gui.__file__).resolve().parent / "locale" / "en.json"


def _strip_comments(text: str) -> str:
    return re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)


def _selectors(text: str) -> set[str]:
    """Return the normalised set of QSS selectors declared in ``text``."""
    text = _strip_comments(text)
    out: set[str] = set()
    for raw, _body in re.findall(r"([^{}]+)\{([^{}]*)\}", text):
        for sel in raw.split(","):
            sel = sel.strip()
            if sel:
                out.add(re.sub(r"\s+", " ", sel))
    return out


def _lin(channel: float) -> float:
    return channel / 12.92 if channel <= 0.03928 else ((channel + 0.055) / 1.055) ** 2.4


def _relative_luminance(hex_colour: str) -> float:
    c = hex_colour.lstrip("#")
    r, g, b = (int(c[i : i + 2], 16) / 255.0 for i in (0, 2, 4))
    return 0.2126 * _lin(r) + 0.7152 * _lin(g) + 0.0722 * _lin(b)


def contrast_ratio(fg: str, bg: str) -> float:
    la, lb = _relative_luminance(fg), _relative_luminance(bg)
    hi, lo = max(la, lb), min(la, lb)
    return (hi + 0.05) / (lo + 0.05)


class TestStylesheetParity(unittest.TestCase):
    def test_dark_and_light_define_the_same_selectors(self) -> None:
        dark = _selectors(_DARK_QSS.read_text(encoding="utf-8"))
        light = _selectors(_LIGHT_QSS.read_text(encoding="utf-8"))
        missing_in_light = dark - light
        missing_in_dark = light - dark
        self.assertFalse(
            missing_in_light,
            f"selectors missing from light theme: {sorted(missing_in_light)}",
        )
        self.assertFalse(
            missing_in_dark,
            f"selectors missing from dark theme: {sorted(missing_in_dark)}",
        )

    def test_required_selectors_present_in_composed_output(self) -> None:
        # Mirror the contract asserted by test_accessibility plus the new
        # generated semantic block, for both themes.
        for name in ("dark", "light"):
            qss = theme.build_stylesheet(name)
            for selector in (
                "QPushButton:focus",
                "QLineEdit:focus",
                "QSlider::groove",
                'QLabel[status="ok"]',
                'QLabel[status="warn"]',
                'QLabel[status="fail"]',
                'QLabel#HeaderAlarmIndicatorIcon[severity="error"]',
            ):
                self.assertIn(selector, qss, f"{selector} missing in {name} theme")


class TestSemanticStatusColours(unittest.TestCase):
    def test_status_colours_are_aa_on_their_theme_background(self) -> None:
        for theme_name in theme_tokens.VALID_THEMES:
            bg = theme_tokens.THEME_BACKGROUND[theme_name]
            for level in theme_tokens.STATUS_LEVELS:
                fg = theme_tokens.status_color(theme_name, level)
                ratio = contrast_ratio(fg, bg)
                self.assertGreaterEqual(
                    ratio,
                    4.5,
                    f"{theme_name}/{level} {fg} on {bg} = {ratio:.2f}:1 (< 4.5)",
                )

    def test_severity_colours_are_aa_on_their_theme_background(self) -> None:
        for theme_name in theme_tokens.VALID_THEMES:
            bg = theme_tokens.THEME_BACKGROUND[theme_name]
            for sev in theme_tokens.SEVERITY_LEVELS:
                fg = theme_tokens.severity_color(theme_name, sev)
                self.assertGreaterEqual(
                    contrast_ratio(fg, bg),
                    4.5,
                    f"{theme_name}/{sev} {fg} on {bg} (< 4.5:1)",
                )

    def test_dark_status_palette_matches_legacy_inline_colours(self) -> None:
        # The dark theme must not change visually: the new tokens reproduce the
        # exact colours that used to be inlined in the preview dialog.
        self.assertEqual(theme_tokens.status_color("dark", STATUS_OK), "#3fb950")
        self.assertEqual(theme_tokens.status_color("dark", STATUS_WARN), "#f0883e")
        self.assertEqual(theme_tokens.status_color("dark", STATUS_FAIL), "#f85149")


class TestApplyStatus(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_apply_and_clear_status_sets_property(self) -> None:
        label = QLabel("x")
        apply_status(label, STATUS_OK)
        self.assertEqual(label.property("status"), STATUS_OK)
        apply_status(label, STATUS_FAIL)
        self.assertEqual(label.property("status"), STATUS_FAIL)
        clear_status(label)
        self.assertIn(label.property("status"), ("", None))


class TestI18nAdoption(unittest.TestCase):
    def test_default_is_portuguese_and_table_overrides(self) -> None:
        reset()
        try:
            self.assertEqual(tr("page.home.title", "Início"), "Início")
            self.assertTrue(_LOCALE_EN.is_file(), "locale/en.json must ship with the package")
            self.assertGreater(load_translations(_LOCALE_EN), 0)
            self.assertEqual(tr("page.home.title", "Início"), "Home")
            self.assertEqual(tr("page.diagnostics.title", "Diagnóstico ROS"), "ROS diagnostics")
        finally:
            reset()


if __name__ == "__main__":
    unittest.main()
