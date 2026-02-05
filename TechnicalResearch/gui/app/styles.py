"""Shared stylesheet constants and theme helpers."""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPalette

# ---------------------------------------------------------------------------
# Color theme — (base, hover) pairs
# ---------------------------------------------------------------------------
COLORS = {
    "blue": ("#2196F3", "#1976D2"),
    "green": ("#4CAF50", "#388E3C"),
    "orange": ("#FF9800", "#F57C00"),
    "red": ("#f44336", "#d32f2f"),
    "teal": ("#2ecc71", "#27ae60"),
    "sky": ("#3498db", "#2980b9"),
    "coral": ("#e74c3c", "#c0392b"),
    "gray": ("#555555", "#666666"),
}


# ---------------------------------------------------------------------------
# Parameterised style generators
# ---------------------------------------------------------------------------


def menu_button_style(color_name: str) -> str:
    """Large 300x300 home-screen menu button."""
    base, hover = COLORS[color_name]
    return f"""
        QPushButton {{
            background: {base}; color: white;
            font-size: 32px; font-weight: bold;
            border-radius: 20px; border: none;
        }}
        QPushButton:hover {{ background: {hover}; }}
    """


def action_button_style(color_name: str) -> str:
    """Standard 60 px tall action button (room nav, bring, put, stop)."""
    base, hover = COLORS[color_name]
    return f"""
        QPushButton {{
            background: {base}; color: white; font-weight: bold;
            height: 60px; font-size: 16px; border-radius: 8px;
        }}
        QPushButton:hover {{ background: {hover}; }}
    """


def schedule_button_style(color_name: str) -> str:
    """Compact 40 px tall button used in the schedule panel."""
    base, hover = COLORS[color_name]
    return f"""
        QPushButton {{
            background: {base}; color: white; font-weight: bold;
            height: 40px; border-radius: 5px;
        }}
        QPushButton:hover {{ background: {hover}; }}
    """


# ---------------------------------------------------------------------------
# Static stylesheet constants
# ---------------------------------------------------------------------------

BACK_BUTTON_STYLE = """
    QPushButton {
        background: #555; color: white; font-weight: bold;
        height: 35px; border-radius: 5px; font-size: 13px;
    }
    QPushButton:hover { background: #666; }
"""

GROUP_BOX_STYLE = """
    QGroupBox {
        color: white; font-size: 16px; font-weight: bold;
        border: 2px solid #444; border-radius: 10px;
        margin-top: 10px; padding-top: 20px;
    }
"""

ROBOT_BOX_STYLE = """
    QWidget {
        background: #2d2d2d; border-radius: 10px;
        border: 2px solid #555; padding: 8px;
    }
"""

LIST_WIDGET_STYLE = """
    QListWidget {
        background: #2d2d2d; color: white; font-size: 14px;
        border: 2px solid #444; border-radius: 5px;
    }
    QListWidget::item { padding: 10px; }
    QListWidget::item:selected { background: #0078d7; }
"""


# ---------------------------------------------------------------------------
# Application-wide dark palette
# ---------------------------------------------------------------------------


def dark_palette() -> QPalette:
    """Return a QPalette that gives the app its dark theme."""
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(30, 30, 46))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    return palette
