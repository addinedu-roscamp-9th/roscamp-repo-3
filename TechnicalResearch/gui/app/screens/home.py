"""Home screen — top-level menu with Pinky and Schedule buttons."""

from typing import Callable

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from app.styles import menu_button_style


class HomeScreen(QWidget):
    """Two large menu buttons that navigate to the other screens."""

    def __init__(
        self,
        on_pinky: Callable[[], None],
        on_schedule: Callable[[], None],
    ):
        super().__init__()
        self._on_pinky = on_pinky
        self._on_schedule = on_schedule
        self._init_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        self.setStyleSheet(
            "background: qlineargradient(x1:0, y1:0, x2:1, y2:1,"
            " stop:0 #1a1a2e, stop:1 #16213e);"
        )

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        title = QLabel("가정용 로봇 제어")
        title.setStyleSheet(
            "font-size: 48px; font-weight: bold; color: white; margin-bottom: 20px;"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        subtitle = QLabel("원하는 서비스를 선택하세요")
        subtitle.setStyleSheet("font-size: 18px; color: #aaa; margin-bottom: 50px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)

        button_row = QHBoxLayout()
        button_row.setSpacing(30)
        button_row.addWidget(self._make_menu_button("Pinky", "blue", self._on_pinky))
        button_row.addWidget(
            self._make_menu_button("Schedule", "green", self._on_schedule)
        )
        layout.addLayout(button_row)

        layout.addStretch()
        self.setLayout(layout)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_menu_button(
        label: str, color: str, callback: Callable[[], None]
    ) -> QPushButton:
        btn = QPushButton(label)
        btn.setFixedSize(300, 300)
        btn.setStyleSheet(menu_button_style(color))
        btn.clicked.connect(callback)
        return btn
