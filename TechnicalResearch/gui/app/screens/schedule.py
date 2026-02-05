"""Schedule screen — manage and run task presets."""

from __future__ import annotations

from typing import Callable

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from app import client
from app.styles import (
    BACK_BUTTON_STYLE,
    GROUP_BOX_STYLE,
    LIST_WIDGET_STYLE,
    schedule_button_style,
)


class ScheduleScreen(QWidget):
    """Add, run, and delete task presets."""

    def __init__(self, on_back: Callable[[], None]):
        super().__init__()
        self._on_back = on_back
        self._schedules: dict[str, list[str]] = {}
        self._preset_list: QListWidget  # assigned in _init_ui
        self._init_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        layout = QVBoxLayout()
        layout.addLayout(self._make_header())
        layout.addWidget(self._make_preset_group())
        self.setLayout(layout)

    def _make_header(self) -> QHBoxLayout:
        header = QHBoxLayout()

        btn_back = QPushButton("⬅️ 홈으로")
        btn_back.setStyleSheet(BACK_BUTTON_STYLE)
        btn_back.clicked.connect(self._on_back)
        header.addWidget(btn_back)
        header.addStretch()

        title = QLabel("작업 예약")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: white;")
        header.addWidget(title)
        header.addStretch()

        return header

    def _make_preset_group(self) -> QGroupBox:
        group = QGroupBox("작업 프리셋")
        group.setStyleSheet(GROUP_BOX_STYLE)
        layout = QVBoxLayout()

        self._preset_list = QListWidget()
        self._preset_list.setStyleSheet(LIST_WIDGET_STYLE)
        layout.addWidget(self._preset_list)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self._make_btn("▶️ 실행", "teal", self._run_preset))
        btn_row.addWidget(self._make_btn("➕ 추가", "sky", self._add_preset))
        btn_row.addWidget(self._make_btn("🗑️ 삭제", "coral", self._delete_preset))
        layout.addLayout(btn_row)

        group.setLayout(layout)
        return group

    # ------------------------------------------------------------------
    # Schedule actions
    # ------------------------------------------------------------------

    def _add_preset(self) -> None:
        name, ok = QInputDialog.getText(
            self, "스케줄 이름", "스케줄 이름을 입력하세요:"
        )
        if ok and name:
            self._schedules[name] = ["안방", "거실", "화장실"]
            self._refresh_list()

    def _run_preset(self) -> None:
        name = self._selected_name()
        if name is None:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return

        try:
            resp = client.execute_schedule(name, self._schedules[name])
            if resp.status_code == 200:
                QMessageBox.information(self, "실행", f"'{name}' 작업 시작!")
        except Exception as exc:
            QMessageBox.critical(self, "오류", f"서버 연결 실패: {exc}")

    def _delete_preset(self) -> None:
        name = self._selected_name()
        if name is None:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return

        ret = QMessageBox.question(
            self,
            "삭제 확인",
            f"'{name}' 작업을 삭제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if ret == QMessageBox.Yes:
            del self._schedules[name]
            self._refresh_list()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _refresh_list(self) -> None:
        """Rebuild the QListWidget to match _schedules."""
        self._preset_list.clear()
        for name, tasks in self._schedules.items():
            summary = " → ".join(tasks[:3])
            item = QListWidgetItem(f"📋 {name}\n   {summary}")
            item.setData(Qt.UserRole, name)  # store name for reliable retrieval
            self._preset_list.addItem(item)

    def _selected_name(self) -> str | None:
        """Return the schedule name of the current selection, or None."""
        item = self._preset_list.currentItem()
        return item.data(Qt.UserRole) if item else None

    @staticmethod
    def _make_btn(label: str, color: str, callback: Callable[[], None]) -> QPushButton:
        btn = QPushButton(label)
        btn.setStyleSheet(schedule_button_style(color))
        btn.clicked.connect(callback)
        return btn
