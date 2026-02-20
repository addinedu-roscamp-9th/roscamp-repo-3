"""Navigation screen — robot status, room commands, and item actions."""

from __future__ import annotations

from typing import Callable

from PyQt5.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from app import client
from app.styles import (
    BACK_BUTTON_STYLE,
    GROUP_BOX_STYLE,
    ROBOT_BOX_STYLE,
    action_button_style,
)

# Rooms shown as quick-nav buttons
_PRESET_LOCATIONS = ["안방", "거실", "옷방", "화장실"]

# Items available in the "bring" dialog
_BRING_ITEMS = ["💊 약", "🍫 초코과자", "🍘 쌀과자"]


class NavigationScreen(QWidget):
    """Controls for moving the robot and fetching / placing items."""

    def __init__(self, on_back: Callable[[], None]):
        super().__init__()
        self._on_back = on_back
        # Labels kept as instance attrs so the battery poller can update them
        self.robot_status_labels: dict[str, dict[str, QLabel]] = {}
        self._init_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        layout = QVBoxLayout()
        layout.addLayout(self._make_header())
        layout.addLayout(self._make_status_panel())
        layout.addWidget(self._make_control_group())
        self.setLayout(layout)

    def _make_header(self) -> QHBoxLayout:
        header = QHBoxLayout()
        btn = QPushButton("뒤로가기")
        btn.setFixedSize(100, 35)
        btn.setStyleSheet(BACK_BUTTON_STYLE)
        btn.clicked.connect(self._on_back)
        header.addWidget(btn)
        header.addStretch()
        return header

    def _make_status_panel(self) -> QHBoxLayout:
        panel = QHBoxLayout()
        for name in ("Pinky1", "Pinky2"):
            box = QWidget()
            box.setStyleSheet(ROBOT_BOX_STYLE)
            box_layout = QVBoxLayout()

            name_label = QLabel(f"🤖 {name}")
            name_label.setStyleSheet(
                "font-size: 13px; font-weight: bold; color: #00FF00; border: none;"
            )
            box_layout.addWidget(name_label)

            battery_label = QLabel("🔋 배터리: -- %")
            battery_label.setStyleSheet("font-size: 11px; color: white; border: none;")
            box_layout.addWidget(battery_label)

            task_label = QLabel("📋 예약 작업: 없음")
            task_label.setStyleSheet("font-size: 11px; color: #aaa; border: none;")
            box_layout.addWidget(task_label)

            box.setLayout(box_layout)
            panel.addWidget(box)

            self.robot_status_labels[name] = {
                "battery": battery_label,
                "task": task_label,
            }
        return panel

    def _make_control_group(self) -> QGroupBox:
        group = QGroupBox("로봇 제어")
        group.setStyleSheet(GROUP_BOX_STYLE)
        layout = QVBoxLayout()

        # Room-navigation row
        room_row = QHBoxLayout()
        for room in _PRESET_LOCATIONS:
            btn = QPushButton(f"{room}")
            btn.setStyleSheet(action_button_style("blue"))
            btn.clicked.connect(lambda checked, r=room: self._send_move(r))
            room_row.addWidget(btn)
        layout.addLayout(room_row)

        # Special-action row
        special_row = QHBoxLayout()
        special_row.addWidget(
            self._make_action_btn("📦 물건 가져오기", "green", self._handle_bring)
        )
        special_row.addWidget(
            self._make_action_btn("📍 물건 갖다놓기", "orange", self._handle_put)
        )
        special_row.addWidget(
            self._make_action_btn("⏹️ 긴급 정지", "red", self._send_stop)
        )
        layout.addLayout(special_row)

        group.setLayout(layout)
        return group

    # ------------------------------------------------------------------
    # Public API — called by the battery poller in MainWindow
    # ------------------------------------------------------------------

    def update_battery(self, robot_name: str, percentage: float) -> None:
        """Refresh the battery label for *robot_name*."""
        if robot_name in self.robot_status_labels:
            self.robot_status_labels[robot_name]["battery"].setText(
                f"🔋 배터리: {percentage:.1f}%"
            )

    # ------------------------------------------------------------------
    # Server actions
    # ------------------------------------------------------------------

    def _send_move(self, destination: str) -> None:
        try:
            resp = client.move(destination)
            if resp.status_code == 200:
                QMessageBox.information(self, "성공", f"{destination}으로 이동")
            else:
                QMessageBox.warning(self, "오류", "서버 응답 오류")
        except Exception as exc:
            QMessageBox.critical(self, "연결 오류", f"서버 연결 실패: {exc}")

    def _send_stop(self) -> None:
        try:
            resp = client.stop()
            if resp.status_code == 200:
                QMessageBox.information(self, "정지", "로봇이 정지되었습니다")
            else:
                QMessageBox.warning(self, "오류", "서버 응답 오류")
        except Exception as exc:
            QMessageBox.critical(self, "연결 오류", f"서버 연결 실패: {exc}")

    def _handle_bring(self) -> None:
        item, ok = QInputDialog.getItem(
            self, "물건 선택", "가져올 물건:", _BRING_ITEMS, 0, False
        )
        if not (ok and item):
            return

        room, ok2 = QInputDialog.getItem(
            self, "방 선택", "가져다 줄 방:", _PRESET_LOCATIONS, 0, False
        )
        if not (ok2 and room):
            return

        try:
            resp = client.bring_item(item, room)
            if resp.status_code == 200:
                QMessageBox.information(
                    self, "작업 시작", f"{item}를 {room}으로 가져옵니다"
                )
        except Exception as exc:
            QMessageBox.critical(self, "오류", f"서버 연결 실패: {exc}")

    def _handle_put(self) -> None:
        room, ok = QInputDialog.getItem(
            self, "현재 위치", "현재 방:", _PRESET_LOCATIONS, 0, False
        )
        if not (ok and room):
            return

        ret = QMessageBox.question(
            self,
            "확인",
            f"{room}에서 Pick Up Zone으로\n물건을 가져다 놓으시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if ret != QMessageBox.Yes:
            return

        try:
            resp = client.put_item(room)
            if resp.status_code == 200:
                QMessageBox.information(self, "작업 시작", "Pick Up Zone으로 이동")
        except Exception as exc:
            QMessageBox.critical(self, "오류", f"서버 연결 실패: {exc}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_action_btn(
        label: str, color: str, callback: Callable[[], None]
    ) -> QPushButton:
        btn = QPushButton(label)
        btn.setStyleSheet(action_button_style(color))
        btn.clicked.connect(callback)
        return btn
