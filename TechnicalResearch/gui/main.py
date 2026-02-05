"""
Pinky Robot GUI - 버튼 클릭만 하는 단순 GUI
서버에 HTTP 요청만 보냄
"""

import json
import sys

import requests
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

with open("./config.json", encoding="UTF-8") as file:
    config = json.load(file)

SERVER_URL = config["server_url"]


class PinkyRobotGUI(QWidget):
    """Pinky Robot 제어 GUI"""

    def __init__(self):
        super().__init__()
        self.sever_url = SERVER_URL

        # 로봇 상태
        self.robot_batteries = {
            "Pinky1": {"percentage": 0.0, "voltage": 0.0},
            "Pinky2": {"percentage": 0.0, "voltage": 0.0},
        }

        self.robot_tasks = {"Pinky1": None, "Pinky2": None}

        self.current_robot = "Pinky1"

        # 위치 프리셋
        self.preset_locations = ["안방", "거실", "옷방", "화장실"]

        # 저장된 스케줄
        self.saved_schedules = {}

        # 배터리 폴링 타이머
        self.battery_timer = QTimer()
        self.battery_timer.timeout.connect(self.update_battery_status)
        self.battery_timer.start(2000)

        self.init_ui()

    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Pinky Robot Control")
        self.resize(1400, 900)

        self.stack = QStackedWidget()

        self.stack.addWidget(self.create_home_screen())
        self.stack.addWidget(self.create_navigation_screen())
        self.stack.addWidget(self.create_schedule_screen())

        layout = QVBoxLayout()
        layout.addWidget(self.stack)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

    def create_home_screen(self):
        """홈 화면"""
        widget = QWidget()
        widget.setStyleSheet(
            "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1a1a2e, stop:1 #16213e);"
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

        button_layout = QHBoxLayout()
        button_layout.setSpacing(30)

        btn_nav = self.create_menu_button(
            "Pinky", "#2196F3", lambda: self.stack.setCurrentIndex(1)
        )
        button_layout.addWidget(btn_nav)

        btn_schedule = self.create_menu_button(
            "Schedule", "#4CAF50", lambda: self.stack.setCurrentIndex(2)
        )
        button_layout.addWidget(btn_schedule)

        layout.addLayout(button_layout)
        layout.addStretch()

        widget.setLayout(layout)
        return widget

    def create_menu_button(self, title, color, callback):
        """메뉴 버튼"""
        btn = QPushButton(title)
        btn.setFixedSize(300, 300)
        btn.setStyleSheet(f"""
            QPushButton {{
                background: {color}; color: white;
                font-size: 32px; font-weight: bold;
                border-radius: 20px; border: none;
            }}
            QPushButton:hover {{ background: {self.darken_color(color)}; }}
        """)
        btn.clicked.connect(callback)
        return btn

    def darken_color(self, hex_color):
        """색상 어둡게"""
        return hex_color.replace("2196F3", "1976D2").replace("4CAF50", "388E3C")

    def create_navigation_screen(self):
        """주행로봇 화면"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 헤더
        header = QHBoxLayout()
        btn_back = QPushButton("⬅️ 홈으로")
        btn_back.setFixedSize(100, 35)
        btn_back.setStyleSheet("""
            QPushButton {
                background: #555; color: white; font-weight: bold;
                border-radius: 5px; font-size: 12px;
            }
            QPushButton:hover { background: #666; }
        """)
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        header.addWidget(btn_back)
        header.addStretch()
        layout.addLayout(header)

        # 로봇 상태 패널
        status_panel = QHBoxLayout()
        self.robot_status_widgets = {}

        for robot_name in ["Pinky1", "Pinky2"]:
            robot_box = QWidget()
            robot_box.setStyleSheet("""
                QWidget {
                    background: #2d2d2d; border-radius: 10px;
                    border: 2px solid #555; padding: 8px;
                }
            """)
            robot_box_layout = QVBoxLayout()

            name_label = QLabel(f"🤖 {robot_name}")
            name_label.setStyleSheet(
                "font-size: 13px; font-weight: bold; color: #00FF00; border: none;"
            )
            robot_box_layout.addWidget(name_label)

            battery_label = QLabel("🔋 배터리: -- %")
            battery_label.setStyleSheet("font-size: 11px; color: white; border: none;")
            robot_box_layout.addWidget(battery_label)

            task_label = QLabel("📋 예약 작업: 없음")
            task_label.setStyleSheet("font-size: 11px; color: #aaa; border: none;")
            robot_box_layout.addWidget(task_label)

            robot_box.setLayout(robot_box_layout)
            status_panel.addWidget(robot_box)

            self.robot_status_widgets[robot_name] = {
                "battery": battery_label,
                "task": task_label,
            }

        layout.addLayout(status_panel)

        # 제어 버튼들
        control_group = QGroupBox("로봇 제어")
        control_group.setStyleSheet("""
            QGroupBox {
                color: white; font-size: 16px; font-weight: bold;
                border: 2px solid #444; border-radius: 10px;
                margin-top: 10px; padding-top: 20px;
            }
        """)
        control_layout = QVBoxLayout()

        # 방 이동 버튼들
        room_layout = QHBoxLayout()
        for room in self.preset_locations:
            btn = QPushButton(f"📍 {room}")
            btn.setStyleSheet("""
                QPushButton {
                    background: #2196F3; color: white; font-weight: bold;
                    height: 60px; font-size: 16px; border-radius: 8px;
                }
                QPushButton:hover { background: #1976D2; }
            """)
            btn.clicked.connect(lambda checked, r=room: self.send_move_command(r))
            room_layout.addWidget(btn)

        control_layout.addLayout(room_layout)

        # 특수 기능 버튼들
        special_layout = QHBoxLayout()

        btn_bring = QPushButton("📦 물건 가져오기")
        btn_bring.setStyleSheet("""
            QPushButton {
                background: #4CAF50; color: white; font-weight: bold;
                height: 60px; font-size: 16px; border-radius: 8px;
            }
            QPushButton:hover { background: #388E3C; }
        """)
        btn_bring.clicked.connect(self.handle_bring_item)
        special_layout.addWidget(btn_bring)

        btn_put = QPushButton("📍 물건 갖다놓기")
        btn_put.setStyleSheet("""
            QPushButton {
                background: #FF9800; color: white; font-weight: bold;
                height: 60px; font-size: 16px; border-radius: 8px;
            }
            QPushButton:hover { background: #F57C00; }
        """)
        btn_put.clicked.connect(self.handle_put_item)
        special_layout.addWidget(btn_put)

        btn_stop = QPushButton("⏹️ 긴급 정지")
        btn_stop.setStyleSheet("""
            QPushButton {
                background: #f44336; color: white; font-weight: bold;
                height: 60px; font-size: 16px; border-radius: 8px;
            }
            QPushButton:hover { background: #d32f2f; }
        """)
        btn_stop.clicked.connect(self.send_stop_command)
        special_layout.addWidget(btn_stop)

        control_layout.addLayout(special_layout)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        widget.setLayout(layout)
        return widget

    def create_schedule_screen(self):
        """작업 예약 화면"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 헤더
        header = QHBoxLayout()
        btn_back = QPushButton("⬅️ 홈으로")
        btn_back.setStyleSheet("""
            QPushButton {
                background: #555; color: white; font-weight: bold;
                height: 40px; border-radius: 5px; font-size: 14px;
            }
            QPushButton:hover { background: #666; }
        """)
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        header.addWidget(btn_back)
        header.addStretch()

        title = QLabel("작업 예약")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: white;")
        header.addWidget(title)
        header.addStretch()
        layout.addLayout(header)

        # 작업 목록
        list_group = QGroupBox("작업 프리셋")
        list_group.setStyleSheet("""
            QGroupBox {
                color: white; font-size: 16px; font-weight: bold;
                border: 2px solid #444; border-radius: 10px;
                margin-top: 10px; padding-top: 20px;
            }
        """)
        list_layout = QVBoxLayout()

        self.preset_list = QListWidget()
        self.preset_list.setStyleSheet("""
            QListWidget {
                background: #2d2d2d; color: white; font-size: 14px;
                border: 2px solid #444; border-radius: 5px;
            }
            QListWidget::item { padding: 10px; }
            QListWidget::item:selected { background: #0078d7; }
        """)
        list_layout.addWidget(self.preset_list)

        # 버튼들
        preset_btn_layout = QHBoxLayout()

        btn_run = QPushButton("▶️ 실행")
        btn_run.setStyleSheet(
            "background: #2ecc71; color: white; font-weight: bold; height: 40px;"
        )
        btn_run.clicked.connect(self.run_selected_preset)
        preset_btn_layout.addWidget(btn_run)

        btn_add = QPushButton("➕ 추가")
        btn_add.setStyleSheet(
            "background: #3498db; color: white; font-weight: bold; height: 40px;"
        )
        btn_add.clicked.connect(self.add_new_schedule)
        preset_btn_layout.addWidget(btn_add)

        btn_delete = QPushButton("🗑️ 삭제")
        btn_delete.setStyleSheet(
            "background: #e74c3c; color: white; font-weight: bold; height: 40px;"
        )
        btn_delete.clicked.connect(self.delete_selected_preset)
        preset_btn_layout.addWidget(btn_delete)

        list_layout.addLayout(preset_btn_layout)
        list_group.setLayout(list_layout)
        layout.addWidget(list_group)

        widget.setLayout(layout)
        return widget

    # ==================== 서버 통신 메서드 ====================

    def send_move_command(self, destination):
        """이동 명령 전송"""
        try:
            response = requests.post(
                f"{self.SERVER_URL}/gui",
                json={
                    "robot_id": 1,
                    "status": "moving",
                    "command": "move",
                    "destination": destination,
                },
            )

            if response.status_code == 200:
                result = response.json()
                QMessageBox.information(self, "성공", f"{destination}으로 이동 시작!")
            else:
                QMessageBox.warning(self, "오류", "서버 응답 오류")
        except Exception as e:
            QMessageBox.critical(self, "연결 오류", f"서버 연결 실패: {str(e)}")

    def send_stop_command(self):
        """긴급 정지 명령"""
        try:
            response = requests.post(
                f"{self.SERVER_URL}/gui",
                json={"robot_id": 1, "status": "stopped", "command": "stop"},
            )

            if response.status_code == 200:
                QMessageBox.information(self, "정지", "로봇이 정지되었습니다")
            else:
                QMessageBox.warning(self, "오류", "서버 응답 오류")
        except Exception as e:
            QMessageBox.critical(self, "연결 오류", f"서버 연결 실패: {str(e)}")

    def handle_bring_item(self):
        """물건 가져오기"""
        items = ["💊 약", "🍫 초코과자", "🍘 쌀과자"]
        item, ok1 = QInputDialog.getItem(
            self, "물건 선택", "가져올 물건:", items, 0, False
        )

        if ok1 and item:
            room, ok2 = QInputDialog.getItem(
                self, "방 선택", "가져다 줄 방:", self.preset_locations, 0, False
            )

            if ok2 and room:
                try:
                    response = requests.post(
                        f"{self.SERVER_URL}/gui",
                        json={
                            "robot_id": 1,
                            "status": "bringing_item",
                            "command": "bring_item",
                            "destination": room,
                            "item": item,
                        },
                    )

                    if response.status_code == 200:
                        QMessageBox.information(
                            self, "작업 시작", f"{item}를 {room}으로 가져옵니다!"
                        )
                except Exception as e:
                    QMessageBox.critical(self, "오류", f"서버 연결 실패: {str(e)}")

    def handle_put_item(self):
        """물건 갖다놓기"""
        room, ok = QInputDialog.getItem(
            self, "현재 위치", "현재 방:", self.preset_locations, 0, False
        )

        if ok and room:
            ret = QMessageBox.question(
                self,
                "확인",
                f"{room}에서 Pick Up Zone으로\n물건을 가져다 놓으시겠습니까?",
                QMessageBox.Yes | QMessageBox.No,
            )

            if ret == QMessageBox.Yes:
                try:
                    response = requests.post(
                        f"{self.SERVER_URL}/gui",
                        json={
                            "robot_id": 1,
                            "status": "putting_item",
                            "command": "put_item",
                            "destination": "pickup_zone",
                            "from_room": room,
                        },
                    )

                    if response.status_code == 200:
                        QMessageBox.information(
                            self, "작업 시작", "Pick Up Zone으로 이동합니다!"
                        )
                except Exception as e:
                    QMessageBox.critical(self, "오류", f"서버 연결 실패: {str(e)}")

    def add_new_schedule(self):
        """새 스케줄 추가"""
        name, ok = QInputDialog.getText(
            self, "스케줄 이름", "스케줄 이름을 입력하세요:"
        )

        if ok and name:
            # 간단한 스케줄 생성 (실제로는 더 복잡한 다이얼로그 필요)
            self.saved_schedules[name] = ["안방", "거실", "화장실"]
            self.update_preset_list()

    def update_preset_list(self):
        """프리셋 목록 업데이트"""
        self.preset_list.clear()
        for name, tasks in self.saved_schedules.items():
            task_summary = " → ".join(tasks[:3])
            item_text = f"📋 {name}\n   {task_summary}"
            self.preset_list.addItem(QListWidgetItem(item_text))

    def run_selected_preset(self):
        """선택된 프리셋 실행"""
        current_item = self.preset_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return

        preset_name = list(self.saved_schedules.keys())[self.preset_list.currentRow()]
        tasks = self.saved_schedules[preset_name]

        try:
            response = requests.post(
                f"{self.SERVER_URL}/gui",
                json={
                    "robot_id": 1,
                    "status": "executing_schedule",
                    "command": "execute_schedule",
                    "schedule_name": preset_name,
                    "tasks": tasks,
                },
            )

            if response.status_code == 200:
                QMessageBox.information(self, "실행", f"'{preset_name}' 작업 시작!")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"서버 연결 실패: {str(e)}")

    def delete_selected_preset(self):
        """프리셋 삭제"""
        current_item = self.preset_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return

        preset_name = list(self.saved_schedules.keys())[self.preset_list.currentRow()]

        ret = QMessageBox.question(
            self,
            "삭제 확인",
            f"'{preset_name}' 작업을 삭제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
        )

        if ret == QMessageBox.Yes:
            del self.saved_schedules[preset_name]
            self.update_preset_list()

    def update_battery_status(self):
        """배터리 상태 업데이트 (서버에서 조회)"""
        try:
            response = requests.get(f"{self.SERVER_URL}/pinky/status/Pinky1")
            if response.status_code == 200:
                data = response.json()

                battery = data.get("battery", 0)
                self.robot_batteries["Pinky1"]["percentage"] = battery

                # UI 업데이트
                if "Pinky1" in self.robot_status_widgets:
                    self.robot_status_widgets["Pinky1"]["battery"].setText(
                        f"🔋 배터리: {battery:.1f}%"
                    )
        except:
            pass  # 서버 연결 실패 시 조용히 무시


def main():
    """메인 함수"""
    app = QApplication(sys.argv)

    # 다크 모드
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(30, 30, 46))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    app.setPalette(palette)

    gui = PinkyRobotGUI()
    gui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
