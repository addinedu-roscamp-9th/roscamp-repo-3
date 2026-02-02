import math
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, 
    QPushButton, QMessageBox, QListWidget, QListWidgetItem,
    QProgressBar, QStackedWidget, QInputDialog, QDialog
)
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QPalette, QColor

from .simple_map_widget import SimpleMapWidget
from .task_builder_dialog import TaskBuilderDialog


class MultiScreenRobotGUI(QWidget):
    """멀티 스크린 메인 GUI"""
    
    def __init__(self, node, battery_monitor):
        super().__init__()
        self.node = node
        self.battery_monitor = battery_monitor
        
        # 현재 선택된 로봇
        self.current_robot = "Pinky1"
        
        # 배터리 (로봇별)
        self.robot_batteries = {
            "Pinky1": {"percentage": 0.0, "voltage": 0.0},
            "Pinky2": {"percentage": 0.0, "voltage": 0.0}
        }
        
        # 작업 예약 (로봇별)
        self.robot_tasks = {
            "Pinky1": None,
            "Pinky2": None
        }
        
        # 위치 프리셋
        self.preset_locations = [
            ("안방", 1.27, -0.439, 0.0),
            ("거실", -1.00, -0.11, math.pi/2),
            ("옷방", -0.28, 0.219, math.pi),
            ("화장실", 0.18, -0.212, -math.pi/2),
        ]
        
        # 작업 스케줄
        self.saved_schedules = {}
        
        self.init_ui()
        
        # 배터리 모니터 시작
        self.battery_monitor.battery_updated.connect(self.on_battery_updated)
        self.battery_monitor.start()
    
    @pyqtSlot(float, float)
    def on_battery_updated(self, percentage, voltage):
        """배터리 정보 업데이트"""
        # 현재 선택된 로봇의 배터리 업데이트
        self.robot_batteries[self.current_robot]["percentage"] = percentage
        self.robot_batteries[self.current_robot]["voltage"] = voltage
        
        # 맵 위젯 업데이트
        if hasattr(self, 'map_widget'):
            self.map_widget.update_battery(percentage, voltage)
        
        # 로봇 상태 위젯 업데이트
        if hasattr(self, 'robot_status_widgets'):
            self.update_robot_status_display()
    
    def update_robot_status_display(self):
        """로봇 상태 디스플레이 업데이트"""
        for robot_name in ["Pinky1", "Pinky2"]:
            if robot_name in self.robot_status_widgets:
                battery_data = self.robot_batteries[robot_name]
                percentage = battery_data["percentage"]
                voltage = battery_data["voltage"]
                
                # 배터리 업데이트
                self.robot_status_widgets[robot_name]['battery'].setText(
                    f"🔋 배터리: {percentage:.1f}% ({voltage:.2f}V)"
                )
                
                # 작업 정보 업데이트
                task_info = self.robot_tasks.get(robot_name)
                if task_info:
                    self.robot_status_widgets[robot_name]['task'].setText(
                        f"📋 예약: {task_info}"
                    )
                    self.robot_status_widgets[robot_name]['task'].setStyleSheet(
                        "font-size: 12px; color: #00FF00; border: none;"
                    )
                else:
                    self.robot_status_widgets[robot_name]['task'].setText("📋 예약 작업: 없음")
                    self.robot_status_widgets[robot_name]['task'].setStyleSheet(
                        "font-size: 12px; color: #aaa; border: none;"
                    )
    
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Pinky Robot System")
        self.resize(1400, 900)
        
        # 스택 위젯으로 화면 전환
        self.stack = QStackedWidget()
        
        # 1. 홈 화면
        home_widget = self.create_home_screen()
        self.stack.addWidget(home_widget)
        
        # 2. 주행로봇 화면
        nav_widget = self.create_navigation_screen()
        self.stack.addWidget(nav_widget)
        
        # 3. 로봇팔 화면
        arm_widget = self.create_arm_screen()
        self.stack.addWidget(arm_widget)
        
        # 4. 작업 예약 화면
        schedule_widget = self.create_schedule_screen()
        self.stack.addWidget(schedule_widget)
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.stack)
        main_layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(main_layout)
    
    def create_home_screen(self):
        """홈 화면 생성"""
        widget = QWidget()
        widget.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1a1a2e, stop:1 #16213e);")
        
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        
        # 타이틀
        title = QLabel("가정용 로봇 제어")
        title.setStyleSheet("""
            font-size: 48px; font-weight: bold; color: white;
            margin-bottom: 20px;
        """)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        subtitle = QLabel("원하는 서비스를 선택하세요")
        subtitle.setStyleSheet("font-size: 18px; color: #aaa; margin-bottom: 50px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        
        # 버튼 그리드
        button_layout = QHBoxLayout()
        button_layout.setSpacing(30)
        
        # 주행로봇 버튼
        btn_nav = self.create_menu_button(
            "Pinky", 
            "실시간 맵 및 로봇 제어",
            "#2196F3",
            lambda: self.stack.setCurrentIndex(1)
        )
        button_layout.addWidget(btn_nav)
        
        # 로봇팔 버튼
        btn_arm = self.create_menu_button(
            "JetcoBot", 
            "로봇팔 제어 및 작업",
            "#9C27B0",
            lambda: self.stack.setCurrentIndex(2)
        )
        button_layout.addWidget(btn_arm)
        
        # 작업 예약 버튼
        btn_schedule = self.create_menu_button(
            "Schedule", 
            "스케줄 관리 및 예약",
            "#4CAF50",
            lambda: self.stack.setCurrentIndex(3)
        )
        button_layout.addWidget(btn_schedule)
        
        layout.addLayout(button_layout)
        layout.addStretch()
        
        widget.setLayout(layout)
        return widget
    
    def create_menu_button(self, title, subtitle, color, callback):
        """메뉴 버튼 생성"""
        btn = QPushButton(title)
        btn.setFixedSize(300, 300)
        btn.setStyleSheet(f"""
            QPushButton {{
                background: {color};
                color: white;
                font-size: 32px;
                font-weight: bold;
                border-radius: 20px;
                border: none;
            }}
            QPushButton:hover {{
                background: {self.darken_color(color)};
            }}
        """)
        btn.clicked.connect(callback)
        return btn
    
    def darken_color(self, hex_color):
        """색상 어둡게"""
        return hex_color.replace("2196F3", "1976D2").replace("9C27B0", "7B1FA2").replace("4CAF50", "388E3C")
    
    def create_navigation_screen(self):
        """주행로봇 화면 생성"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # 헤더
        header = QHBoxLayout()
        
        btn_back = QPushButton("⬅️ 홈으로")
        btn_back.setFixedSize(100, 35)
        btn_back.setStyleSheet("""
            QPushButton {
                background: #555; color: white; font-weight: bold;
                border-radius: 5px; font-size: 12px; padding: 0 10px;
            }
            QPushButton:hover { background: #666; }
        """)
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        header.addWidget(btn_back)
        
        header.addStretch()
        
        layout.addLayout(header)
        
        # 로봇 상태 표시 패널 (가로 배치)
        status_panel = QHBoxLayout()
        status_panel.setSpacing(10)
        
        self.robot_status_widgets = {}
        for robot_name in ["Pinky1", "Pinky2"]:
            # 로봇별 상태 박스
            robot_box = QWidget()
            robot_box.setStyleSheet("""
                QWidget {
                    background: #2d2d2d; border-radius: 10px;
                    border: 2px solid #555; padding: 8px;
                }
            """)
            robot_box_layout = QVBoxLayout()
            robot_box_layout.setContentsMargins(8, 8, 8, 8)
            robot_box_layout.setSpacing(5)
            
            # 로봇 이름
            name_label = QLabel(f"🤖 {robot_name}")
            name_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #00FF00; border: none;")
            robot_box_layout.addWidget(name_label)
            
            # 배터리 표시
            battery_label = QLabel("🔋 배터리: -- %")
            battery_label.setStyleSheet("font-size: 11px; color: white; border: none;")
            robot_box_layout.addWidget(battery_label)
            
            # 작업 예약 표시
            task_label = QLabel("📋 예약 작업: 없음")
            task_label.setStyleSheet("font-size: 11px; color: #aaa; border: none;")
            task_label.setWordWrap(True)
            robot_box_layout.addWidget(task_label)
            
            robot_box.setLayout(robot_box_layout)
            status_panel.addWidget(robot_box)
            
            self.robot_status_widgets[robot_name] = {
                'battery': battery_label,
                'task': task_label
            }
        
        layout.addLayout(status_panel)
        
        # 맵 위젯
        self.map_widget = SimpleMapWidget()
        layout.addWidget(self.map_widget)
        
        widget.setLayout(layout)
        return widget
    
    def create_arm_screen(self):
        """로봇팔 화면 생성"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        btn_back = QPushButton("⬅️ 홈으로")
        btn_back.setStyleSheet("""
            QPushButton {
                background: #555; color: white; font-weight: bold;
                height: 40px; border-radius: 5px; font-size: 14px; padding: 0 20px;
            }
            QPushButton:hover { background: #666; }
        """)
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        layout.addWidget(btn_back, alignment=Qt.AlignLeft)
        
        # 준비중 메시지
        message = QLabel("🦾 로봇팔 기능은 준비 중입니다")
        message.setStyleSheet("font-size: 32px; color: white;")
        message.setAlignment(Qt.AlignCenter)
        layout.addWidget(message)
        
        widget.setLayout(layout)
        return widget
    
    def create_schedule_screen(self):
        """작업 예약 화면 생성"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # 헤더
        header = QHBoxLayout()
        
        btn_back = QPushButton("⬅️ 홈으로")
        btn_back.setStyleSheet("""
            QPushButton {
                background: #555; color: white; font-weight: bold;
                height: 40px; border-radius: 5px; font-size: 14px; padding: 0 20px;
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
        header.addWidget(QLabel(""))  # 간격 맞추기
        
        layout.addLayout(header)
        
        # 작업 목록
        list_group = QGroupBox("작업 프리셋")
        list_group.setStyleSheet("""
            QGroupBox {
                color: white; font-size: 16px; font-weight: bold;
                border: 2px solid #444; border-radius: 10px;
                margin-top: 10px; padding-top: 20px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
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
        btn_run.setStyleSheet("background: #2ecc71; color: white; font-weight: bold; height: 40px; font-size: 14px;")
        btn_run.clicked.connect(self.run_selected_preset)
        preset_btn_layout.addWidget(btn_run)
        
        btn_edit = QPushButton("✏️ 수정")
        btn_edit.setStyleSheet("background: #f39c12; color: white; font-weight: bold; height: 40px; font-size: 14px;")
        btn_edit.clicked.connect(self.edit_selected_preset)
        preset_btn_layout.addWidget(btn_edit)
        
        btn_delete = QPushButton("🗑️ 삭제")
        btn_delete.setStyleSheet("background: #e74c3c; color: white; font-weight: bold; height: 40px; font-size: 14px;")
        btn_delete.clicked.connect(self.delete_selected_preset)
        preset_btn_layout.addWidget(btn_delete)
        
        list_layout.addLayout(preset_btn_layout)
        
        list_group.setLayout(list_layout)
        layout.addWidget(list_group)
        
        # 작업 추가 버튼
        btn_add_task = QPushButton("➕ 새 작업 추가")
        btn_add_task.setStyleSheet("""
            background: #3498db; color: white; font-weight: bold;
            height: 60px; border-radius: 10px; font-size: 18px;
        """)
        btn_add_task.clicked.connect(self.add_new_task)
        layout.addWidget(btn_add_task)
        
        widget.setLayout(layout)
        return widget
    
    # 작업 예약 관련 메서드들
    def add_new_task(self):
        dialog = TaskBuilderDialog(self, self.preset_locations)
        if dialog.exec_() == QDialog.Accepted:
            if dialog.tasks:
                title, ok = QInputDialog.getText(self, "작업 이름", "작업 스케줄 이름을 입력하세요:")
                if ok and title:
                    self.saved_schedules[title] = dialog.tasks
                    self.update_preset_list()
    
    def update_preset_list(self):
        self.preset_list.clear()
        for name, tasks in self.saved_schedules.items():
            task_summary = " → ".join([f"{t[1]}" if t[0] == "move" else "📦" for t in tasks[:3]])
            if len(tasks) > 3:
                task_summary += "..."
            item_text = f"📋 {name}\n   {task_summary}"
            self.preset_list.addItem(QListWidgetItem(item_text))
    
    def run_selected_preset(self):
        current_item = self.preset_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return
        
        preset_name = list(self.saved_schedules.keys())[self.preset_list.currentRow()]
        tasks = self.saved_schedules[preset_name]
        
        task_text = "\n".join([f"{i+1}. {t[1]}" if t[0] == "move" else f"{i+1}. 📦 {t[1]}" for i, t in enumerate(tasks)])
        
        ret = QMessageBox.question(self, f"작업 실행: {preset_name}", 
                                   f"다음 작업을 실행하시겠습니까?\n\n{task_text}", 
                                   QMessageBox.Yes | QMessageBox.No)
        
        if ret == QMessageBox.Yes:
            # 작업 요약 생성
            task_summary = " → ".join([t[1] for t in tasks[:2]])
            if len(tasks) > 2:
                task_summary += f" 외 {len(tasks)-2}개"
            
            # 현재 로봇에 작업 표시
            self.robot_tasks[self.current_robot] = task_summary
            self.update_robot_status_display()
            
            QMessageBox.information(self, "완료", f"'{preset_name}' 작업 시작!")
            
            # 실제 작업 실행 로직
            for task_type, task_name in tasks:
                if task_type == "move":
                    location = next((loc for loc in self.preset_locations if loc[0] == task_name), None)
                    if location:
                        _, x, y, yaw = location
                        self.node.navigate_to_pose(x, y, yaw)
    
    def edit_selected_preset(self):
        current_item = self.preset_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return
        
        preset_name = list(self.saved_schedules.keys())[self.preset_list.currentRow()]
        existing_tasks = self.saved_schedules[preset_name]
        
        dialog = TaskBuilderDialog(self, self.preset_locations, existing_tasks)
        if dialog.exec_() == QDialog.Accepted:
            if dialog.tasks:
                self.saved_schedules[preset_name] = dialog.tasks
                self.update_preset_list()
    
    def delete_selected_preset(self):
        current_item = self.preset_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "작업을 선택하세요!")
            return
        
        preset_name = list(self.saved_schedules.keys())[self.preset_list.currentRow()]
        ret = QMessageBox.question(self, "삭제 확인", 
                                   f"'{preset_name}' 작업을 삭제하시겠습니까?", 
                                   QMessageBox.Yes | QMessageBox.No)
        
        if ret == QMessageBox.Yes:
            del self.saved_schedules[preset_name]
            self.update_preset_list()
    
    def closeEvent(self, event):
        """종료 시 배터리 모니터 정리"""
        self.battery_monitor.stop()
        event.accept()