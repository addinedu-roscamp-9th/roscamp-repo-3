from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QListWidget, 
    QPushButton, QComboBox, QLabel, QMessageBox,
    QInputDialog, QListWidgetItem
)
from PyQt5.QtCore import Qt


class TaskBuilderDialog(QDialog):
    """작업 생성 다이얼로그"""
    
    def __init__(self, parent=None, preset_locations=None, existing_tasks=None):
        super().__init__(parent)
        self.preset_locations = preset_locations or []
        self.tasks = existing_tasks[:] if existing_tasks else []
        
        self.setWindowTitle("📋 작업 스케줄 만들기")
        self.resize(500, 600)
        
        self.init_ui()
        self.update_task_list()
    
    def init_ui(self):
        """UI 초기화"""
        layout = QVBoxLayout()
        
        # 안내 문구
        info_label = QLabel("작업 순서를 만드세요:")
        info_label.setStyleSheet("font-size: 14px; font-weight: bold; margin: 10px;")
        layout.addWidget(info_label)
        
        # 작업 목록
        self.task_list = QListWidget()
        self.task_list.setStyleSheet("""
            QListWidget {
                background: #2d2d2d; color: white; font-size: 13px;
                border: 2px solid #444; border-radius: 5px;
            }
            QListWidget::item { padding: 8px; }
            QListWidget::item:selected { background: #0078d7; }
        """)
        layout.addWidget(self.task_list)
        
        # 작업 추가 영역
        add_layout = QHBoxLayout()
        
        self.location_combo = QComboBox()
        self.location_combo.setStyleSheet("""
            QComboBox {
                background: #3d3d3d; color: white; font-size: 13px;
                border: 2px solid #555; border-radius: 5px; padding: 5px;
            }
        """)
        
        for name, _, _, _ in self.preset_locations:
            self.location_combo.addItem(f"📍 {name}")
        
        add_layout.addWidget(self.location_combo)
        
        btn_add_move = QPushButton("➕ 이동 추가")
        btn_add_move.setStyleSheet("""
            background: #2ecc71; color: white; font-weight: bold;
            height: 35px; border-radius: 5px; font-size: 13px;
        """)
        btn_add_move.clicked.connect(self.add_move_task)
        add_layout.addWidget(btn_add_move)
        
        btn_add_action = QPushButton("📦 동작 추가")
        btn_add_action.setStyleSheet("""
            background: #3498db; color: white; font-weight: bold;
            height: 35px; border-radius: 5px; font-size: 13px;
        """)
        btn_add_action.clicked.connect(self.add_action_task)
        add_layout.addWidget(btn_add_action)
        
        layout.addLayout(add_layout)
        
        # 작업 관리 버튼
        manage_layout = QHBoxLayout()
        
        btn_move_up = QPushButton("⬆️")
        btn_move_up.setStyleSheet("background: #95a5a6; color: white; height: 35px; font-size: 16px;")
        btn_move_up.clicked.connect(self.move_task_up)
        manage_layout.addWidget(btn_move_up)
        
        btn_move_down = QPushButton("⬇️")
        btn_move_down.setStyleSheet("background: #95a5a6; color: white; height: 35px; font-size: 16px;")
        btn_move_down.clicked.connect(self.move_task_down)
        manage_layout.addWidget(btn_move_down)
        
        btn_delete = QPushButton("🗑️ 삭제")
        btn_delete.setStyleSheet("background: #e74c3c; color: white; height: 35px; font-weight: bold;")
        btn_delete.clicked.connect(self.delete_task)
        manage_layout.addWidget(btn_delete)
        
        layout.addLayout(manage_layout)
        
        # 확인/취소 버튼
        button_layout = QHBoxLayout()
        
        btn_ok = QPushButton("✅ 저장")
        btn_ok.setStyleSheet("""
            background: #27ae60; color: white; font-weight: bold;
            height: 45px; border-radius: 5px; font-size: 14px;
        """)
        btn_ok.clicked.connect(self.accept)
        button_layout.addWidget(btn_ok)
        
        btn_cancel = QPushButton("❌ 취소")
        btn_cancel.setStyleSheet("""
            background: #c0392b; color: white; font-weight: bold;
            height: 45px; border-radius: 5px; font-size: 14px;
        """)
        btn_cancel.clicked.connect(self.reject)
        button_layout.addWidget(btn_cancel)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def add_move_task(self):
        """이동 작업 추가"""
        index = self.location_combo.currentIndex()
        if index < 0:
            return
        
        location_name = self.preset_locations[index][0]
        self.tasks.append(("move", location_name))
        self.update_task_list()
    
    def add_action_task(self):
        """동작 작업 추가"""
        action_name, ok = QInputDialog.getText(
            self, 
            "동작 추가", 
            "동작 이름을 입력하세요:\n(예: '물건 싣기', '벨 울리기' 등)"
        )
        
        if ok and action_name:
            self.tasks.append(("action", action_name))
            self.update_task_list()
    
    def delete_task(self):
        """작업 삭제"""
        current_row = self.task_list.currentRow()
        if current_row >= 0:
            del self.tasks[current_row]
            self.update_task_list()
    
    def move_task_up(self):
        """작업 위로 이동"""
        current_row = self.task_list.currentRow()
        if current_row > 0:
            self.tasks[current_row], self.tasks[current_row - 1] = \
                self.tasks[current_row - 1], self.tasks[current_row]
            self.update_task_list()
            self.task_list.setCurrentRow(current_row - 1)
    
    def move_task_down(self):
        """작업 아래로 이동"""
        current_row = self.task_list.currentRow()
        if 0 <= current_row < len(self.tasks) - 1:
            self.tasks[current_row], self.tasks[current_row + 1] = \
                self.tasks[current_row + 1], self.tasks[current_row]
            self.update_task_list()
            self.task_list.setCurrentRow(current_row + 1)
    
    def update_task_list(self):
        """작업 목록 업데이트"""
        self.task_list.clear()
        
        for i, (task_type, task_name) in enumerate(self.tasks):
            if task_type == "move":
                icon = "📍"
                text = f"{i+1}. {icon} {task_name}로 이동"
            else:
                icon = "📦"
                text = f"{i+1}. {icon} {task_name}"
            
            self.task_list.addItem(QListWidgetItem(text))
    
    def accept(self):
        """저장 버튼"""
        if not self.tasks:
            QMessageBox.warning(self, "경고", "최소 1개 이상의 작업을 추가하세요!")
            return
        super().accept()
