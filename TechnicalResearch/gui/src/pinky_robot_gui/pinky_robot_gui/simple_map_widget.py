from PyQt5.QtWidgets import QWidget, QDialog, QVBoxLayout, QPushButton, QLabel, QMessageBox, QInputDialog
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont


class SimpleMapWidget(QWidget):
    """간단한 맵 위젯 (가상 맵 - 가로로 길게)"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(1200, 400)  # 가로로 길게 변경
        
        # 로봇 위치
        self.robot_x = 600
        self.robot_y = 200
        
        # 배터리
        self.battery_percentage = 0.0
        
        # 방 위치 (픽셀 좌표 - 가로로 배치)
        self.rooms = [
            {"name": "안방", "x": 80, "y": 150},
            {"name": "거실", "x": 310, "y": 150},
            {"name": "옷방", "x": 540, "y": 150},
            {"name": "화장실", "x": 770, "y": 150}
        ]
        
        # Pick Up Zone (오른쪽)
        self.pickup_zone = {"name": "Pick Up Zone", "x": 1000, "y": 150}
        
        # 로봇 클릭 영역
        self.robot_rect = None
        
        # 애니메이션 타이머
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self.update)
        self.animation_timer.start(50)
    
    def update_battery(self, percentage, voltage):
        """배터리 정보 업데이트"""
        self.battery_percentage = percentage
        self.update()
    
    def mousePressEvent(self, event):
        """마우스 클릭 이벤트"""
        if self.robot_rect and self.robot_rect.contains(event.pos()):
            self.show_robot_menu()
    
    def show_robot_menu(self):
        """로봇 메뉴 표시"""
        menu_dialog = QDialog(self)
        menu_dialog.setWindowTitle("로봇 제어")
        menu_dialog.setModal(True)
        menu_dialog.setFixedSize(250, 200)
        
        layout = QVBoxLayout()
        
        title = QLabel("🤖 로봇 제어")
        title.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        btn_bring = QPushButton("📦 물건 가져오기")
        btn_bring.setStyleSheet("""
            QPushButton {
                background: #2196F3; color: white; font-weight: bold;
                height: 40px; border-radius: 5px; font-size: 14px;
            }
            QPushButton:hover { background: #1976D2; }
        """)
        btn_bring.clicked.connect(lambda: self.handle_bring_item(menu_dialog))
        layout.addWidget(btn_bring)
        
        btn_put = QPushButton("📍 물건 갖다놓기")
        btn_put.setStyleSheet("""
            QPushButton {
                background: #4CAF50; color: white; font-weight: bold;
                height: 40px; border-radius: 5px; font-size: 14px;
            }
            QPushButton:hover { background: #388E3C; }
        """)
        btn_put.clicked.connect(lambda: self.handle_put_item(menu_dialog))
        layout.addWidget(btn_put)
        
        btn_move = QPushButton("🚀 이동")
        btn_move.setStyleSheet("""
            QPushButton {
                background: #FF9800; color: white; font-weight: bold;
                height: 40px; border-radius: 5px; font-size: 14px;
            }
            QPushButton:hover { background: #F57C00; }
        """)
        btn_move.clicked.connect(lambda: self.handle_simple_move(menu_dialog))
        layout.addWidget(btn_move)
        
        menu_dialog.setLayout(layout)
        menu_dialog.exec_()
    
    def handle_bring_item(self, parent_dialog):
        """물건 가져오기 처리"""
        parent_dialog.close()
        
        # 물건 선택
        items = ["💊 약", "🍫 초코과자", "🍘 쌀과자"]
        item, ok = QInputDialog.getItem(self, "물건 선택", "가져올 물건을 선택하세요:", items, 0, False)
        
        if ok and item:
            # 방 선택
            room_names = [room["name"] for room in self.rooms]
            room, ok2 = QInputDialog.getItem(self, "방 선택", "가져다 줄 방을 선택하세요:", room_names, 0, False)
            
            if ok2 and room:
                QMessageBox.information(self, "작업 시작", f"{item}를 {room}으로 가져오겠습니다!")
    
    def handle_put_item(self, parent_dialog):
        """물건 갖다놓기 처리"""
        parent_dialog.close()
        
        # 현재 방 선택
        room_names = [room["name"] for room in self.rooms]
        room, ok = QInputDialog.getItem(self, "현재 위치", "현재 있는 방을 선택하세요:", room_names, 0, False)
        
        if ok and room:
            # Pick Up Zone 확인
            ret = QMessageBox.question(
                self, 
                "확인", 
                f"{room}에서 Pick Up Zone으로\n물건을 가져다 놓으시겠습니까?",
                QMessageBox.Yes | QMessageBox.No
            )
            
            if ret == QMessageBox.Yes:
                QMessageBox.information(self, "작업 시작", "Pick Up Zone으로 이동합니다!")
    
    def handle_simple_move(self, parent_dialog):
        """단순 이동 처리"""
        parent_dialog.close()
        
        # 모든 구역 (방 + Pick Up Zone)
        all_locations = [room["name"] for room in self.rooms] + [self.pickup_zone["name"]]
        
        location, ok = QInputDialog.getItem(
            self, 
            "이동할 구역 선택", 
            "이동할 구역을 선택하세요:", 
            all_locations, 
            0, 
            False
        )
        
        if ok and location:
            QMessageBox.information(self, "이동 시작", f"{location}(으)로 이동합니다!")
    
    def paintEvent(self, event):
        """위젯 그리기"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경
        painter.fillRect(self.rect(), QColor(40, 40, 40))
        
        # 그리드 (더 촘촘하게)
        pen = QPen(QColor(60, 60, 60), 1)
        painter.setPen(pen)
        
        for x in range(0, self.width(), 40):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 40):
            painter.drawLine(0, y, self.width(), y)
        
        # 방 그리기 (가로로 배치)
        for room in self.rooms:
            painter.setBrush(QBrush(QColor(33, 150, 243, 80)))
            painter.setPen(QPen(QColor(33, 150, 243), 2))
            painter.drawRoundedRect(room["x"], room["y"], 150, 100, 10, 10)
            
            painter.setPen(QColor(255, 255, 255))
            painter.setFont(QFont("Arial", 14, QFont.Bold))
            painter.drawText(room["x"] + 20, room["y"] + 60, room["name"])
        
        # Pick Up Zone 그리기
        painter.setBrush(QBrush(QColor(255, 193, 7, 80)))
        painter.setPen(QPen(QColor(255, 193, 7), 2))
        painter.drawRoundedRect(self.pickup_zone["x"] - 75, self.pickup_zone["y"], 
                               150, 100, 10, 10)
        
        painter.setPen(QColor(255, 255, 255))
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        painter.drawText(self.pickup_zone["x"] - 65, self.pickup_zone["y"] + 50, "Pick Up Zone")
        
        # 로봇 그리기
        robot_size = 30
        self.robot_rect = QRectF(self.robot_x - robot_size/2, self.robot_y - robot_size/2, 
                                 robot_size, robot_size)
        
        painter.setBrush(QBrush(QColor(244, 67, 54)))
        painter.setPen(QPen(QColor(255, 255, 255), 3))
        painter.drawEllipse(self.robot_rect)
        
        # 로봇 아이콘
        painter.setFont(QFont("Arial", 16))
        painter.drawText(self.robot_rect, Qt.AlignCenter, "🤖")