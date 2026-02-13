"""Login screen — authentication before accessing home screen."""

from typing import Callable

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGroupBox,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QMessageBox,
)


class LoginScreen(QWidget):
    """Login screen with ID and password fields."""

    def __init__(self, on_login_success: Callable[[], None]):
        super().__init__()
        self._on_login_success = on_login_success
        self._init_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _init_ui(self) -> None:
        # Background color #fff2ccff
        self.setStyleSheet("background: #fff2cc;")

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(50, 50, 50, 50)

        # Title
        title = QLabel("Pinky Service")
        title.setStyleSheet(
            "font-size: 56px; font-weight: bold; color: #5a5a5a; "
            "margin-bottom: 50px; background: transparent;"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Login box
        login_box = self._create_login_box()
        layout.addWidget(login_box, 0, Qt.AlignCenter)

        layout.addStretch()
        self.setLayout(layout)

    def _create_login_box(self) -> QGroupBox:
        """Create the login box with ID and password fields."""
        box = QGroupBox("Login")
        box.setFixedSize(600, 400)  # 크기 대폭 증가
        box.setStyleSheet(
            """
            QGroupBox {
                background: #eeeeee;
                border: 3px solid #9999aa;
                border-radius: 30px;
                font-size: 24px;
                font-weight: bold;
                color: #5a5a5a;
                padding-top: 30px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 10px 30px;
            }
            """
        )

        layout = QVBoxLayout()
        layout.setSpacing(20)
        layout.setContentsMargins(50, 60, 50, 40)

        # ID field
        id_label = QLabel("Id :")
        id_label.setStyleSheet(
            "font-size: 20px; color: #333; background: transparent; "
            "border: none; margin-bottom: 5px;"
        )
        layout.addWidget(id_label)

        self.id_input = QLineEdit()
        self.id_input.setPlaceholderText(" ")
        self.id_input.setStyleSheet(
            """
            QLineEdit {
                background: white;
                border: 2px solid #999;
                border-radius: 8px;
                padding: 15px;
                font-size: 18px;
                color: #333;
                min-height: 25px;
            }
            QLineEdit:focus {
                border: 2px solid #5a5a5a;
            }
            """
        )
        layout.addWidget(self.id_input)

        # Password field
        pw_label = QLabel("Pw :")
        pw_label.setStyleSheet(
            "font-size: 20px; color: #333; background: transparent; "
            "border: none; margin-top: 10px; margin-bottom: 5px;"
        )
        layout.addWidget(pw_label)

        self.pw_input = QLineEdit()
        self.pw_input.setPlaceholderText(" ")
        self.pw_input.setEchoMode(QLineEdit.Password)
        self.pw_input.setStyleSheet(
            """
            QLineEdit {
                background: white;
                border: 2px solid #999;
                border-radius: 8px;
                padding: 15px;
                font-size: 18px;
                color: #333;
                min-height: 25px;
            }
            QLineEdit:focus {
                border: 2px solid #5a5a5a;
            }
            """
        )
        # Enter key to login
        self.pw_input.returnPressed.connect(self._handle_login)
        layout.addWidget(self.pw_input)

        # Login button
        login_btn = QPushButton("로그인")
        login_btn.setFixedHeight(50)
        login_btn.setStyleSheet(
            """
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #6b9bd1, stop:1 #5a7fb8);
                color: white;
                font-size: 18px;
                font-weight: bold;
                border: 2px solid #4a6a95;
                border-radius: 10px;
                margin-top: 15px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #7aabef, stop:1 #6a8fc8);
            }
            QPushButton:pressed {
                background: #4a6a95;
            }
            """
        )
        login_btn.clicked.connect(self._handle_login)
        layout.addWidget(login_btn)

        box.setLayout(layout)
        return box

    # ------------------------------------------------------------------
    # Login handling
    # ------------------------------------------------------------------

    def _handle_login(self) -> None:
        """Validate credentials and proceed to home screen."""
        user_id = self.id_input.text().strip()
        password = self.pw_input.text().strip()

        # Simple validation (you can replace with actual authentication)
        if not user_id or not password:
            QMessageBox.warning(
                self, "입력 오류", "아이디와 비밀번호를 모두 입력하세요."
            )
            return

        # TODO: Replace with actual authentication logic
        # For now, accept any non-empty credentials
        if user_id and password:
            self._on_login_success()
        else:
            QMessageBox.warning(
                self, "로그인 실패", "아이디 또는 비밀번호가 올바르지 않습니다."
            )
            self.pw_input.clear()
            self.id_input.setFocus()
