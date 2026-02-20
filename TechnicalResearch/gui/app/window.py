"""Main application window — owns the screen stack and background timers."""

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QStackedWidget, QVBoxLayout, QWidget

from app import client
from app.screens.home import HomeScreen
from app.screens.login import LoginScreen
from app.screens.navigation import NavigationScreen
from app.screens.schedule import ScheduleScreen


class MainWindow(QWidget):
    """Top-level widget.  Wires the screens together and runs the
    battery-status poller."""

    _BATTERY_POLL_MS = 2000
    _ROBOTS = ("Pinky1", "Pinky2")

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pinky Robot Control")
        self.resize(1400, 900)

        # --- screens ------------------------------------------------------
        self._stack = QStackedWidget()

        # Login screen (index 0)
        self._login = LoginScreen(
            on_login_success=lambda: self._stack.setCurrentIndex(1)
        )
        self._stack.addWidget(self._login)

        # Home screen (index 1)
        self._home = HomeScreen(
            on_pinky=lambda: self._stack.setCurrentIndex(2),
            on_schedule=lambda: self._stack.setCurrentIndex(3),
        )
        self._stack.addWidget(self._home)

        # Navigation screen (index 2)
        self._navigation = NavigationScreen(
            on_back=lambda: self._stack.setCurrentIndex(1)
        )
        self._stack.addWidget(self._navigation)

        # Schedule screen (index 3)
        self._schedule = ScheduleScreen(on_back=lambda: self._stack.setCurrentIndex(1))
        self._stack.addWidget(self._schedule)

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._stack)
        self.setLayout(layout)

        # --- battery poller -----------------------------------------------
        self._battery_timer = QTimer(self)
        self._battery_timer.timeout.connect(self._poll_batteries)
        self._battery_timer.start(self._BATTERY_POLL_MS)

    # ------------------------------------------------------------------
    # Battery polling
    # ------------------------------------------------------------------

    def _poll_batteries(self) -> None:
        """Fetch battery status for every known robot and push to the UI."""
        for robot_name in self._ROBOTS:
            try:
                resp = client.get_robot_status(robot_name)
                if resp.status_code == 200:
                    battery = resp.json().get("battery", 0)
                    self._navigation.update_battery(robot_name, battery)
            except Exception:  # network errors are non-fatal here
                pass
