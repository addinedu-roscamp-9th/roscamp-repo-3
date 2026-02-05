"""Main application window — owns the screen stack and background timers."""

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QStackedWidget, QVBoxLayout, QWidget

from app import client
from app.screens.home import HomeScreen
from app.screens.navigation import NavigationScreen
from app.screens.schedule import ScheduleScreen


class MainWindow(QWidget):
    """Top-level widget.  Wires the three screens together and runs the
    battery-status poller."""

    _BATTERY_POLL_MS = 2000
    _ROBOTS = ("Pinky1", "Pinky2")

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pinky Robot Control")
        self.resize(1400, 900)

        # --- screens ------------------------------------------------------
        self._stack = QStackedWidget()

        self._navigation = NavigationScreen(
            on_back=lambda: self._stack.setCurrentIndex(0)
        )
        self._schedule = ScheduleScreen(on_back=lambda: self._stack.setCurrentIndex(0))

        self._stack.addWidget(
            HomeScreen(
                on_pinky=lambda: self._stack.setCurrentIndex(1),
                on_schedule=lambda: self._stack.setCurrentIndex(2),
            )
        )  # index 0
        self._stack.addWidget(self._navigation)  # index 1
        self._stack.addWidget(self._schedule)  # index 2

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
