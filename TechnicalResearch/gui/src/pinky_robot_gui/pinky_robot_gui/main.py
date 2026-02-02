"""
Pinky Robot GUI - 메인 프로그램 (멀티 스크린 버전)
"""
import sys
import threading
import signal
import rclpy

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor

from pinky_robot_gui.robot_node import SimpleRobotNode
from pinky_robot_gui.multi_screen_gui import MultiScreenRobotGUI
from pinky_robot_gui.battery_monitor import BatteryMonitor


def main():
    """메인 함수"""
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # ROS2 초기화
    rclpy.init()
    node = SimpleRobotNode()

    # ROS2 스핀 (별도 스레드)
    threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    ).start()

    # Qt 애플리케이션
    app = QApplication(sys.argv)

    # 다크 모드 설정
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(30, 30, 46))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)

    # 배터리 모니터
    battery_monitor = BatteryMonitor()

    # GUI 실행
    gui = MultiScreenRobotGUI(node, battery_monitor)
    gui.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()