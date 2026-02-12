import time

from pymycobot.mycobot import MyCobot

from app.model.posture import Posture

PORT = "/dev/ttyJETCOBOT"
BAUD = "1000000"
SPEED = 30


class Move:
    def __init__(self, port: str = PORT, baud: str = BAUD):
        self.mc = MyCobot(port, baud)

    def send_angles(self, data: Posture, speed: int = SPEED):
        angles = [data.j1, data.j2, data.j3, data.j4, data.j5, data.j6]
        self.mc.send_angles(angles, speed)
        time.sleep(2.0)

    def set_gripper(self, gap: int, speed: int = SPEED):
        self.mc.set_gripper_value(gap, speed)
        time.sleep(0.5)

    def execute(self, data: Posture, speed: int = SPEED):
        self.send_angles(data, speed)
        self.set_gripper(data.gap)
