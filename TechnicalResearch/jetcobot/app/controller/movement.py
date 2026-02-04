import time

from pymycobot.mycobot280 import MyCobot280

from app.model.model import PosturesData

PORT = "/dev/ttyJETCOBOT"
BAUD = 1_000_000
SPEED = 30


class Move:
    def __init__(self, port: str = PORT, baud: int = BAUD):
        self.mc = MyCobot280(port, baud)
        time.sleep(1.0)

    def send_angles(self, data: PosturesData, speed: int = SPEED):
        """Move robot to joint angles from PosturesData."""
        angles = [data.j1, data.j2, data.j3, data.j4, data.j5, data.j6]
        self.mc.send_angles(angles, speed)
        time.sleep(2.0)

    def set_gripper(self, gap: int, speed: int = 50):
        """Set gripper opening value (0-100)."""
        self.mc.set_gripper_value(gap, speed)
        time.sleep(0.5)

    def execute(self, data: PosturesData, speed: int = SPEED):
        """Execute full posture: move to angles and set gripper."""
        self.send_angles(data, speed)
        self.set_gripper(data.gap)
