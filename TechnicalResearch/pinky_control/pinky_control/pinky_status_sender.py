import rclpy
import requests
from rclpy.node import Node


class PinkyStatusSender(Node):
    def __init__(self):
        super().__init__("pinky_status_sender")

        self.server_url = "http://192.168.0.77:8000/pinky/robot/status"
        self.robot_id = "pinky_01"

        # 1초마다 서버로 상태 전송
        self.timer = self.create_timer(1.0, self.send_status)

    def send_status(self):
        data = {
            "robot_id": self.robot_id,
            "x": 1.0,
            "y": 2.0,
            "theta": 0.3,
            "battery": 85,
        }

        try:
            res = requests.post(self.server_url, json=data, timeout=1.0)
            self.get_logger().info(f"Sent status → {res.status_code}")
        except Exception as e:
            self.get_logger().error(f"Failed to send status: {e}")


def main():
    rclpy.init()
    node = PinkyStatusSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
