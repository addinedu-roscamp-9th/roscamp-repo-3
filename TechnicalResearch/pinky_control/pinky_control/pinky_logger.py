import rclpy
import requests
from geometry_msgs.msg import Twist
from rclpy.node import Node

API_URL = "http://192.168.0.52:5000/robot/command"  # 서버 주소
ROBOT_ID = "pinky_d988"


class PinkyLogger(Node):
    def __init__(self):
        super().__init__("pinky_logger")
        # cmd_vel 토픽 구독
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.callback_cmd_vel, 10
        )
        self.get_logger().info("Pinky Logger Node started!")

    def callback_cmd_vel(self, msg):
        # 움직임 기록
        linear = msg.linear.x
        angular = msg.angular.z

        # action 결정
        if linear > 0:
            action = "forward"
        elif linear < 0:
            action = "backward"
        elif angular > 0:
            action = "turn_left"
        elif angular < 0:
            action = "turn_right"
        else:
            action = "stop"

        payload = {"robot_id": ROBOT_ID, "command": "move", "value": action}

        try:
            resp = requests.post(API_URL, json=payload, timeout=2)
            if resp.status_code == 200:
                self.get_logger().info(f"[DB] Movement logged: {action}")
            else:
                self.get_logger().error(f"[DB] Failed to log: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"[DB] Exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PinkyLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
