import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # 위치/회전 정보
from sensor_msgs.msg import BatteryState  # 배터리 정보
import requests
import os

class PinkyStatusSender(Node):
    """
    ROS2 노드: 핑키 로봇 상태를 1초마다 서버로 전송
    - odom 토픽에서 위치/회전 정보 수신
    - battery_state 토픽에서 배터리 상태 수신
    """

    def __init__(self):
        super().__init__('pinky_status_sender')

        # 서버 주소 (환경 변수 또는 기본값)
        host = os.getenv("ROBOT_SERVER_HOST", "192.168.0.52")
        port = os.getenv("ROBOT_SERVER_PORT", "8000")
        self.server_url = f"http://{host}:{port}/pinky/robot/status"

        # 로봇 ID
        self.robot_id = "pinky_01"

        # 초기 상태
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.battery = 100

        # 구독: odom과 battery_state 토픽
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

        # 1초마다 서버로 상태 전송
        self.timer = self.create_timer(1.0, self.send_status)

    # =========================
    # 콜백 함수
    # =========================
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 단순화: yaw 계산 (z 축 회전)
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def battery_callback(self, msg: BatteryState):
        self.battery = int(msg.percentage * 100)  # 0~100%

    # =========================
    # 서버 전송
    # =========================
    def send_status(self):
        data = {
            "robot_id": self.robot_id,
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "battery": self.battery
        }
        try:
            res = requests.post(self.server_url, json=data, timeout=1.0)
            if res.status_code == 200:
                self.get_logger().info(f"Sent status → 200 OK")
            else:
                self.get_logger().warn(f"Sent status → {res.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send status: {e}")


def main():
    rclpy.init()
    node = PinkyStatusSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

