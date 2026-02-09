# pinky_cmd_listener.py
import time

import rclpy
import requests
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PinkyCmdListener(Node):
    """
    ROS2 노드: 서버에서 핑키 명령을 받아 Twist 메시지로 변환 후
    핑키 모터에 publish
    - move 명령: distance_cm 단위 입력 → 실제 이동 거리 계산
    """

    def __init__(self):
        super().__init__("pinky_cmd_listener")

        # 핑키 로봇 ID (서버에서 구분용)
        self.robot_id = "pinky_01"

        # 서버에서 명령을 GET할 URL
        self.server_url = f"http://192.168.0.52:8000/pinky/robot/cmd/{self.robot_id}"

        # ROS2 퍼블리셔 생성 (/cmd_vel 토픽, QoS 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 0.5초마다 서버에서 명령 가져오기
        self.timer = self.create_timer(0.5, self.fetch_and_execute_cmd)

        # 기본 이동 속도 (m/s)
        self.default_speed = 0.2

    def fetch_and_execute_cmd(self):
        """서버에서 명령을 가져와서 실제로 이동시키는 함수"""
        try:
            res = requests.get(self.server_url, timeout=2.0)
            if res.status_code == 200:
                cmd = res.json()
                self.get_logger().info(f"CMD received: {cmd}")

                # cmd가 move 타입이면 이동 처리
                if cmd.get("type") == "move":
                    distance_cm = cmd.get("distance_cm", 10)
                    if distance_cm > 0:
                        self.move_distance(distance_cm)
                else:
                    self.get_logger().info("No move command, robot stays idle")

            else:
                self.get_logger().warn(f"Server returned status {res.status_code}")

        except Exception as e:
            self.get_logger().error(f"Failed to fetch cmd: {e}")

    def move_distance(self, distance_cm):
        """
        distance_cm 만큼 이동
        - 기본 속도 self.default_speed 사용
        - 이동 시간 계산: t = distance / speed
        - 이동 후 멈춤
        """
        # 미터 단위로 변환
        distance_m = distance_cm / 100.0
        duration = distance_m / self.default_speed  # 이동 시간 = 거리 / 속도

        self.get_logger().info(f"Moving {distance_cm}cm (~{duration:.2f}s)")

        twist = Twist()
        twist.linear.x = self.default_speed  # x축 전진
        twist.angular.z = 0.0  # 회전 없음

        start_time = time.time()
        while time.time() - start_time < duration:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=2.0)

        # 이동 종료 후 완전히 멈춤
        self.pub.publish(Twist())
        self.get_logger().info("Movement completed, robot stopped.")


def main():
    rclpy.init()
    node = PinkyCmdListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
