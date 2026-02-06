# pinky_cmd_listener.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # 로봇 이동 명령을 보내는 메시지 타입
import requests  # HTTP 요청을 보내기 위해 사용
import time  # 시간 계산용

class PinkyCmdListener(Node):
    def __init__(self):
        super().__init__('pinky_cmd_listener')

        # 핑키 로봇 ID
        self.robot_id = "pinky_01"

        # 명령을 가져올 서버 URL (FastAPI)
        self.server_url = f"http://192.168.0.52:8000/pinky/robot/cmd/{self.robot_id}"

        # ROS2 퍼블리셔 생성 (cmd_vel 토픽으로 Twist 메시지 전송)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 0.5초마다 서버에서 명령 가져오기
        self.timer = self.create_timer(0.5, self.fetch_and_execute_cmd)

    def fetch_and_execute_cmd(self):
        """서버에서 명령을 가져와서 로봇을 움직이는 함수"""
        try:
            # 서버에서 명령(GET) 요청
            res = requests.get(self.server_url, timeout=0.5)
            if res.status_code == 200:
                cmd = res.json()
                self.get_logger().info(f"CMD received: {cmd}")

                # 명령 실행
                self.move_robot(cmd)
            else:
                # 서버가 200 OK가 아닌 경우
                self.get_logger().warn(f"Server returned {res.status_code}")
        except Exception as e:
            # 서버 연결 실패 시 로그 출력
            self.get_logger().error(f"Failed to fetch cmd: {e}")

    def move_robot(self, cmd):
        """서버에서 받은 명령으로 3초간 이동 후 자동 정지"""
        # 명령에서 linear, angular 값 읽기
        linear = cmd.get("linear", 0.0)
        angular = cmd.get("angular", 0.0)

        # Twist 메시지 생성
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        # 3초 동안 로봇 이동
        start = time.time()
        while time.time() - start < 3.0:
            self.pub.publish(twist)
            # 0.1초마다 spin_once 호출하여 ROS 이벤트 처리
            rclpy.spin_once(self, timeout_sec=0.1)

        # 이동 종료 후 멈추기
        stop_twist = Twist()  # 모든 값 0
        self.pub.publish(stop_twist)
        self.get_logger().info("Movement completed, robot stopped.")

def main():
    # ROS2 초기화
    rclpy.init()

    # 노드 생성
    node = PinkyCmdListener()

    # ROS2 스핀 (콜백 함수 실행)
    rclpy.spin(node)

    # 종료 처리
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

