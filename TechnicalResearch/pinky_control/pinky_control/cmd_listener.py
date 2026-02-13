import math
import threading
import time

import rclpy
import requests
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node


class PinkyCmdListener(Node):
    def __init__(self):
        super().__init__("pinky_cmd_listener")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.robot_id = "pinky_01"
        self.server_url = f"http://192.168.0.52:8000/pinky/robot/cmd/{self.robot_id}"

        # ✅ 너무 빠르면 와이파이/서버에 부담
        self.timer = self.create_timer(0.3, self.check_cmd)

        # move(수동) 상태
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.cmd_end_time = 0.0

        self.default_speed_mps = 0.2
        self.http_timeout_sec = 2.0

        # Nav2(goal) 처리 상태
        self._nav_lock = threading.Lock()
        self._nav_busy = False
        self._navigator = None  # BasicNavigator는 필요할 때 1번만 생성

    def _ensure_navigator(self) -> BasicNavigator:
        if self._navigator is None:
            self._navigator = BasicNavigator()
            self.get_logger().info("Waiting Nav2 active...")
            self._navigator.waitUntilNav2Active()
            self.get_logger().info("Nav2 is active.")
        return self._navigator

    def _start_goto_pose(self, x: float, y: float, theta: float):
        """Nav2 goal은 오래 걸릴 수 있어서 별도 스레드로 실행"""

        def worker():
            with self._nav_lock:
                self._nav_busy = True
            try:
                nav = self._ensure_navigator()

                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = nav.get_clock().now().to_msg()

                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.orientation.z = math.sin(theta / 2.0)
                goal.pose.orientation.w = math.cos(theta / 2.0)

                self.get_logger().info(f"CMD goto_pose: x={x}, y={y}, theta={theta}")
                nav.goToPose(goal)

                while not nav.isTaskComplete():
                    time.sleep(0.1)

                result = nav.getResult()
                self.get_logger().info(f"goto_pose result: {result}")

            except Exception as e:
                self.get_logger().error(f"goto_pose failed: {e}")
            finally:
                with self._nav_lock:
                    self._nav_busy = False

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def check_cmd(self):
        cmd = None
        try:
            res = requests.get(self.server_url, timeout=self.http_timeout_sec)
            cmd = res.json()
        except Exception as e:
            self.get_logger().error(f"Failed to fetch cmd: {e}")

        # 1) 새 명령 해석
        if isinstance(cmd, dict):
            t = cmd.get("type")

            # ✅ 서버가 보내는 move: {"type":"move","distance_cm":10}
            if t == "move":
                try:
                    distance_cm = float(cmd.get("distance_cm", 0.0))
                except (TypeError, ValueError):
                    distance_cm = 0.0

                if distance_cm > 0.0:
                    distance_m = distance_cm / 100.0
                    duration = distance_m / self.default_speed_mps
                    duration = max(0.1, min(duration, 5.0))

                    self.current_linear = self.default_speed_mps
                    self.current_angular = 0.0
                    self.cmd_end_time = time.time() + duration

                    self.get_logger().info(
                        f"CMD move: {distance_cm:.1f}cm -> v={self.default_speed_mps:.2f}m/s for {duration:.2f}s"
                    )

            # ✅ 좌표 이동: {"type":"goto_pose","x":..,"y":..,"theta":..}
            elif t == "goto_pose":
                # nav 수행 중이면 중복 goal 방지
                with self._nav_lock:
                    busy = self._nav_busy
                if not busy:
                    try:
                        x = float(cmd["x"])
                        y = float(cmd["y"])
                        theta = float(cmd.get("theta", 0.0))
                        self._start_goto_pose(x, y, theta)
                    except Exception as e:
                        self.get_logger().error(f"invalid goto_pose cmd: {e} cmd={cmd}")

        # 2) /cmd_vel publish (Nav2 goal 수행 중이면 수동 cmd_vel은 0으로 유지)
        twist = Twist()
        with self._nav_lock:
            nav_busy = self._nav_busy

        now = time.time()
        if nav_busy:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            if now <= self.cmd_end_time:
                twist.linear.x = self.current_linear
                twist.angular.z = self.current_angular
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = PinkyCmdListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
