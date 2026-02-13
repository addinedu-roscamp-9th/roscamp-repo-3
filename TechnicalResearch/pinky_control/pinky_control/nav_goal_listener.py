import math
import os
import time

import rclpy
import requests
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node


class NavGoalListener(Node):
    """
    FastAPI 서버에서 (x,y,yaw) 목표를 가져와 Nav2로 이동시키는 노드
    - GUI가 POST로 /pinky/nav/goal/{robot_id} 에 목표 저장
    - 이 노드는 GET으로 목표를 가져가면(pop) 실행
    """

    def __init__(self):
        super().__init__("nav_goal_listener")

        self.robot_id = os.getenv("ROBOT_ID", "pinky_01")

        host = os.getenv("ROBOT_SERVER_HOST", "192.168.0.52")
        port = os.getenv("ROBOT_SERVER_PORT", "8000")
        self.goal_url = f"http://{host}:{port}/pinky/nav/goal/{self.robot_id}"

        self.get_logger().info(f"robot_id = {self.robot_id}")
        self.get_logger().info(f"goal_url  = {self.goal_url}")

        # Nav2 Navigator 준비
        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting Nav2 active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 active ✅")

        # 주기적으로 목표 확인
        self.timer = self.create_timer(0.5, self.poll_goal)

        self.is_moving = False

    def poll_goal(self):
        # 이동 중이면 새로운 goal을 안 받게(충돌 방지)
        if self.is_moving:
            return

        try:
            res = requests.get(self.goal_url, timeout=1.0)
            if res.status_code != 200:
                self.get_logger().warn(f"Server returned {res.status_code}")
                return

            data = res.json()

            # idle이면 아무 것도 안함
            if data.get("type") != "goto":
                return

            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            yaw = float(data.get("yaw", 0.0))

            self.get_logger().info(f"✅ Goal received: x={x}, y={y}, yaw={yaw}")
            self.go_to_pose(x, y, yaw)

        except Exception as e:
            self.get_logger().error(f"Failed to fetch goal: {e}")

    def go_to_pose(self, x: float, y: float, yaw: float):
        self.is_moving = True

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y

        # yaw(rad) → quaternion
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.navigator.goToPose(goal)

        # 완료 대기(너무 빡세게 돌지 않게 sleep)
        while not self.navigator.isTaskComplete():
            time.sleep(0.2)

        result = self.navigator.getResult()
        self.get_logger().info(f"🏁 Navigation result code: {result}")

        self.is_moving = False


def main():
    rclpy.init()
    node = NavGoalListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
