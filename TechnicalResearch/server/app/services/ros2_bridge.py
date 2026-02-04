"""
ROS2 Bridge - ROS2 노드와 통신
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import threading


class ROS2Bridge(Node):
    """ROS2 통신 브릿지"""

    def __init__(self):
        super().__init__("server_ros2_bridge")

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 위치 매핑 (방 이름 → 좌표)
        self.location_map = {
            "안방": (1.27, -0.439, 0.0),
            "거실": (-1.00, -0.11, math.pi / 2),
            "옷방": (-0.28, 0.219, math.pi),
            "화장실": (0.18, -0.212, -math.pi / 2),
            "pickup_zone": (2.0, 0.0, 0.0),
        }

        self.get_logger().info("🔗 ROS2 Bridge 초기화 완료")

    def move_to_location(self, location_name: str) -> bool:
        """특정 위치로 이동"""
        if location_name not in self.location_map:
            self.get_logger().error(f"❌ 알 수 없는 위치: {location_name}")
            return False

        x, y, yaw = self.location_map[location_name]

        # Nav2 서버 대기
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("❌ Nav2 서버 응답 없음")
            return False

        # Goal 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Yaw → Quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"📍 {location_name}로 이동: ({x:.2f}, {y:.2f})")

        # Goal 전송
        self.nav_client.send_goal_async(goal_msg)
        return True

    def emergency_stop(self):
        """긴급 정지"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().warn("🚨 긴급 정지!")


# 전역 ROS2 브릿지 인스턴스
_ros2_bridge = None
_ros2_thread = None


def init_ros2_bridge():
    """ROS2 브릿지 초기화"""
    global _ros2_bridge, _ros2_thread

    if _ros2_bridge is None:
        rclpy.init()
        _ros2_bridge = ROS2Bridge()

        # 별도 스레드에서 스핀
        _ros2_thread = threading.Thread(
            target=rclpy.spin, args=(_ros2_bridge,), daemon=True
        )
        _ros2_thread.start()

    return _ros2_bridge


def get_ros2_bridge():
    """ROS2 브릿지 가져오기"""
    global _ros2_bridge

    if _ros2_bridge is None:
        return init_ros2_bridge()

    return _ros2_bridge
