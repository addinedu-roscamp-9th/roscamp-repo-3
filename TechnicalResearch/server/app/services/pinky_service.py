import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from debugcrew_msgs.msg import PorterTarget
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Global ROS2 node instance
_ros_node = None
_ros_lock = threading.Lock()


class PinkyNavigationClient(Node):
    """ROS2 node for sending navigation commands to Pinky robot"""

    def __init__(self):
        super().__init__("pinky_navigation_client")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None
        self._result_future = None

        # Topic publisher for /pinky/target
        self.target_publisher = self.create_publisher(
            PorterTarget, "/porter_target", 10
        )

        self.get_logger().info("Pinky navigation client initialized")


def _get_ros_node() -> PinkyNavigationClient:
    """Get or create the global ROS2 node instance"""
    global _ros_node

    with _ros_lock:
        if _ros_node is None:
            # Initialize rclpy if not already initialized
            if not rclpy.ok():
                rclpy.init()

            _ros_node = PinkyNavigationClient()

    return _ros_node


def cmd_pinky(position) -> bool:
    """
    Send navigation command to Pinky robot

    Args:
        position: Object with x, y, w attributes
                 x, y: coordinates in meters
                 w: yaw angle in radians

    Returns:
        bool: True if command was sent successfully, False otherwise
    """
    x = position.x
    y = position.y
    yaw = position.yaw

    try:
        node = _get_ros_node()

        # Wait for action server
        if not node._action_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("Navigation action server not available")
            return False

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        node.get_logger().info(f"Sending goal: x={x:.4f}, y={y:.4f}, yaw={yaw:.4f} rad")

        # Send goal
        send_goal_future = node._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)

        if not send_goal_future.done():
            node.get_logger().error("Failed to send goal - timeout")
            return False

        node._goal_handle = send_goal_future.result()

        if not node._goal_handle.accepted:
            node.get_logger().error("Goal was rejected by action server")
            return False

        node.get_logger().info("Goal accepted by action server")
        node._result_future = node._goal_handle.get_result_async()
        return True

    except Exception as e:
        print(f"Error sending goal to Pinky: {e}")
        return False


def wait_pinky(timeout: int) -> bool:
    """
    Wait for Pinky to reach the navigation goal

    Args:
        timeout: Maximum time to wait in seconds

    Returns:
        bool: True if goal was reached, False if timeout or failed
    """
    try:
        node = _get_ros_node()
        timeout_sec = float(timeout)

        if node._result_future is None:
            node.get_logger().error("No active goal to wait for")
            return False

        rclpy.spin_until_future_complete(
            node, node._result_future, timeout_sec=timeout_sec
        )

        if not node._result_future.done():
            node.get_logger().warn(f"Navigation timeout after {timeout_sec} seconds")
            # Cancel the goal
            if node._goal_handle:
                cancel_future = node._goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
            return False

        result = node._result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().info("Navigation succeeded!")
            return True
        else:
            node.get_logger().error(f"Navigation failed with status: {result.status}")
            return False

    except Exception as e:
        print(f"Error waiting for Pinky: {e}")
        return False


def publish_pinky_target(x: float, y: float, yaw: float) -> bool:
    """
    Publish target to /pinky/target topic

    Args:
        x: X coordinate
        y: Y coordinate
        yaw: Yaw angle in radians

    Returns:
        bool: True if message was published successfully, False otherwise
    """
    try:
        node = _get_ros_node()

        target = PorterTarget()
        target.x = x
        target.y = y
        target.yaw = yaw

        node.target_publisher.publish(target)
        node.get_logger().info(f"Published target: x={x:.4f}, y={y:.4f}, yaw={yaw:.4f}")

        return True
    except Exception as e:
        print(f"Error publishing target: {e}")
        return False
