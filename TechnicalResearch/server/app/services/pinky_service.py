import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Global ROS2 node instance
_pinky_node = None
_ros2_thread = None


class PinkyNavigationNode(Node):
    """ROS2 node for Pinky robot navigation"""

    def __init__(self):
        super().__init__("pinky_navigation_node")

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Navigation status tracking
        self.navigation_status = "idle"  # idle, navigating, succeeded, failed
        self.current_goal = None
        self.goal_handle = None

        self.get_logger().info("Pinky Navigation Node initialized")

    def navigate_to_position(self, x: float, y: float, yaw: float) -> bool:
        """Send navigation goal to position (x, y, yaw)"""

        # Wait for Nav2 server
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 server not available")
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Store current goal
        self.current_goal = {"x": x, "y": y, "yaw": yaw}
        self.navigation_status = "navigating"

        self.get_logger().info(
            f"Navigating to position: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})"
        )

        # Send goal asynchronously with callbacks
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

        return True

    def _goal_response_callback(self, future):
        """Called when goal is accepted or rejected"""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.navigation_status = "failed"
            return

        self.get_logger().info("Goal accepted by Nav2")

        # Get result
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Called when navigation completes"""
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation succeeded - Pinky reached destination")
            self.navigation_status = "succeeded"
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
            self.navigation_status = "failed"

    def _feedback_callback(self, feedback_msg):
        """Called periodically during navigation"""
        # feedback = feedback_msg.feedback
        # Can log current position, distance remaining, etc.
        pass

    def get_navigation_status(self):
        """Get current navigation status"""
        return {
            "status": self.navigation_status,
            "goal": self.current_goal,
        }


def _init_pinky_node():
    """Initialize Pinky ROS2 node"""
    global _pinky_node, _ros2_thread

    if _pinky_node is None:
        rclpy.init()
        _pinky_node = PinkyNavigationNode()

        # Spin in separate daemon thread
        _ros2_thread = threading.Thread(
            target=rclpy.spin, args=(_pinky_node,), daemon=True
        )
        _ros2_thread.start()

    return _pinky_node


def _get_pinky_node():
    """Get or initialize Pinky ROS2 node"""
    global _pinky_node

    if _pinky_node is None:
        return _init_pinky_node()

    return _pinky_node


def send_command_to_pinky(position):
    """Send Pinky robot to position (x, y, w)"""
    x = position.x
    y = position.y
    w = position.w  # w is yaw angle in radians

    print(f"Sending Pinky to position - x: {x}, y: {y}, w: {w}")

    try:
        # Get or initialize ROS2 node
        node = _get_pinky_node()
        success = node.navigate_to_position(x, y, w)

        if success:
            print("Navigation command sent successfully")
        else:
            print("Failed to send navigation command")

        return success
    except Exception as e:
        print(f"Error sending command to Pinky: {e}")
        return False


def wait_for_pinky_arrival(timeout=60, check_interval=0.5):
    """
    Wait for Pinky to reach its destination (blocking)

    Args:
        timeout: Maximum time to wait in seconds (default 60s)
        check_interval: How often to check status in seconds (default 0.5s)

    Returns:
        True if Pinky reached destination, False if failed or timeout
    """

    start_time = time.time()
    elapsed = 0

    print(f"Waiting for Pinky to reach destination (timeout: {timeout}s)...")

    while elapsed < timeout:
        status = get_pinky_status()
        current_status = status.get("status")

        if current_status == "succeeded":
            elapsed = time.time() - start_time
            print(f"Pinky reached destination in {elapsed:.1f}s")
            return True
        elif current_status == "failed":
            print("Pinky navigation failed")
            return False
        elif current_status == "idle":
            print("Warning: Pinky status is idle (navigation may not have started)")

        time.sleep(check_interval)
        elapsed = time.time() - start_time

    print(f"Timeout: Pinky did not reach destination within {timeout}s")
    return False


def get_pinky_status():
    """Get Pinky's current navigation status"""
    try:
        node = _get_pinky_node()
        return node.get_navigation_status()
    except Exception as e:
        print(f"Error getting Pinky status: {e}")
        return {"status": "unknown", "goal": None}
