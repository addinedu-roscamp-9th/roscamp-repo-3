import math

import rclpy as rp
from action_msgs.msg import GoalStatus
from debugcrew_msgs.msg import PorterTarget
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


# Distance (metres) at which Nav2 is cancelled and PID takes over
SWITCH_DISTANCE = 0.30


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """Convert yaw angle to quaternion (z, w) components."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class VelSub(Node):
    """Relay node: receives /porter_target, drives Nav2, hands off to PID."""

    def __init__(self):
        super().__init__("vel_sub")

        # Active goal state
        self._goal_handle = None
        self._target: PorterTarget | None = None
        self._pid_activated = False

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Subscribe to target commands from the server
        self._target_sub = self.create_subscription(
            PorterTarget, "/porter_target", self._target_callback, 10
        )

        # Publisher to activate the PID node with the precise final target
        self._pid_pub = self.create_publisher(PorterTarget, "/pid_activate", 10)

        self.get_logger().info("vel_sub started — waiting for /porter_target")

    # ------------------------------------------------------------------
    # Target received from server
    # ------------------------------------------------------------------

    def _target_callback(self, msg: PorterTarget) -> None:
        """Handle a new navigation target from the gateway server."""
        self.get_logger().info(
            f"Received target: x={msg.x:.4f}, y={msg.y:.4f}, yaw={msg.yaw:.4f}"
        )
        self._target = msg
        self._pid_activated = False

        # Cancel any in-flight Nav2 goal before issuing a new one
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling previous Nav2 goal")
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self._send_nav2_goal(msg))
            self._goal_handle = None
        else:
            self._send_nav2_goal(msg)

    # ------------------------------------------------------------------
    # Nav2 action lifecycle
    # ------------------------------------------------------------------

    def _send_nav2_goal(self, target: PorterTarget) -> None:
        """Build and send a NavigateToPose goal for the given target."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available — aborting")
            return

        qz, qw = _yaw_to_quaternion(float(target.yaw))

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target.x)
        goal_msg.pose.pose.position.y = float(target.y)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending Nav2 goal: x={target.x:.4f}, y={target.y:.4f}, yaw={target.yaw:.4f}"
        )

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav2_feedback_cb
        )
        send_future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future) -> None:
        """Handle Nav2 goal acceptance/rejection."""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 goal send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            return

        self.get_logger().info("Nav2 goal accepted")
        self._goal_handle = goal_handle

        # Register result callback (fires when Nav2 finishes normally)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_feedback_cb(self, feedback_msg) -> None:
        """Monitor remaining distance; hand off to PID when close enough."""
        if self._pid_activated:
            return

        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f"Nav2 distance remaining: {remaining:.3f} m")

        if remaining < SWITCH_DISTANCE and self._target is not None:
            self.get_logger().info(
                f"Distance {remaining:.3f} m < {SWITCH_DISTANCE} m "
                "— cancelling Nav2, activating PID"
            )
            self._pid_activated = True
            self._activate_pid()

    def _activate_pid(self) -> None:
        """Cancel Nav2 and hand control to the PID node."""
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_nav2_cancelled)
            self._goal_handle = None
        else:
            self._publish_pid_activate()

    def _on_nav2_cancelled(self, future) -> None:
        """Called once Nav2 cancel is confirmed; then publish PID activation."""
        try:
            future.result()
            self.get_logger().info("Nav2 goal cancelled successfully")
        except Exception as e:
            self.get_logger().warn(f"Nav2 cancel returned error (ignoring): {e}")
        self._publish_pid_activate()

    def _publish_pid_activate(self) -> None:
        """Publish the final target to /pid_activate so the PID node takes over."""
        if self._target is None:
            return
        self.get_logger().info(
            f"Publishing PID activate: x={self._target.x:.4f}, "
            f"y={self._target.y:.4f}, yaw={self._target.yaw:.4f}"
        )
        self._pid_pub.publish(self._target)

    def _nav2_result_cb(self, future) -> None:
        """Handle Nav2 navigation result (only relevant if PID was not triggered)."""
        if self._pid_activated:
            return  # PID already took over — ignore Nav2 result

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"Nav2 result callback error: {e}")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                "Nav2 reached goal without PID handoff — activating PID for fine correction"
            )
            self._pid_activated = True
            self._publish_pid_activate()
        else:
            self.get_logger().warn(f"Nav2 finished with non-success status: {status}")


def main(args=None) -> None:
    """Entry point for vel_sub node."""
    rp.init(args=args)

    node = VelSub()

    try:
        rp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rp.shutdown()


if __name__ == "__main__":
    main()
