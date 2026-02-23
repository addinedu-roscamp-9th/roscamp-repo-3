"""ROS2 interface service for the Pinky robot.

Flow
----
1. Server calls ``send_pinky_target(position)``
   → publishes a ``PorterTarget`` on ``/porter_target``
   → ``vel_sub`` node receives it and sends a Nav2 goal
   → when Nav2 gets close enough, it cancels and publishes ``/pid_activate``
   → ``pid_node`` takes over, does fine PID positioning
   → ``pid_node`` publishes ``PorterStatus(status="arrived")`` on ``/porter_status``

2. Server calls ``wait_pinky(timeout)``
   → blocks (spins the ROS node) until ``/porter_status`` is received or timeout

3. ``nav_pinky(position, timeout)`` is a convenience wrapper for both steps.
"""

import math
import threading
import time

import rclpy
from debugcrew_msgs.msg import PorterStatus, PorterTarget
from rclpy.node import Node

# ---------------------------------------------------------------------------
# Global singleton
# ---------------------------------------------------------------------------
_ros_node: "PinkyClient | None" = None
_ros_lock = threading.Lock()


class PinkyClient(Node):
    """Minimal ROS2 node used by the gateway server to command Pinky."""

    def __init__(self) -> None:
        super().__init__("pinky_client")

        # Publisher: send navigation targets to the robot
        self.target_pub = self.create_publisher(PorterTarget, "/porter_target", 10)

        # Subscriber: receive completion notification from the robot
        self._status_event = threading.Event()
        self._last_status: PorterStatus | None = None

        self._status_sub = self.create_subscription(
            PorterStatus, "/porter_status", self._status_cb, 10
        )

        self.get_logger().info("PinkyClient initialised")

    def _status_cb(self, msg: PorterStatus) -> None:
        """Handle /porter_status message from pid_node."""
        self.get_logger().info(
            f"/porter_status received: status='{msg.status}' "
            f"x={msg.x:.4f}, y={msg.y:.4f}, yaw={msg.yaw:.4f}"
        )
        self._last_status = msg
        self._status_event.set()

    def clear_status(self) -> None:
        """Reset completion state before starting a new navigation leg."""
        self._last_status = None
        self._status_event.clear()


# ---------------------------------------------------------------------------
# Singleton accessor
# ---------------------------------------------------------------------------


def _get_ros_node() -> PinkyClient:
    """Return (and lazily create) the global PinkyClient node."""
    global _ros_node

    with _ros_lock:
        if _ros_node is None:
            if not rclpy.ok():
                rclpy.init()
            _ros_node = PinkyClient()

    return _ros_node


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def send_pinky_target(position) -> bool:
    """Publish a navigation target to the robot via /porter_target.

    Args:
        position: Object with ``x``, ``y``, ``yaw`` attributes (floats).
                  ``x`` and ``y`` are in metres.
                  ``yaw`` is in **degrees** (as stored in the DB):
                    0   → facing +X axis (initial/forward direction)
                    90  → turned 90° to the left  (facing +Y)
                    -90 → turned 90° to the right (facing -Y)
                  The value is converted to radians before publishing to ROS.

    Returns:
        True if the message was published successfully, False otherwise.
    """
    try:
        node = _get_ros_node()
        node.clear_status()

        msg = PorterTarget()
        msg.x = float(position.x)
        msg.y = float(position.y)
        # position.yaw is in degrees (as stored in the DB); convert to radians
        # for the ROS pipeline.  0° = facing +X, 90° = facing +Y (left turn),
        # -90° = facing -Y (right turn).
        msg.yaw = math.radians(float(position.yaw))

        # Wait for vel_sub subscriber to be discovered (up to 2 s).
        # ROS 2 subscriber discovery is asynchronous; publishing immediately
        # after node creation drops the message before vel_sub is matched.
        # Use time.sleep instead of spin_once so that no ROS callbacks are
        # processed here — spin_once would risk delivering a /porter_status
        # message (from the previous run) and setting _status_event before
        # wait_pinky even starts.
        deadline = time.monotonic() + 2.0
        while node.target_pub.get_subscription_count() == 0:
            time.sleep(0.05)
            if time.monotonic() > deadline:
                node.get_logger().warn(
                    "No subscriber on /porter_target after 2 s — publishing anyway"
                )
                break

        node.target_pub.publish(msg)
        node.get_logger().info(
            f"Published /porter_target: x={msg.x:.4f}, y={msg.y:.4f}, yaw={msg.yaw:.4f}"
        )
        return True

    except Exception as e:
        print(f"Error publishing /porter_target: {e}")
        return False


def wait_pinky(timeout: int) -> bool:
    """Block until /porter_status is received or timeout expires.

    Spins the ROS node in a loop so callbacks are processed while waiting.

    Args:
        timeout: Maximum seconds to wait.

    Returns:
        True if the robot reported ``status == "arrived"``, False otherwise.
    """
    try:
        node = _get_ros_node()
        deadline = time.monotonic() + float(timeout)

        while rclpy.ok():
            # Process any pending ROS callbacks (non-blocking spin_once)
            rclpy.spin_once(node, timeout_sec=0.1)

            if node._status_event.is_set():
                status = node._last_status
                node._status_event.clear()
                if status is not None and status.status == "arrived":
                    node.get_logger().info("Navigation complete: arrived")
                    return True
                else:
                    node.get_logger().warn(
                        f"Unexpected status: {status.status if status else 'None'}"
                    )
                    return False

            if time.monotonic() >= deadline:
                node.get_logger().warn(
                    f"Navigation timeout after {timeout} s — no /porter_status received"
                )
                return False

        return False

    except Exception as e:
        print(f"Error waiting for Pinky: {e}")
        return False


def nav_pinky(position, timeout: int = 120) -> bool:
    """Send navigation target and wait for completion.

    Convenience wrapper combining ``send_pinky_target`` and ``wait_pinky``.

    Args:
        position: Object with ``x``, ``y``, ``yaw`` attributes.
                  ``yaw`` must be in **degrees** (see ``send_pinky_target``).
        timeout:  Maximum seconds to wait for arrival (default 120).

    Returns:
        True if Pinky arrived successfully, False otherwise.
    """
    if not send_pinky_target(position):
        return False
    return wait_pinky(timeout)
