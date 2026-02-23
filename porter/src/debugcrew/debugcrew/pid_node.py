"""PID docking node for Pinky robot.

Subscribes to /pid_activate (PorterTarget) to receive the precise goal,
uses /amcl_pose for localisation, publishes velocity commands to /cmd_vel,
and publishes /porter_status (PorterStatus) when the goal is reached.

State machine
-------------
IDLE           — waiting for /pid_activate
PID_POS        — rotating toward goal + driving forward
PID_FINE_POS   — slow approach when very close
PID_YAW_FINAL  — rotate in-place to reach final yaw
DONE           — goal reached, /porter_status published
"""

import math
from math import atan2, sqrt

import rclpy
from debugcrew_msgs.msg import PorterStatus, PorterTarget
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# ---------------------------------------------------------------------------
# Tolerances and gains
# ---------------------------------------------------------------------------
POS_TOLERANCE = 0.05      # metres — switch to fine positioning
FINE_TOLERANCE = 0.02     # metres — switch to yaw alignment
YAW_TOLERANCE = 0.05      # radians — goal reached
SWITCH_DISTANCE = 0.08    # metres — initial PID coarse→fine switch (replicated for clarity)

P_LIN, I_LIN, D_LIN = 1.2, 0.02, 0.10
MAX_LIN, MIN_LIN = 0.20, -0.20

P_ANG, I_ANG, D_ANG = 2.0, 0.00, 0.10
MAX_ANG, MIN_ANG = 0.80, -0.80

CONTROL_PERIOD = 0.05     # seconds (20 Hz)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    """Extract yaw from a ROS quaternion message."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PID:
    """Minimal discrete PID controller with output clamping and integral limit."""

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_limit: float = 1.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time: float | None = None

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def update(self, error: float, now_sec: float) -> float:
        """Compute PID output for the given error at the given time."""
        dt = 0.01 if self._prev_time is None else max(now_sec - self._prev_time, 0.001)

        self._integral += error * dt
        self._integral = max(
            -self.integral_limit, min(self.integral_limit, self._integral)
        )

        derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(self.output_min, min(self.output_max, output))

        self._prev_error = error
        self._prev_time = now_sec
        return output


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class PidNode(Node):
    """PID fine-positioning node for Pinky robot."""

    def __init__(self) -> None:
        super().__init__("pid_node")

        # State
        self._mode = "IDLE"
        self._current_pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self._target: PorterTarget | None = None

        # PID controllers
        self._lin_pid = PID(
            kp=P_LIN, ki=I_LIN, kd=D_LIN,
            output_min=MIN_LIN, output_max=MAX_LIN,
        )
        self._ang_pid = PID(
            kp=P_ANG, ki=I_ANG, kd=D_ANG,
            output_min=MIN_ANG, output_max=MAX_ANG,
        )

        # /amcl_pose is latched — use transient_local so we get the last pose
        # immediately on subscribe even if AMCL hasn't published a new one yet.
        amcl_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscriptions
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, amcl_qos
        )
        self._activate_sub = self.create_subscription(
            PorterTarget, "/pid_activate", self._activate_cb, 10
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status_pub = self.create_publisher(PorterStatus, "/porter_status", 10)

        # Control loop timer (20 Hz)
        self._timer = self.create_timer(CONTROL_PERIOD, self._control_loop)

        self.get_logger().info("pid_node started — waiting for /pid_activate")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """Update current pose from AMCL localisation."""
        p = msg.pose.pose
        self._current_pose = (
            float(p.position.x),
            float(p.position.y),
            float(yaw_from_quaternion(p.orientation)),
        )

    def _activate_cb(self, msg: PorterTarget) -> None:
        """Start PID control toward the given target."""
        if self._mode != "IDLE":
            self.get_logger().info(
                f"PID re-activated from {self._mode} — resetting for new target "
                f"x={msg.x:.4f}, y={msg.y:.4f}, yaw={msg.yaw:.4f}"
            )
        else:
            self.get_logger().info(
                f"PID activated: target x={msg.x:.4f}, y={msg.y:.4f}, yaw={msg.yaw:.4f}"
            )
        self._publish_stop()   # Ensure robot is stopped before starting fresh
        self._target = msg
        self._mode = "PID_POS"
        self._lin_pid.reset()
        self._ang_pid.reset()

    # ------------------------------------------------------------------
    # Control loop (20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Main PID control state machine."""
        if self._mode == "IDLE" or self._mode == "DONE":
            self._publish_stop()
            return

        if self._current_pose is None or self._target is None:
            self._publish_stop()
            return

        cx, cy, cyaw = self._current_pose
        tx, ty, tyaw = float(self._target.x), float(self._target.y), float(self._target.yaw)

        dx = tx - cx
        dy = ty - cy
        distance = sqrt(dx * dx + dy * dy)
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # ---- PID_POS: drive toward goal ----
        if self._mode == "PID_POS":
            desired_heading = atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - cyaw)

            v_cmd = self._lin_pid.update(distance, now_sec)
            w_cmd = self._ang_pid.update(heading_error, now_sec)

            # Suppress forward motion while misaligned
            if abs(heading_error) > 0.7:
                v_cmd = 0.0

            self._publish_cmd(v_cmd, w_cmd)

            if distance < POS_TOLERANCE:
                self.get_logger().info(
                    f"[PID_POS→PID_FINE_POS] dist={distance:.4f} m"
                )
                self._publish_stop()
                self._lin_pid.reset()
                self._ang_pid.reset()
                self._mode = "PID_FINE_POS"

        # ---- PID_FINE_POS: slow approach ----
        elif self._mode == "PID_FINE_POS":
            desired_heading = atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - cyaw)

            if distance < FINE_TOLERANCE:
                self.get_logger().info(
                    f"[PID_FINE_POS→PID_YAW_FINAL] dist={distance:.4f} m"
                )
                self._publish_stop()
                self._ang_pid.reset()
                self._mode = "PID_YAW_FINAL"
                return

            w_cmd = self._ang_pid.update(heading_error, now_sec)
            if abs(heading_error) > 0.5:
                v_cmd = 0.0
            else:
                v_cmd = self._lin_pid.update(distance, now_sec)

            # Slow down in fine mode
            v_cmd *= 0.5
            w_cmd *= 0.5

            if abs(v_cmd) < 0.003:
                v_cmd = 0.0
            if abs(w_cmd) < 0.010:
                w_cmd = 0.0

            self._publish_cmd(v_cmd, w_cmd)

        # ---- PID_YAW_FINAL: rotate to target yaw ----
        elif self._mode == "PID_YAW_FINAL":
            yaw_error = normalize_angle(tyaw - cyaw)

            if abs(yaw_error) < YAW_TOLERANCE:
                self.get_logger().info(
                    f"[PID_YAW_FINAL→DONE] yaw_error={yaw_error:.4f} rad"
                )
                self._publish_stop()
                self._mode = "DONE"
                self._publish_status("arrived")
                return

            w_cmd = self._ang_pid.update(yaw_error, now_sec)
            if abs(w_cmd) < 0.01:
                w_cmd = 0.0
            self._publish_cmd(0.0, w_cmd)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_cmd(self, v: float, w: float) -> None:
        """Publish a Twist velocity command."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_vel_pub.publish(msg)

    def _publish_stop(self) -> None:
        """Publish zero velocity."""
        self._publish_cmd(0.0, 0.0)

    def _publish_status(self, status: str) -> None:
        """Publish arrival status back to the server via /porter_status."""
        msg = PorterStatus()
        msg.status = status
        if self._current_pose is not None:
            msg.x = float(self._current_pose[0])
            msg.y = float(self._current_pose[1])
            msg.yaw = float(self._current_pose[2])
        self._status_pub.publish(msg)
        self.get_logger().info(
            f"Published /porter_status: status='{status}' "
            f"x={msg.x:.4f}, y={msg.y:.4f}, yaw={msg.yaw:.4f}"
        )


def main(args=None) -> None:
    """Entry point for pid_node."""
    rclpy.init(args=args)

    node = PidNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
