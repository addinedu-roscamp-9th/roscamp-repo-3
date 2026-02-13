import math

import rclpy as rp
from debugcrew_msgs.msg import PinkyTarget
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PinkyNode(Node):
    def __init__(self):
        super().__init__("pinky_node")
        self.subscription = self.create_subscription(
            PinkyTarget, "/pinky/target", self.target_callback, 10
        )
        # Changed to standard /cmd_vel topic
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = None
        self.current_cmd = Twist()
        self.publishing = False
        self.get_logger().info(
            "Pinky node started, listening for targets on /pinky/target"
        )

    def target_callback(self, msg):
        """receives target position (x, y) and direction (theta) and moves pinky"""
        self.get_logger().info(
            f"Received target: x={msg.x}, y={msg.y}, theta={msg.theta}"
        )

        # Stop previous movement timer if exists
        if self.timer is not None:
            self.timer.cancel()

        # Create velocity command for pinky machine
        cmd = Twist()

        # Calculate distance to target (assuming current position is 0,0)
        distance = math.sqrt(msg.x**2 + msg.y**2)

        # Calculate angle to target
        target_angle = math.atan2(msg.y, msg.x)

        # Calculate angle difference (how much we need to rotate)
        angle_diff = msg.theta - target_angle

        self.get_logger().info(
            f"Distance: {distance:.2f}m, Target angle: {target_angle:.2f}rad, "
            f"Final heading: {msg.theta:.2f}rad"
        )

        # Set reasonable velocities (NOT the raw position values!)
        # Linear velocity: move forward at reasonable speed
        if distance > 0.1:  # If target is more than 10cm away
            cmd.linear.x = min(0.3, distance * 0.1)  # Max 0.3 m/s
        else:
            cmd.linear.x = 0.0

        # Angular velocity: rotate towards target
        if abs(angle_diff) > 0.1:  # If angle difference > ~6 degrees
            cmd.angular.z = max(min(angle_diff * 0.5, 1.0), -1.0)  # Max ±1.0 rad/s
        else:
            cmd.angular.z = 0.0

        self.current_cmd = cmd
        self.publishing = True

        # Create timer to publish continuously (10 Hz)
        # Mobile robots need continuous velocity commands
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Stop after 5 seconds (safety timeout)
        self.create_timer(5.0, self.stop_publishing)

        self.get_logger().info(
            f"Publishing velocities: linear.x={cmd.linear.x:.2f} m/s, "
            f"angular.z={cmd.angular.z:.2f} rad/s to /cmd_vel"
        )

    def publish_velocity(self):
        """Continuously publish velocity commands"""
        if self.publishing:
            self.publisher.publish(self.current_cmd)

    def stop_publishing(self):
        """Stop publishing and send zero velocity"""
        self.publishing = False
        if self.timer is not None:
            self.timer.cancel()

        # Send stop command
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        self.get_logger().info("Stopped publishing - sent zero velocity")


def main(args=None) -> None:
    rp.init(args=args)

    pinky_node = PinkyNode()

    try:
        rp.spin(pinky_node)
    finally:
        pinky_node.destroy_node()
        rp.shutdown()


if __name__ == "__main__":
    main()
