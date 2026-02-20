import rclpy as rp
from debugcrew_msgs.msg import PorterTarget
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class VelSub(Node):
    def __init__(self):
        super().__init__("vel_sub")
        self.subscription = self.create_subscription(
            PorterTarget, "/porter_target", self.target_callback, 10
        )
        self.publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.get_logger().info(
            "Velocity subscriber started, listening for targets on /porter_target"
        )

    def target_callback(self, msg: PorterTarget) -> None:
        """Receives target position (x, y) and orientation (w), sends Nav2 goal."""
        self.get_logger().info(f"Received target: x={msg.x}, y={msg.y}, w={msg.w}")

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(msg.x)
        goal.pose.position.y = float(msg.y)
        goal.pose.orientation.w = float(msg.w)

        self.publisher.publish(goal)
        self.get_logger().info(
            f"Goal sent: x={msg.x}, y={msg.y}, orientation.w={msg.w}"
        )


def main(args=None) -> None:
    rp.init(args=args)

    vel_sub = VelSub()

    try:
        rp.spin(vel_sub)
    finally:
        vel_sub.destroy_node()
        rp.shutdown()


if __name__ == "__main__":
    main()
