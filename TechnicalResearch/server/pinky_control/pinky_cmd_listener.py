import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests
import time

class PinkyCmdListener(Node):
    def __init__(self):
        super().__init__('pinky_cmd_listener')

        self.robot_id = "pinky_01"
        self.server_url = f"http://192.168.0.52:8000/pinky/robot/cmd/{self.robot_id}"

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.last_cmd_time = time.time()
        self.timer = self.create_timer(0.5, self.fetch_cmd)

    def fetch_cmd(self):
        try:
            res = requests.get(self.server_url, timeout=0.5)
            cmd = res.json()

            self.get_logger().info(f"CMD: {cmd}")

            twist = Twist()
            twist.linear.x = float(cmd.get("linear", 0.0))
            twist.angular.z = float(cmd.get("angular", 0.0))

            self.cmd_pub.publish(twist)
            self.last_cmd_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Failed to fetch cmd: {e}")

def main():
    rclpy.init()
    node = PinkyCmdListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

