import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    rclpy.init()

    if len(sys.argv) < 3:
        print("사용법: ros2 run pinky_control goto_pose x y")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0

    print(f"➡️ 목표 이동: x={x}, y={y}")
    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        pass

    print("✅ 도착 완료")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
