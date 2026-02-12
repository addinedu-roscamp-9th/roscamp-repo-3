#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def main():
    rclpy.init()

    # ===============================
    # 1. 인자 확인
    # ===============================
    if len(sys.argv) < 3:
        print("사용법: ros2 run pinky_control goto_pose x y [yaw]")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0

    print("🚀 Nav2 좌표 이동 시작")

    # ===============================
    # 2. Navigator 생성
    # ===============================
    navigator = BasicNavigator()

    print("⏳ Nav2 활성화 대기 중...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 활성화 완료")

    # ===============================
    # 3. 목표 좌표 생성
    # ===============================
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = navigator.get_clock().now().to_msg()

    goal.pose.position.x = x
    goal.pose.position.y = y

    # yaw → quaternion 변환
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)

    print(f"➡️ 목표 좌표: x={x}, y={y}, yaw={yaw}")

    # ===============================
    # 4. 이동 명령
    # ===============================
    navigator.goToPose(goal)

    # ===============================
    # 5. 이동 완료 대기
    # ===============================
    while not navigator.isTaskComplete():
        print("📍 이동 중...")
        time.sleep(1.0)  # CPU 점유 방지

    # ===============================
    # 6. 결과 확인
    # ===============================
    result = navigator.getResult()

    if result == 0:
        print("🎉 목표 지점 도착 성공!")
    else:
        print(f"⚠ 이동 실패 (result code: {result})")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
