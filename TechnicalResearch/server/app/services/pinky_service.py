"""
Pinky Robot 서비스 로직
"""

import random


def get_robot_status(robot_id: str):
    """로봇 상태 조회"""
    # 실제로는 DB나 로봇에서 조회
    # 여기서는 시뮬레이션
    return {
        "robot_id": robot_id,
        "battery": random.uniform(70, 95),  # 시뮬레이션
        "x": 0.0,
        "y": 0.0,
        "status": "idle",
    }


def update_robot_status(robot_id: str, battery: int, x: float, y: float):
    """로봇 상태 업데이트"""
    # 실제로는 DB에 저장
    return {"status": "success", "message": "Status updated"}
