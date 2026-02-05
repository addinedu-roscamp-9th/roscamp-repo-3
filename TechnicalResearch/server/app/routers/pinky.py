"""
Pinky Robot API 라우터
"""

from fastapi import APIRouter
from app.models.pinky_model import RobotStatus
from app.services import pinky_service

router = APIRouter()


@router.get("/status/{robot_id}")
def get_robot_status(robot_id: str):
    """로봇 상태 조회"""
    return pinky_service.get_robot_status(robot_id)


@router.post("/status")
def update_robot_status(status: RobotStatus):
    """로봇 상태 업데이트"""
    return pinky_service.update_robot_status(
        status.robot_id, status.battery, status.x, status.y
    )
