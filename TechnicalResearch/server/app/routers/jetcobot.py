from fastapi import APIRouter


from app.models.robot_model import RobotData
from app.services import jetcobot_service

router = APIRouter()

gui_connections: list = []


@router.post("/")
def jetcobot_connection_test(data: RobotData):
    result = jetcobot_service.test_connection(data)
    return result
