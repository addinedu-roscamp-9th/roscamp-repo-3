from app.models.robot_model import RobotData
from app.services import jetcobot_service
from fastapi import APIRouter

router = APIRouter()

gui_connections: list = []


# http://192.168.5.10:8000/jetcobot
@router.post("")
def jetcobot_connection_test(data: RobotData):
    result = jetcobot_service.test_connection(data)
    return result
