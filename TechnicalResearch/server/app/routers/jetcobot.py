from fastapi import APIRouter

from app.models.robots_model import RobotsData
from app.services import jetcobot_service

router = APIRouter()


# http://192.168.5.10:8000/jetcobot
@router.post("")
def jetcobot_connection_test(data: RobotsData):
    result = jetcobot_service.test_connection(data)
    return result
