from fastapi import APIRouter

from app.services import jetcobot_service

router = APIRouter()

gui_connections: list = []


@router.post("/")
def jetcobot_connection_test():
    result = jetcobot_service.test_connection()
    return result
