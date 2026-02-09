from fastapi import APIRouter

from app.services import jetcobot_service

router = APIRouter()


# POST http://192.168.0.56:8000/jetcobot
@router.post("")
def jetcobot_connection():
    result = jetcobot_service.jetcobot_connection()
    return result
