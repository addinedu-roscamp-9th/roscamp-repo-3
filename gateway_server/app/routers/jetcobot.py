from fastapi import APIRouter

router = APIRouter()


# POST http://192.168.0.56:8000/jetcobot
@router.post("")
def jetcobot_connection():
    return {"msg": "connect", "data": {"success": True}}
