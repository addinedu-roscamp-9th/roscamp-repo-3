from fastapi import APIRouter

from app.models.gui_model import GuiData
from app.services import gui_service

router = APIRouter()

gui_connections: list = []


# http://192.168.5.10:8000/gui
@router.post("")
def gui_connection_test(data: GuiData):
    result = gui_service.gui_connection_test(data)
    return result
