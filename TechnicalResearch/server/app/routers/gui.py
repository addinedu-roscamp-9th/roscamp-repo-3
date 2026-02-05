"""
GUI API 라우터
"""

from fastapi import APIRouter
from app.models.gui_model import GuiData
from app.services import gui_service

router = APIRouter()


@router.post("")
def gui_command(data: GuiData):
    """GUI 명령 처리"""
    return gui_service.handle_gui_command(data)
