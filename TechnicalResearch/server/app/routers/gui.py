from fastapi import APIRouter

from app.services import gui_service

router = APIRouter()


@router.post("")
def gui_command():
    return gui_service.gui_connection_test()


@router.post("login")
def gui_login():
    result = gui_service.query_user_details()
    return result
