from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from app.services import gui_service

router = APIRouter()


# ws://192.168.0.56:8000/gui
@router.websocket("")
async def gui_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            msg = await websocket.receive_json()
            result = gui_controller(msg)
            await websocket.send_json(result)
    except WebSocketDisconnect:
        print("Client disconnected")
    except ValidationError as e:
        print(e)


def gui_controller(req):
    msg = req.get("msg")
    data = req.get("data")
    res = None

    match msg:
        case "connect":
            res = {"success": True}

        case "login":
            res = gui_service.login(data)

        case "fetch_req":
            res = gui_service.fetch_info()

        case "fetch_cmd":
            res = gui_service.fetch_cmd(data)

        case "fetch_confirm":
            res = gui_service.fetch_confirm()

        case "take_req":
            res = gui_service.take_info()

        case "take_cmd":
            res = gui_service.take_cmd(data)

        case "take_confirm":
            res = gui_service.take_confirm()

        case "schedule_req":
            res = gui_service.schedule_info()

        case "schedule_edit":
            res = gui_service.schedule_edit(data)

        case "history_req":
            res = gui_service.history_info()

        case _:
            msg = "error"
            res = "unknown message"

    return {"msg": msg, "data": res}
