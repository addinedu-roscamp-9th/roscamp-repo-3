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


def gui_controller(msg):
    data = msg.get("data")
    msg = msg["msg"]
    response = None

    match msg:
        case "connect":
            response = {"success": True}

        case "login":
            response = gui_service.login(data)

        case "fetch_req":
            response = gui_service.fetch_info()

        case "fetch_cmd":
            response = gui_service.fetch_cmd(data)

        case "take_req":
            response = gui_service.take_info()

        case "take_cmd":
            response = gui_service.take_cmd(data)

        case "schedule_req":
            response = gui_service.schedule_info()

        case "schedule_edit":
            response = gui_service.schedule_edit(data)

        case "history_req":
            response = gui_service.history_info()

        case _:
            msg = "error"
            data = "unknown message"

    return {"msg": msg, "data": response}
