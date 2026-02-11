from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from app.services import gui_service

router = APIRouter()


# ws://192.168.0.56:8000/gui
@router.websocket("")
async def jetcobot_connection_test(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            msg = await websocket.receive_json()

            result = gui_controller(msg)

            await websocket.send_json(result)
    except WebSocketDisconnect:
        print("Client disconnected normally")
    except ValidationError as e:
        print(f"Invalid data format from client: {e}")
    except ValueError as e:
        print(f"Service error: {e}")


def gui_controller(msg):
    msg = msg["msg"]
    data = msg["data"]

    match msg:
        case "connect":
            return True

        case "login":
            user_name = gui_service.login(data)
            print(user_name)
            return user_name

        case "fetch_req":
            what_where = gui_service.fetch_info()
            return what_where

        case "fetch_cmd":
            result = gui_service.fetch_cmd(data)
            return result

        case "take_req":
            result = gui_service.take_info()
            return result

        case "take_cmd":
            result = gui_service.take_cmd()
            return result

        case "schedule_req":
            result = gui_service.schedule_info()
            return result

        case "schedule_edit":
            result = gui_service.schedule_edit(data)
            return result

        case "history_req":
            pass

        case _:
            return {"status": "error", "error": f"Unknown msg_type: {msg}"}
