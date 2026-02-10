from app.services import gui_service
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import ValidationError

router = APIRouter()


# ws://192.168.0.56:8000/gui
@router.websocket("")
async def jetcobot_connection_test(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            msg = await websocket.receive_json()

            # commands from gui are handled from controller
            result = gui_controller(msg)

            await websocket.send_json(result)
    except WebSocketDisconnect:
        print("Client disconnected normally")
    except ValidationError as e:
        print(f"Invalid data format from client: {e}")
    except ValueError as e:
        print(f"Service error: {e}")


# msg example from gui
#
# {
#     "msg_type": "login",
#     "data": {
#         "id": "admin",
#         "pw": "1234"
#     }
# }


def gui_controller(msg):
    msg_type = msg["msg_type"]
    data = msg["data"]

    match msg_type:
        case "connect":
            return True

        case "login":
            user_name = gui_service.login(data)
            print(user_name)
            return user_name

        case "fetch":
            what_where = gui_service.fetch_info()
            return what_where

        case "fetch_cmd":
            result = gui_service.fetch_cmd(data)
            return result

        case "schedule_info":
            result = gui_service.schedule_info()
        case _:
            return {"status": "error", "error": f"Unknown msg_type: {msg_type}"}
