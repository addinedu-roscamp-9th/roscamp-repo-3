from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from app.controller import gui_controller

router = APIRouter()


# ws://192.168.0.56:8000/gui
@router.websocket("")
async def jetcobot_connection_test(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            msg = await websocket.receive_json()

            # commands from gui are handled from controller
            result = gui_controller.gui_controller(msg)

            await websocket.send_json(result)
    except WebSocketDisconnect:
        print("Client disconnected normally")
    except ValidationError as e:
        print(f"Invalid data format from client: {e}")
    except ValueError as e:
        print(f"Service error: {e}")
