from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from app.models.robots_model import RobotsData
from app.services import jetcobot_service

router = APIRouter()


# ws://192.168.0.56:8000/jetcobot
@router.websocket("")
async def jetcobot_connection_test(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            raw = await websocket.receive_json()
            data = RobotsData(**raw)
            result = jetcobot_service.test_connection(data)
            await websocket.send_json(result)
    except WebSocketDisconnect:
        print("Client disconnected normally")
    except Exception as e:
        print(f"WebSocket error: {e}")
    # Don't manually call close() - FastAPI handles it
