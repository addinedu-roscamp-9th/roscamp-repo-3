"""
WebSocket router for real-time communication with robots and GUI.
Runs alongside existing HTTP endpoints.
"""

from typing import Dict

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from app.models.gui_model import GuiData
from app.models.robots_model import RobotsData
from app.services import gui_service, jetcobot_service, pinky_service

router = APIRouter()


class ConnectionManager:
    """Manages WebSocket connections for multiple clients."""

    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}

    async def connect(self, websocket: WebSocket, client_id: str):
        await websocket.accept()
        self.active_connections[client_id] = websocket
        print(f"[WS] Client connected: {client_id}")

    def disconnect(self, client_id: str):
        self.active_connections.pop(client_id, None)
        print(f"[WS] Client disconnected: {client_id}")

    async def send_json(self, client_id: str, data: dict):
        if client_id in self.active_connections:
            await self.active_connections[client_id].send_json(data)

    async def broadcast(self, data: dict, exclude: str = None):
        """Broadcast message to all connected clients."""
        for client_id, ws in self.active_connections.items():
            if client_id != exclude:
                await ws.send_json(data)

    def get_connected_clients(self) -> list:
        return list(self.active_connections.keys())


manager = ConnectionManager()


def handle_jetcobot_message(data: dict) -> dict:
    """Handle JetCobot messages."""
    try:
        robot_data = RobotsData(
            robot_id=data.get("robot_id", ""),
            namespace=data.get("namespace", ""),
            robot_type=data.get("robot_type", ""),
            robot_name=data.get("robot_name", ""),
        )
        result = jetcobot_service.test_connection(robot_data)
        return {"type": "jetcobot_response", "status": "success", "data": result}
    except Exception as e:
        return {"type": "jetcobot_response", "status": "error", "message": str(e)}


def handle_pinky_message(data: dict) -> dict:
    """Handle Pinky robot messages."""
    try:
        action = data.get("action", "get_status")
        robot_id = data.get("robot_id", "")

        if action == "get_status":
            result = pinky_service.get_robot_status(robot_id)
            return {"type": "pinky_response", "status": "success", "data": result}

        if action == "update_status":
            result = pinky_service.update_robot_status(
                robot_id=robot_id,
                battery=data.get("battery", 0),
                x=data.get("x", 0.0),
                y=data.get("y", 0.0),
            )
            return {"type": "pinky_response", "status": "success", "data": result}

        return {
            "type": "pinky_response",
            "status": "error",
            "message": "Unknown action",
        }
    except Exception as e:
        return {"type": "pinky_response", "status": "error", "message": str(e)}


def handle_gui_message(data: dict) -> dict:
    """Handle GUI command messages."""
    try:
        gui_data = GuiData(
            robot_id=data.get("robot_id"),
            status=data.get("status"),
            command=data.get("command", ""),
            destination=data.get("destination"),
            item=data.get("item"),
            from_room=data.get("from_room"),
            schedule_name=data.get("schedule_name"),
            tasks=data.get("tasks"),
        )
        result = gui_service.handle_gui_command(gui_data)
        return {"type": "gui_response", "status": "success", "data": result}
    except Exception as e:
        return {"type": "gui_response", "status": "error", "message": str(e)}


@router.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    """
    WebSocket endpoint for real-time communication.

    Message format:
    {
        "type": "jetcobot" | "pinky" | "gui" | "ping",
        ... (type-specific fields)
    }
    """
    await manager.connect(websocket, client_id)

    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type", "")

            if msg_type == "jetcobot":
                response = handle_jetcobot_message(data)

            elif msg_type == "pinky":
                response = handle_pinky_message(data)

            elif msg_type == "gui":
                response = handle_gui_message(data)

            elif msg_type == "ping":
                response = {"type": "pong", "clients": manager.get_connected_clients()}

            elif msg_type == "broadcast":
                # Broadcast message to all other clients
                await manager.broadcast(
                    {
                        "type": "broadcast",
                        "from": client_id,
                        "message": data.get("message"),
                    },
                    exclude=client_id,
                )
                response = {"type": "broadcast_sent", "status": "success"}

            else:
                response = {
                    "type": "error",
                    "message": f"Unknown message type: {msg_type}",
                }

            await websocket.send_json(response)

    except WebSocketDisconnect:
        manager.disconnect(client_id)
    except Exception as e:
        print(f"[WS] Error for client {client_id}: {e}")
        manager.disconnect(client_id)


@router.get("/ws/clients")
async def get_connected_clients():
    """HTTP endpoint to check connected WebSocket clients."""
    return {"clients": manager.get_connected_clients()}
