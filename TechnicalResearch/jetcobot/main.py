import os
import threading

import uvicorn
from app.model.msg_model import ServerMsg
from app.model.posture_model import PosturesData
from app.service.connect import Connect
from app.service.movement import Move
from dotenv import load_dotenv
from fastapi import FastAPI

load_dotenv()

GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
ENDPOINT = os.getenv("ENDPOINT", "jetcobot")

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8001

# Global state
move_controller = None
postures_map = {}

# FastAPI app
app = FastAPI()


@app.post("/pose")
async def handle_pose(request: ServerMsg):
    msg_type = request.msg_type
    print(f"msg_type: {msg_type}")
    print(f"item: {request.item}")

    match msg_type:
        case "fetch":
            posture = postures_map.get("shelve_side")
        case "take":
            posture = postures_map.get("pinky_side")

    try:
        if move_controller is None:
            return {"status": "error", "message": "Robot arm not initialized"}

        if posture is None:
            return {
                "status": "error",
                "message": f"Posture not found for item: {request.item}",
                "available_items": list(postures_map.keys()),
            }

        # Execute movement
        print(f"Executing posture: {posture.pos_name}")
        move_controller.execute(posture)

        return {
            "status": "success",
            "message": f"Pose executed for item: {request.item}",
            "item": request.item,
            "posture": posture.pos_name,
        }
    except Exception as e:
        print(f"Error executing pose: {e}")
        return {"status": "error", "message": str(e)}


def run_http_server():
    """Run FastAPI HTTP server in background thread."""
    uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT, log_level="info")


def main():
    global move_controller, postures_map

    try:
        move_controller = Move()
        print("Arm initialized")
    except Exception as e:
        print(f"Failed to initialize arm: {e}")

    print(f"\nConnecting to gateway server at {GATEWAY_HOST}:{GATEWAY_PORT}...")
    try:
        conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
        response = conn.gateway()

        if response is None:
            print("Failed to fetch posture")
            return

        # Store postures
        postures = [PosturesData(**item) for item in response]

        # Build postures map (item_name -> posture)
        for posture in postures:
            postures_map[posture.pos_name] = posture

        print(f"Loaded {len(postures)} postures:")
        for posture in postures:
            print(f"  - {posture.pos_name} (ID: {posture.pos_id})")
        print()

    except Exception as e:
        print(f"Error fetching postures: {e}")

    print("\n" + "=" * 25)
    print("Jetcobot Controller Ready")
    print("=" * 25 + "\n")

    # Start HTTP server in background thread
    print(f"Starting HTTP server on {HTTP_HOST}:{HTTP_PORT}")
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    # Keep main thread alive
    try:
        http_thread.join()
    except KeyboardInterrupt:
        print("\nShutting down...")


if __name__ == "__main__":
    main()
