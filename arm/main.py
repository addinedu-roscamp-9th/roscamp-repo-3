import os
import threading
from typing import List

import uvicorn
from dotenv import load_dotenv
from fastapi import FastAPI

from app.model.posture import Posture
from app.service.connect_gateway import Connect
from app.service.move import Move

load_dotenv()

GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
ENDPOINT = os.getenv("ENDPOINT", "jetcobot")

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8001

move = None

app = FastAPI()


@app.post("/pose")
async def handle_pose(req: List[Posture]):
    print(req)


def run_http_server():
    uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT, log_level="info")


def main() -> None:
    global move

    try:
        move = Move()
    except Exception as e:
        print(f"Failed to initialize arm: {e}")

    try:
        connect = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
        connect.gateway()

        print("Connected to gateway server")
    except Exception as e:
        print(f"Failed to connect to gateway server: {e}")

    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()
    try:
        http_thread.join()
    except KeyboardInterrupt:
        print("\nShutting down...")


if __name__ == "__main__":
    main()
