import os

from dotenv import load_dotenv

from app.controller.movement import Move
from app.model.model import PosturesData
from app.service.connect import Connect

# Load environment variables from .env file
load_dotenv()

GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.22")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", 8000))
ENDPOINT = os.getenv("ENDPOINT", "jetcobot")
SPEED = 30


def main():
    print("connect to server")
    conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
    response = conn.gateway()
    print(type(response))
    print(response)

    if response is None:
        print("Failed to connect to server")
        return

    # Parse response into PosturesData (response is already a dict from response.json())
    posture = PosturesData(**response)
    print(f"Received posture: {posture}")

    # Initialize robot and execute movement
    move = Move()
    move.execute(posture, SPEED)


if __name__ == "__main__":
    main()
