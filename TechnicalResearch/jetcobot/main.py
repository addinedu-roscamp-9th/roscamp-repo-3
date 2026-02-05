import os

from dotenv import load_dotenv

from app.controller.movement import Move
from app.model.posture_model import PosturesData
from app.service.connect import Connect

# Load environment variables from .env file
load_dotenv()

# 1st args: value read from .env
# 2nd args: default value
GATEWAY_HOST = os.getenv("HOST", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("PORT", "8000"))
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
