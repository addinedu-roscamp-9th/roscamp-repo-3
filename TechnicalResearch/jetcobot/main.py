import asyncio
import os

from dotenv import load_dotenv

from app.model.posture_model import PosturesData
from app.service.connect import Connect
from app.service.movement import Move

# Load environment variables from .env file
load_dotenv()

# 1st args: value read from .env
# 2nd args: default value
GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
ENDPOINT = os.getenv("ENDPOINT", "jetcobot")
SPEED = 30


def main():
    # initialize first
    move = Move()

    print("Connect to server ...")
    conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
    response = asyncio.run(conn.gateway())  # Run async function

    if response is None:
        print("Failed to connect to server")
        return

    posture = PosturesData(**response)
    print(f"Received posture: {posture}")

    move.execute(posture, SPEED)


if __name__ == "__main__":
    main()
