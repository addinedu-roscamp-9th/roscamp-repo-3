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


def main():
    # initialize the arm
    Move()

    print("Connect to server ...")
    conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
    response = asyncio.run(conn.gateway())  # Run async function

    if response is None:
        print("Failed to connect to server")
        return

    # Store postures in a list (like ArrayList in Java)
    postures = [PosturesData(**item) for item in response]

    print(f"Stored {len(postures)} postures:")
    for posture in postures:
        print(f"  - {posture.pos_name} (ID: {posture.pos_id})")

    # Now you can access postures by index or iterate through them
    # Example usage:
    # move.execute(postures[0], SPEED)  # pinky_side
    # move.execute(postures[1], SPEED)  # shelve_side
    # move.execute(postures[2], SPEED)  # trash_side


if __name__ == "__main__":
    main()
