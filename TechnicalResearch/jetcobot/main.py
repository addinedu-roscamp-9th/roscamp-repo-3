from app.controller.movement import Move
from app.model.model import PosturesData
from app.service.connect import Connect

# on hotspot
GATEWAY_HOST = "172.20.10.4"
GATEWAY_PORT = 8000
ENDPOINT = "jetcobot"

INITIAL_ANGLES = [0, 0, 0, 0, 0, 0]
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

    # Parse response into PosturesData
    posture = PosturesData(**response)
    print(f"Received posture: {posture}")

    # Initialize robot and execute movement
    move = Move()
    move.execute(posture, SPEED)


if __name__ == "__main__":
    main()
