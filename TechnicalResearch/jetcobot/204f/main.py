import base64
import json
import socket
from pathlib import Path

import cv2
import requests
from model.model import RobotsData

GATEWAY_HOST = "http://192.168.0.56"
GATEWAY_PORT = 8000

AI_SERVER_HOST = "192.168.0.56"
AI_SERVER_PORT = 9000


def connect_to_gateway():
    data = RobotsData(
        robot_id="robot_id",
        namespace="name_space",
        robot_type="robot_type",
        robot_name="robot_name",
    )

    try:
        response = requests.post(
            f"{GATEWAY_HOST}:{GATEWAY_PORT}/jetcobot",
            json=data.model_dump(),
            timeout=5,
        )
        print("Gateway status:", response.status_code)
        print("Gateway body:", response.text)
        return response.status_code == 200
    except requests.RequestException as e:
        print("Gateway connection failed:", e)
        return False


def send_image_to_ai_server(img_path: Path):
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"Failed to read image: {img_path}")
        return None

    img = cv2.resize(img, (640, 480))
    _, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 50])
    img_b64 = base64.b64encode(encoded).decode()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5.0)

    try:
        request = json.dumps({"image": img_b64})
        print(f"Payload size: {len(request)} bytes")

        sock.sendto(request.encode(), (AI_SERVER_HOST, AI_SERVER_PORT))
        data, _ = sock.recvfrom(65535)
        response = json.loads(data.decode())
        print("AI Response:", response)
        return response
    except socket.timeout:
        print("AI server timeout - no response")
        return None
    finally:
        sock.close()


def main():
    print("Connecting to gateway server...")
    if not connect_to_gateway():
        print("Warning: Gateway connection failed, continuing anyway")

    print("\nReady for AI inference. Commands:")
    print("  <image_path> - Send image for inference")
    print("  q            - Quit")

    while True:
        user_input = input("\n> ").strip()

        if user_input.lower() == "q":
            print("Exiting")
            break

        if not user_input:
            continue

        img_path = Path(user_input)
        if not img_path.exists():
            img_path = Path(__file__).parent / user_input

        if not img_path.exists():
            print(f"Image not found: {user_input}")
            continue

        send_image_to_ai_server(img_path)


if __name__ == "__main__":
    main()
