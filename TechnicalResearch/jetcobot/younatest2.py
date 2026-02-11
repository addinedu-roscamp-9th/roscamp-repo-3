import base64
import json
import os
import socket
import threading
import time

import cv2
from dotenv import load_dotenv

load_dotenv()

AI_HOST = os.getenv("AI_HOST", "192.168.5.3")
AI_PORT = int(os.getenv("AI_PORT", "9000"))
RECV_PORT = 9001  # ⬅ 좌표 수신 포트


# ================= [좌표 수신 스레드] =================
def recv_coords():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", RECV_PORT))
    print("📥 좌표 수신 대기 중...")

    while True:
        data, _ = sock.recvfrom(1024)
        payload = json.loads(data.decode())
        print(
            f"\n🎯 인식 물체: {payload['class']} | "
            f"cx={payload['cx']} cy={payload['cy']}"
        )


threading.Thread(target=recv_coords, daemon=True).start()

# ================= [카메라 송신] =================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    send_frame = cv2.resize(frame, (320, 240))
    _, encoded = cv2.imencode(".jpg", send_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])

    payload = json.dumps({"image": base64.b64encode(encoded).decode("utf-8")})

    sock.sendto(payload.encode(), (AI_HOST, AI_PORT))

    now = time.time()
    fps = 1 / (now - prev_time)
    prev_time = now
    print(f"\r📡 Send FPS: {fps:.1f}", end="")
