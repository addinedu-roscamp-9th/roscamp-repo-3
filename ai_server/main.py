import base64
import json
import os
import socket
import sys
import time

import cv2
import numpy as np
from dotenv import load_dotenv
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from ultralytics import YOLO

import mediapipe as mp

load_dotenv()

AI_HOST = "0.0.0.0"
AI_PORT = 9000

# http://192.168.0.56:8000/detection
GATEWAY_IP = os.getenv("GATEWAY_IP", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
GATEWAY_ENDPOINT = os.getenv("GATEWAY_ENDPOINT", "detection")

ARM_IP = os.getenv("ARM_IP", "192.168.5.1")
ARM_PORT = int(os.getenv("ARM_PORT", "9001"))

YOLO_MODEL_PATH = "./model/best.pt"
HAND_LANDMARKER_PATH = "./mediapipe/hand_landmarker.task"

UDP_SIZE = (2**16) - 1  # 65535 - max UDP datagram size

try:
    yolo_model = YOLO(YOLO_MODEL_PATH)
    print("yolo loaded")

    base_options = mp_python.BaseOptions(model_asset_path=HAND_LANDMARKER_PATH)
    hand_options = vision.HandLandmarkerOptions(
        base_options=base_options,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    hands_model = vision.HandLandmarker.create_from_options(hand_options)
    print("mediapipe loaded")
except Exception as e:
    print(f"Failed to load models: {e}")
    sys.exit()

# socket settings
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
sock.bind((AI_HOST, AI_PORT))
sock.setblocking(False)

stop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# for calculating fps
prev_time = 0
process_fps = 0


while True:
    try:
        data = None  # reset data
        while True:
            try:
                data, _ = sock.recvfrom(UDP_SIZE)  # assign latest packet
            except BlockingIOError:
                break
        if data is None:
            continue

        # turn it into img
        payload = json.loads(data.decode())
        img_bytes = base64.b64decode(payload["image"])
        frame = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)

        if frame is None:
            continue

        # fps calculation
        curr_time = time.time()
        if prev_time != 0:
            process_fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        # object detection
        yolo_results = yolo_model.predict(source=frame, conf=0.6, verbose=False)

        # hand detection (new Tasks API)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        hand_results = hands_model.detect(mp_image)

        # yolo detection visualisation
        for r in yolo_results:
            for box in r.boxes:
                # bbox corner grid
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                # bbox center grid
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cls_name = yolo_model.names[int(box.cls[0])]
                # draw bbox outline
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # draw dot at the center of bbox
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                # visualize grid and classification
                label = f"{cls_name} ({cx}, {cy})"
                cv2.putText(
                    frame,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

        # stop on hand detection
        if hand_results.hand_landmarks:
            stop_sock.sendto(b"STOP", (ARM_IP, ARM_PORT))

            cv2.putText(
                frame,
                "Emergency stop",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                3,
            )

        # print fps
        cv2.putText(
            frame,
            f"FPS: {process_fps:.1f}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 0, 0),
            2,
        )

        cv2.imshow("Live feeds from jetcobot", frame)

    except Exception as e:
        print(f"Error: {e}")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

sock.close()
stop_sock.close()
cv2.destroyAllWindows()
