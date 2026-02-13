import cv2
import base64
import json
import socket
import numpy as np
import time
import sys
from ultralytics import YOLO

# --- 설정 ---
PC_PORT = 9000
ROBOT_IP = "192.168.5.1"
STOP_SIGNAL_PORT = 9001
YOLO_MODEL_PATH = "/home/youna/dev_ws/jetcobot/best.pt"

# 1. 모델 및 MediaPipe 로드
try:
    import mediapipe.python.solutions.hands as mp_hands
    import mediapipe.python.solutions.drawing_utils as mp_drawing

    yolo_model = YOLO(YOLO_MODEL_PATH)
    hands_model = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    print("✅ YOLO 및 MediaPipe 모델 로드 완료")
except Exception as e:
    print(f"❌ 로드 실패: {e}")
    sys.exit()

# 2. 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
sock.bind(("0.0.0.0", PC_PORT))
sock.setblocking(False)

stop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# FPS 계산 변수
prev_time = 0
process_fps = 0

print("🖥️ 통합 분석 서버 시작 (YOLO 좌표 출력 모드)...")

while True:
    try:
        data = None
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except BlockingIOError:
                break

        if data is None:
            continue

        payload = json.loads(data.decode())
        img_bytes = base64.b64decode(payload["image"])
        frame = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)

        if frame is None:
            continue

        # --- ⏱️ FPS 계산 ---
        curr_time = time.time()
        if prev_time != 0:
            process_fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        # --- AI 분석 ---
        # A. YOLO 분석 (물체)
        yolo_results = yolo_model.predict(source=frame, conf=0.6, verbose=False)

        # B. MediaPipe 분석 (손)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hand_results = hands_model.process(rgb_frame)

        # --- 🎯 1. YOLO 결과 시각화 (좌표 및 중앙점 추가) ---
        for r in yolo_results:
            for box in r.boxes:
                # 경계 박스 좌표
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                # 📍 중앙점 계산
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                cls_name = yolo_model.names[int(box.cls[0])]

                # 경계 상자 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # 중앙점 그리기 (빨간색 점)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # 좌표 및 이름 텍스트 출력
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

        # --- ✋ 2. 손 인식 및 비상 정지 ---
        if hand_results.multi_hand_landmarks:
            stop_sock.sendto(b"STOP", (ROBOT_IP, STOP_SIGNAL_PORT))

            for hand_landmarks in hand_results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(
                        color=(255, 0, 0), thickness=2, circle_radius=2
                    ),
                    mp_drawing.DrawingSpec(color=(255, 100, 100), thickness=2),
                )

            cv2.putText(
                frame,
                "!!! EMERGENCY STOP !!!",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                3,
            )

        # --- 📺 3. 화면 FPS 출력 ---
        cv2.putText(
            frame,
            f"FPS: {process_fps:.1f}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 0, 0),
            2,
        )

        cv2.imshow("JetCobot Integrated AI View", frame)

    except Exception as e:
        print(f"🚨 에러 발생: {e}")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

sock.close()
stop_sock.close()
cv2.destroyAllWindows()
