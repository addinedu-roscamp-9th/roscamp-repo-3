import base64
import json
import socket
from collections import deque

import cv2
import numpy as np
from ultralytics import YOLO

# ================= [캘리브레이션 데이터 로드] =================
# try:
#    K = np.load("K.npy")
#    dist = np.load("dist.npy")
#    R_cb = np.load("R_cb.npy")
#    t_cb = np.load("t_cb.npy")
#    print("✅ 캘리브레이션 파일 4개 로드 성공")
# except Exception as e:
#    print(f"❌ 파일 로드 실패: {e}")
#    exit()

# ================= [설정 데이터] =================
PC_PORT = 9000
JETCOBOT_IP = "192.168.5.1"  # ⬅ JetCobot IP
JETCOBOT_PORT = 9001  # ⬅ 좌표 수신 포트

MODEL_PATH = "./model/best.pt"

CONF_THRESHOLD = 0.35
IOU_THRESHOLD = 0.45
IMG_SIZE = 640

AVG_FRAMES = 20
history = {}

# ================= [모델 로드] =================
model = YOLO(MODEL_PATH)
CLASS_NAMES = model.names
print("✅ 모델 로드 성공")

# ================= [UDP 설정] =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PC_PORT))
sock.setblocking(False)

coord_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ⬅ 좌표 송신용

# ================= [메인 루프] =================
while True:
    try:
        data, _ = sock.recvfrom(65535)
        payload = json.loads(data.decode())
        img_bytes = base64.b64decode(payload["image"])
        frame = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)

        if frame is None:
            continue

        results = model.predict(
            source=frame,
            conf=CONF_THRESHOLD,
            iou=IOU_THRESHOLD,
            imgsz=IMG_SIZE,
            augment=True,
            verbose=False,
        )

        current_detected_classes = []

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue

            class_groups = {}
            for box in r.boxes:
                cls_id = int(box.cls[0])
                coords = box.xyxy[0].cpu().numpy()
                class_groups.setdefault(cls_id, []).append(coords)

            for cls_id, box_list in class_groups.items():
                current_detected_classes.append(cls_id)
                boxes = np.array(box_list)

                raw_x1, raw_y1 = np.min(boxes[:, 0]), np.min(boxes[:, 1])
                raw_x2, raw_y2 = np.max(boxes[:, 2]), np.max(boxes[:, 3])

                if cls_id not in history:
                    history[cls_id] = deque(maxlen=AVG_FRAMES)
                history[cls_id].append([raw_x1, raw_y1, raw_x2, raw_y2])

                avg_coords = np.mean(history[cls_id], axis=0)
                fx1, fy1, fx2, fy2 = map(int, avg_coords)
                cx, cy = (fx1 + fx2) // 2, (fy1 + fy2) // 2

                cls_name = CLASS_NAMES[cls_id]

                # ===== 화면 표시 (기존 그대로) =====
                cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), (255, 50, 50), 3)
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)

                label = f"{cls_name} | X:{cx} Y:{cy}"
                cv2.putText(
                    frame,
                    label,
                    (fx1, fy1 - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 0),
                    4,
                )
                cv2.putText(
                    frame,
                    label,
                    (fx1, fy1 - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

                # ===== 🔴 추가된 부분: 좌표 UDP 전송 =====
                coord_payload = json.dumps({"class": cls_name, "cx": cx, "cy": cy})
                coord_sock.sendto(coord_payload.encode(), (JETCOBOT_IP, JETCOBOT_PORT))

        cv2.imshow("JetCobot Real-time AI Tracker", frame)

    except BlockingIOError:
        pass
    except Exception as e:
        print(f"\n🚨 에러: {e}")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
sock.close()
coord_sock.close()
