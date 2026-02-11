import cv2
import base64
import json
import socket
import time
import threading
from pymycobot.mycobot import MyCobot

# --- 설정 ---
PC_IP = "192.168.5.3"
PC_PORT = 9000

# 📍 각도/좌표 설정
HOME_ANGLES = [5.36, 121.99, -140.09, -24.69, 6.41, -126.82]
READY_POS = [50.2, 116.0, 311.6, -160.96, -5.67, 48.23]
PICK_POS = [33.2, 241.5, 114.7, -168.73, 7.76, 45.65]
PLACE_POS = [265.2, 45.4, 42.8, -174.55, 14.04, -35.09]

mc = None
try:
    mc = MyCobot("/dev/ttyJETCOBOT", 1000000)
    mc.power_on()
    time.sleep(1.0)
    mc.send_angles(HOME_ANGLES, 40)
    time.sleep(3)
    print("✅ 로봇 연결 성공 및 HOME 자세 고정")
except Exception as e:
    print(f"⚠️ 연결 실패: {e}")


# =========================
# 🤖 로봇 동작 스레드
# =========================
def robot_pick_and_place_task():
    if mc is None:
        return
    time.sleep(5)

    try:
        print("🔓 1. 그리퍼 개방 및 READY 위치 이동")
        mc.set_gripper_state(0, 50)
        mc.send_coords(READY_POS, 40, 1)
        time.sleep(4)

        print("🚀 2. 물체 위치(PICK)로 이동")
        mc.send_coords(PICK_POS, 20, 1)
        time.sleep(6)

        print("🔒 3. 물체 집기")
        mc.set_gripper_state(1, 50)
        time.sleep(2)

        print("⬆️ 4. 안전 높이(READY) 복귀")
        mc.send_coords(READY_POS, 30, 1)
        time.sleep(4)

        print(f"🚚 5. 목표 장소(PLACE)로 이동 중...")
        mc.send_coords(PLACE_POS, 20, 1)
        time.sleep(10)

        print("\n🔓 6. 물체 내려놓기")
        mc.set_gripper_state(0, 50)
        time.sleep(2)

        print("🏠 7. 완전 초기 자세(HOME)로 복귀")
        mc.send_angles(HOME_ANGLES, 30)
        time.sleep(5)
        print("✨ 모든 시퀀스 동작 완료!")

    except Exception as e:
        print(f"🚨 에러: {e}")


t = threading.Thread(target=robot_pick_and_place_task, daemon=True)
t.start()

# =========================
# 📡 영상 송신 루프 (30 FPS 최적화)
# =========================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

prev_time = 0

try:
    print(f"📡 영상 송신 시작 (Target: 30 FPS) -> {PC_IP}")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 현재 FPS 계산
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
        prev_time = curr_time

        _, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        payload = json.dumps({"image": base64.b64encode(encoded).decode("utf-8")})
        sock.sendto(payload.encode(), (PC_IP, PC_PORT))

        print(f"\r📡 송신 중... [FPS: {fps:.1f}]", end="")

        # 30 FPS를 위해 대기 시간을 0.01로 단축
        time.sleep(0.01)
except Exception as e:
    print(f"\n🚨 송신 에러: {e}")
finally:
    cap.release()
    sock.close()
