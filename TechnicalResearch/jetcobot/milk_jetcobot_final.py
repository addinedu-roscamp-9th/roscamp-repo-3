import base64
import datetime
import json
import socket
import threading
import time

import cv2

# pymycobot 버전에 따라 MyCobot 또는 MyCobot280을 사용합니다.
try:
    from pymycobot.mycobot280 import MyCobot280
except ImportError:
    from pymycobot.mycobot import MyCobot as MyCobot280

# ================= [1. 설정 데이터] =================
PC_IP = "192.168.5.3"
PC_PORT = 9000
STOP_SIGNAL_PORT = 9001  # PC로부터 STOP 신호를 받을 포트

# 📍 로봇 좌표 및 각도 설정
HOME_ANGLES = [0, 0, 0, 0, 0, -47.33]
READY_POS = [91.2, -49.5, 339.8, -144.65, -13.02, -43.64]
PICK_POS = [260.8, -17.2, 43.9, -170.5, 22.21, -37.29]
PLACE_POS = [38.3, -262.2, 267.2, -123.03, -22.08, -131.31]
SAFE_HIGH_ANGLES = [0, 0, 0, 0, 0, -47.33]
MID_POS = [-48.0, -31.3, 407.9, -92.85, -52.43, -157.51]

# 비상 정지 상태 변수
emergency_stop = False
last_stop_time = 0

# ================= [2. 로봇 연결] =================
mc = None
try:
    mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
    mc.power_on()
    time.sleep(1.0)
    mc.send_angles(HOME_ANGLES, 40)
    print("✅ 로봇 연결 성공 및 HOME 자세 이동 완료")
except Exception as e:
    print(f"❌ 연결 실패: {e}")

# ================= [3. 통신 및 제어 함수] =================


def check_pause():
    """비상 정지 신호가 왔는지 확인하고 대기합니다."""
    global emergency_stop, last_stop_time
    if emergency_stop and (time.time() - last_stop_time < 0.5):
        print("\n🛑 [비상 정지] 손 감지! 10초간 정지합니다...")
        time.sleep(10)
        emergency_stop = False
        print("🔄 동작을 재개합니다.")


def listen_stop_signal():
    """PC로부터 오는 STOP 신호를 무한 대기합니다."""
    global emergency_stop, last_stop_time
    stop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop_sock.bind(("0.0.0.0", STOP_SIGNAL_PORT))
    while True:
        try:
            data, _ = stop_sock.recvfrom(1024)
            if data.decode() == "STOP":
                emergency_stop = True
                last_stop_time = time.time()
        except:
            pass


threading.Thread(target=listen_stop_signal, daemon=True).start()

# ================= [4. 로봇 작업 시퀀스] =================


def robot_task():
    if mc is None:
        return
    time.sleep(5)

    try:
        check_pause()
        print("🔓 1. 그리퍼 개방 및 READY 위치 이동")
        mc.set_gripper_state(0, 50)
        mc.send_coords(READY_POS, 40, 1)
        time.sleep(4)

        check_pause()
        print("🚀 2. 물체 위치(PICK)로 이동")
        mc.send_coords(PICK_POS, 20, 1)
        time.sleep(6)

        check_pause()
        print("🔒 3. 물체 집기")
        mc.set_gripper_state(1, 50)
        time.sleep(2)

        check_pause()
        print("⬆️ 4. 최대 안전 높이 상승")
        mc.send_angles(SAFE_HIGH_ANGLES, 30)
        time.sleep(5)

        check_pause()
        print("🧭 4-1. 중간 경유 좌표 이동")
        mc.send_coords(MID_POS, 25, 1)
        time.sleep(4)

        check_pause()
        print(f"🚚 5. PLACE 이동 중...")
        mc.send_coords(PLACE_POS, 20, 1)
        time.sleep(5)

        check_pause()
        print("🔓 6. 물체 내려놓기")
        mc.set_gripper_state(0, 50)
        time.sleep(2)

        print("🏠 7. HOME 복귀")
        mc.send_angles(HOME_ANGLES, 30)
        time.sleep(5)

        # ✨ 모든 작업 종료
        print("✨ 모든 시퀀스 동작 완료!")

    except Exception as e:
        print(f"🚨 에러: {e}")


threading.Thread(target=robot_task, daemon=True).start()

# ================= [5. 영상 송신 루프] =================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"📡 영상 송신 중... (Target PC: {PC_IP})")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        _, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
        payload = json.dumps({"image": base64.b64encode(encoded).decode("utf-8")})
        video_sock.sendto(payload.encode(), (PC_IP, PC_PORT))
        time.sleep(0.07)
except KeyboardInterrupt:
    print("\n👋 종료합니다.")
finally:
    cap.release()
    video_sock.close()
