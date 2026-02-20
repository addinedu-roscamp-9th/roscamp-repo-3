import base64
import datetime
import json
import socket
import threading
import time

import cv2
from pymycobot.mycobot280 import MyCobot280

# ================= [1. 설정 데이터] =================
PC_IP = "192.168.5.3"
JETCOBOT_PORT = 9000  # 영상 송신용 포트
STOP_SIGNAL_PORT = 9001  # PC로부터 STOP 신호를 받을 포트

# 📍 로봇 좌표 및 각도 설정
HOME_ANGLES = [0, 0, 0, 0, 0, -47.33]
READY_POS = [47.8, 53.6, 323.9, -147.83, -3.83, 48.32]
PICK_POS = [-13.8, 273.3, 119.2, -170.71, 17.92, 48.2]
PLACE_POS = [260.8, -17.2, 43.9, -170.5, 22.21, -37.29]

# 상태 제어 변수
emergency_stop = False
last_stop_time = 0  # 마지막 STOP 신호 수신 시간

# ================= [2. 로봇 연결] =================
mc = None
try:
    mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
    mc.power_on()
    time.sleep(1.0)
    mc.send_angles(HOME_ANGLES, 40)
    print("✅ 로봇 연결 성공 및 초기 위치 이동 완료")
except Exception as e:
    print(f"❌ 로봇 연결 실패: {e}")

# ================= [3. 비상 정지 로직] =================


def check_pause():
    global emergency_stop, last_stop_time
    current_time = time.time()

    # 0.5초 이내의 신선한 신호일 때만 정지
    if emergency_stop and (current_time - last_stop_time < 0.5):
        print(
            f"\n🛑 [비상 정지] 손 감지됨! ({datetime.datetime.now().strftime('%H:%M:%S')})"
        )
        print("⏳ 10초 대기 후 동작을 재개합니다...")
        time.sleep(10)
        print("🔄 대기 종료. 동작 재개.")
        emergency_stop = False
    else:
        emergency_stop = False


def listen_stop_signal():
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
    try:
        time.sleep(5)

        check_pause()
        print("📦 1. READY 위치 이동")
        mc.set_gripper_state(0, 50)
        mc.send_coords(READY_POS, 40, 1)
        time.sleep(4)

        check_pause()
        print("🚀 2. PICK 위치 이동")
        mc.send_coords(PICK_POS, 20, 1)
        time.sleep(6)

        check_pause()
        print("🔒 3. 물체 집기")
        mc.set_gripper_state(1, 50)
        time.sleep(2)

        check_pause()
        print("⬆️ 4. 안전 높이 복귀")
        mc.send_coords(READY_POS, 30, 1)
        time.sleep(4)

        check_pause()
        print("🚚 5. PLACE 위치 이동")
        mc.send_coords(PLACE_POS, 20, 1)
        time.sleep(5)

        check_pause()
        print("🔓 6. 물체 놓기")
        mc.set_gripper_state(0, 50)
        time.sleep(2)

        check_pause()
        print("🏠 7. HOME 귀환")
        mc.send_angles(HOME_ANGLES, 30)
        time.sleep(5)

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

        # 이미지 압축 및 인코딩
        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
        payload = json.dumps({"image": base64.b64encode(buffer).decode("utf-8")})

        # UDP 전송
        video_sock.sendto(payload.encode(), (PC_IP, JETCOBOT_PORT))

        # CPU 부하 감소를 위한 미세 대기
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n👋 프로그램을 종료합니다.")
finally:
    if cap:
        cap.release()
    video_sock.close()
