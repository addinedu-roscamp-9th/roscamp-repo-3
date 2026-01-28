# motion.py
import time
import json
import os
import numpy as np
from pymycobot.mycobot280 import MyCobot280

# ==========================================================
# [사용자 설정 구간] ★ 여기만 환경에 맞게 바꾸세요(아직 안바꿈)
# ==========================================================
PORT = "/dev/ttyJETCOBOT"   # ★ JetCobot에서 쓰는 포트 확인
BAUD = 1000000
SPEED = 40

CALIB_DIR = "./calib_out"
TARGET_FILE = "./target.json"

# 로봇 대기 자세(안전한 곳)
HOME = [180, 0, 180, 180, 0, 0]  # [x,y,z,rx,ry,rz] ★ 환경에 맞게

# 집기 높이/접근 높이 (mm)
APPROACH_Z_OFFSET = 70.0  # 위에서 접근
GRASP_Z_OFFSET = 10.0     # 실제 집기(바닥/물체 높이에 맞춰 조절)

# 그리퍼 제어 (모델마다 API가 다를 수 있어, 일단 가장 흔한 형태)
GRIPPER_OPEN = 100   # ★ 모델에 따라 0~100 또는 반대일 수 있음
GRIPPER_CLOSE = 30   # ★ 물체 잡히는 값으로 튜닝

# 분리수거통 좌표(마커 id별 목적지)
# 예: 1=캔통, 2=플라스틱, 3=유리
DROP_POSE_BY_ID = {
    0: [250, -120, 150, 180, 0, 0],
    1: [250,    0, 150, 180, 0, 0],
    2: [250,  120, 150, 180, 0, 0],
}

# 카메라좌표(mm) → 로봇 base좌표(mm) 변환 후, 그리퍼 중심 보정(카메라가 그리퍼 위에 달려있으면 오프셋 필요)
# 아주 쉬운 버전: 일단 0으로 시작하고, 집기가 빗나가면 여기 값을 조절
GRIPPER_OFFSET_IN_BASE_MM = np.array([0.0, 0.0, 0.0], dtype=np.float64)
# ==========================================================


def load_T_cb():
    R_cb = np.load(os.path.join(CALIB_DIR, "R_cb.npy"))
    t_cb = np.load(os.path.join(CALIB_DIR, "t_cb.npy")).reshape(3, 1)
    return R_cb, t_cb


def cam_to_base(R_cb, t_cb, tvec_cam_mm):
    # p_b = R_cb * p_c + t_cb
    p_c = np.array(tvec_cam_mm, dtype=np.float64).reshape(3, 1)
    p_b = (R_cb @ p_c) + t_cb
    p_b = p_b.reshape(3)
    p_b = p_b + GRIPPER_OFFSET_IN_BASE_MM
    return p_b


def read_latest_target():
    if not os.path.exists(TARGET_FILE):
        return None
    try:
        with open(TARGET_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except:
        return None


def gripper_open(mc: MyCobot280):
    # 모델에 따라 함수가 다를 수 있음
    # 자주 쓰는: mc.set_gripper_value(value, speed)
    try:
        mc.set_gripper_value(GRIPPER_OPEN, 50)
    except:
        pass


def gripper_close(mc: MyCobot280):
    try:
        mc.set_gripper_value(GRIPPER_CLOSE, 50)
    except:
        pass


def move_pose(mc: MyCobot280, pose, speed=SPEED, wait=True):
    mc.send_coords(pose, speed, 0)
    if wait:
        time.sleep(2.0)


def pick_and_place(mc: MyCobot280, pick_xyz, drop_pose6):
    x, y, z = pick_xyz

    # 1) 접근(위에서)
    approach = [float(x), float(y), float(z + APPROACH_Z_OFFSET), 180, 0, 0]
    move_pose(mc, approach)

    # 2) 내려가서 집기
    grasp = [float(x), float(y), float(z + GRASP_Z_OFFSET), 180, 0, 0]
    move_pose(mc, grasp)

    gripper_close(mc)
    time.sleep(0.8)

    # 3) 다시 위로
    move_pose(mc, approach)

    # 4) 드롭 위치로 이동
    move_pose(mc, drop_pose6)

    # 5) 놓기
    gripper_open(mc)
    time.sleep(0.8)

    # 6) 홈 복귀
    move_pose(mc, HOME)


def main():
    R_cb, t_cb = load_T_cb()

    mc = MyCobot280(PORT, BAUD)
    time.sleep(1.0)

    gripper_open(mc)
    move_pose(mc, HOME)

    print("[INFO] target.json을 보면서 마커를 집어서 id별 위치로 옮깁니다. Ctrl+C 종료")

    while True:
        data = read_latest_target()
        if not data or "markers" not in data:
            time.sleep(0.1)
            continue

        # 가장 가까운(=tvec z가 작은) 마커를 하나 선택해서 처리 (쉬운 버전)
        candidates = []
        for m in data["markers"]:
            mid = m["id"]
            if mid in DROP_POSE_BY_ID:
                candidates.append(m)

        if not candidates:
            time.sleep(0.1)
            continue

        target = sorted(candidates, key=lambda m: m["tvec_mm"][2])[0]
        mid = target["id"]
        tvec_cam = target["tvec_mm"]

        # camera -> base
        p_b = cam_to_base(R_cb, t_cb, tvec_cam)

        drop_pose = DROP_POSE_BY_ID[mid]

        print(f"[PICK] id={mid} cam_tvec={tvec_cam} -> base_xyz={p_b.tolist()} -> drop={drop_pose}")

        # 실행
        pick_and_place(mc, p_b, drop_pose)

        # 같은 마커를 연속 집는 걸 방지하기 위한 쿨다운
        time.sleep(1.0)


if __name__ == "__main__":
    main()
