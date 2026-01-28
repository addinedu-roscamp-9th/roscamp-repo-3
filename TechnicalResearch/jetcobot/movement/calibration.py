# calibration.py
import cv2
import numpy as np
import glob
import json
import os

# ==========================================================
# [사용자 설정 구간] ★ 여기만 환경에 맞게 바꾸세요
# ==========================================================
CAM_INDEX = 0  # USB 카메라 번호 (0부터)
SAVE_DIR = "./calib_out"
os.makedirs(SAVE_DIR, exist_ok=True)

# (1) 체커보드 내부 코너 개수 (가로, 세로)  예: 9x6 내부코너
CHESSBOARD_SIZE = (8, 6)

# (2) 체커보드 한 칸의 실제 크기(mm)
SQUARE_SIZE_MM = 28.0

# (3) 체커보드 캘리브레이션용 이미지 경로
#     capture 후 images/*.jpg 형태로 저장했다고 가정
IMAGE_GLOB = "./images/*.png"

# (4) 기준 ArUco 마커(고정용) 설정
ARUCO_DICT = cv2.aruco.DICT_4X4_50
REF_MARKER_ID = 0          # 기준 마커 ID (테이블에 고정한 마커)
REF_MARKER_SIZE_MM = 30.0  # 기준 마커 한 변 길이(mm)

# (5) 기준 마커의 "Base 좌표계에서의 포즈" (아주 쉬운 버전: 회전은 일단 무시/단순화)
#     자로 측정해서 넣기: 마커 중심이 base에서 (x,y,z)에 있음
#     z는 보통 테이블 높이(또는 기준면)로 0에 두고 시작해도 됨(환경에 따라)
REF_MARKER_POS_IN_BASE_MM = np.array([200.0, 0.0, 0.0], dtype=np.float64)
# ==========================================================


def calibrate_camera_from_chessboard():
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM  # mm 단위

    objpoints = []
    imgpoints = []

    images = sorted(glob.glob(IMAGE_GLOB))
    if len(images) < 10:
        raise RuntimeError(f"체커보드 이미지가 너무 적어요. 최소 10장 이상 권장. 현재: {len(images)}장")

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        if ret:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            objpoints.append(objp)
            imgpoints.append(corners2)

    if len(objpoints) < 8:
        raise RuntimeError(f"체커보드 인식 성공 장수가 부족해요. 성공: {len(objpoints)}장 (8장 이상 권장)")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    if not ret:
        raise RuntimeError("카메라 캘리브레이션 실패")

    np.save(os.path.join(SAVE_DIR, "K.npy"), K)
    np.save(os.path.join(SAVE_DIR, "dist.npy"), dist)

    print("[OK] Camera intrinsics saved:", os.path.join(SAVE_DIR, "K.npy"), os.path.join(SAVE_DIR, "dist.npy"))
    print("K=\n", K)
    print("dist=\n", dist)
    return K, dist


def estimate_T_cb_from_reference_marker(K, dist):
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("카메라 열기 실패")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    print("\n[INFO] 기준 마커를 화면에 잘 보이게 두고 's'를 누르면 Camera→Base를 계산합니다. (q=종료)\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        corners, ids, _ = detector.detectMarkers(frame)
        vis = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        cv2.putText(vis, "Press 's' to solve T_cb from REF marker, 'q' to quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("calibration - ref marker", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        if key == ord('s'):
            if ids is None or REF_MARKER_ID not in ids.flatten():
                print("[WARN] 기준 마커가 안 보여요.")
                continue

            # 기준 마커의 pose in camera: T_cm
            idx = np.where(ids.flatten() == REF_MARKER_ID)[0][0]
            c = corners[idx][0].astype(np.float64)

            # 마커 3D 모델(중심 기준)
            half = REF_MARKER_SIZE_MM / 2.0
            obj = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float64)

            ok, rvec, tvec = cv2.solvePnP(obj, c, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                print("[WARN] solvePnP 실패")
                continue

            R_cm, _ = cv2.Rodrigues(rvec)

            # 아주 쉬운 버전:
            # 기준 마커의 base 위치만 알고(REF_MARKER_POS_IN_BASE_MM), 회전은 일단 "동일"하다고 가정
            # => T_bm에서 R_bm = I 로 둠
            R_bm = np.eye(3, dtype=np.float64)
            t_bm = REF_MARKER_POS_IN_BASE_MM.reshape(3, 1)

            # T_cb = T_bm * inv(T_cm)
            # inv(T_cm): R_mc = R_cm^T, t_mc = -R_cm^T * t_cm
            R_mc = R_cm.T
            t_mc = -R_cm.T @ tvec

            R_cb = R_bm @ R_mc
            t_cb = t_bm + (R_bm @ t_mc)

            np.save(os.path.join(SAVE_DIR, "R_cb.npy"), R_cb)
            np.save(os.path.join(SAVE_DIR, "t_cb.npy"), t_cb)

            print("[OK] Saved R_cb, t_cb to", SAVE_DIR)
            print("R_cb=\n", R_cb)
            print("t_cb(mm)=\n", t_cb.reshape(-1))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    K, dist = calibrate_camera_from_chessboard()
    estimate_T_cb_from_reference_marker(K, dist)
