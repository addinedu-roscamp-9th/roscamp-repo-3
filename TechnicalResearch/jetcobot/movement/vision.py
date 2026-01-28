# vision.py
import cv2
import numpy as np
import json
import time
import os

# ==========================================================
# [사용자 설정 구간] ★ 여기만 환경에 맞게 바꾸세요
# ==========================================================
CAM_INDEX = 0
CALIB_DIR = "./calib_out"
OUT_FILE = "./target.json"

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE_MM = 30.0  # 물체에 붙인 마커 한 변 길이(mm)  ★실제와 맞추기
# ==========================================================


def load_calib():
    K = np.load(os.path.join(CALIB_DIR, "K.npy"))
    dist = np.load(os.path.join(CALIB_DIR, "dist.npy"))
    return K, dist


def detect_markers_and_save():
    K, dist = load_calib()

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("카메라 열기 실패")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    print("[INFO] 마커가 보이면 target.json으로 저장됩니다. (q=종료)")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        corners, ids, _ = detector.detectMarkers(frame)
        vis = frame.copy()

        result = {
            "timestamp": time.time(),
            "markers": []
        }

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            for i, mid in enumerate(ids.flatten()):
                c = corners[i][0].astype(np.float64)

                half = MARKER_SIZE_MM / 2.0
                obj = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float64)

                ok, rvec, tvec = cv2.solvePnP(obj, c, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue

                # tvec: camera 좌표계(mm)
                result["markers"].append({
                    "id": int(mid),
                    "tvec_mm": [float(tvec[0]), float(tvec[1]), float(tvec[2])],
                    "rvec": [float(rvec[0]), float(rvec[1]), float(rvec[2])]
                })

        # 저장(항상 최신값 유지)
        with open(OUT_FILE, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)

        cv2.putText(vis, f"Saved: {OUT_FILE} (markers={len(result['markers'])})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("vision", vis)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_markers_and_save()
