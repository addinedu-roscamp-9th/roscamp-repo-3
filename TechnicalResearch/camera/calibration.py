import os

import cv2
import numpy as np

# ====================
# 체스보드 및 카메라 설정
# ====================

# 체스보드 내부 코너 개수 (가로 8, 세로6)
CHECKERBOARD = (8, 6)

# 체스보드 한칸의 실제 크기 (단위: meter)
SQUARE_SIZE = 0.0281

# 카메라 ID (0 = 기본 카메라)
CAMERA_ID = 0


# ====================
# 이미지 폴더 저장 및 생성
# ====================

# 캘리브레이션 이미지를 저장할 폴더 생성
os.makedirs("calibration_image", exist_ok=True)

# 저장된 이미지 개수 카운트
img_count = 0


# ====================
# 체스보드의 3D 실제 좌표 생성
# ====================

# 체스보드 코너 개수만큼 (x, y, z) 좌표 배열 생성
# z = 0 (평면 위에 놓여있다고 가정)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)

# (0,0), (1,0), (2,0) 형태의 2D 그리드 생성
objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)

# 이미지 체스보드 크기를 반영
objp *= SQUARE_SIZE


# ====================
# 캘리브레이션용 데이터 저장 리스트
# ====================

# 실제 3D 좌표 저장 리스트
objpoints = []
# 이미지 상의 2D 코너 좌표 저장 리스트
imgpoints = []

# ====================
# 카메라 열기
# ====================
cap = cv2.VideoCapture(CAMERA_ID)

print("체스보드를 화면에 비추고 'c'를 눌러 촬영하세요 (ESC 종료)")


# ====================
# 실시간 영상처리 루프
# ====================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 흑백 이미지로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 체스보드 코너 발견
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # 코너가 발견되면 화면에 표시
    if found:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, found)

    # 화면 출력
    cv2.imshow("Calibration", frame)

    # 키 입력 대기
    key = cv2.waitKey(1) & 0xFF

    # ====================
    # 'c' 키 입력 시 이미지 저장
    # ====================
    if key == ord("c"):
        if not found:
            print("Chessboard NOT detected")
            continue
        print(f"Image saved ({img_count})")

        # 코너 좌표를 서브픽셀 단위로 정밀화
        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )

        # 3D 실제 좌표와 2D 이미지 좌표 저장
        objpoints.append(objp)
        imgpoints.append(corners2)

        # 이미지 파일 저장
        cv2.imwrite(f"calibration_image/img_{img_count:02d}.jpg", frame)
        img_count += 1

    # ESC 키 입력 시 종료
    elif key == 27:
        break


# ====================
# 자원 해제(카메라 사용을 끝내고, 카메라 자원을 돌려준다.)
# ====================
cap.release()
cv2.destroyAllWindows()


# 카메라 캘리브레이션 수행
print("⚙ 카메라 캘리브레이션 계산 중...")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("캘리브레이션 완료")
print("Camera Matrix:\n", camera_matrix)
print("Dist Coeffs:\n", dist_coeffs)

# ====================
# 결과 저장
# ====================
np.savez("camera_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

print("camera_calibration.npz 저장 완료")
