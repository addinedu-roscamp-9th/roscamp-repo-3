import cv2
import numpy as np

# .waitKey()에 hardcoded 1을 매개변수로 주는 대신 전역 상수 선언 후 사용
#  대문자는 상수라는 뜻
#  자료형은 int로 지정
ESC_KEY: int = 1

# ====================
# 카메라 캘리브레이션 결과 불러오기
# ====================

# camera_calibration.npz 파일에는 카메라 내부 파라미터랑 외곡 계수가 저장되어 있음
data = np.load("camera_calibration.npz")

# 카메라 내부 행렬 (Camera Matrix)
K = data["camera_matrix"]

# 왜곡 계수 (Distortion Coefficients)
# 렌즈의 배럴 왜곡, 핀쿠션 왜곡 보정용
D = data["dist_coeffs"]


# ====================
# 카메라 열기
# ====================

# 0번 카메라(기본 카메라)
cap = cv2.VideoCapture(0)

# 카메라 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ESC 키를 누르면 프로그램이 종료 (안내 메시지 출력)
print("Press ESC to exit")


# ====================
# 실시간 영상 처리 루프
# ====================

while True:
    # 카메라부터 한 프레임(영상) 읽기
    ret, frame = cap.read()
    # 카메라부터 한 프레임(영상)을 못받으면 종료
    if not ret:
        break

    # 원본 카메라 파라미터를 그래도 쓰면서 영상이 깨지거나 잘리지 않게 왜곡만 보정하는 방식
    # SAFE undistortion (no new camera matrix)

    # 원본 영상(frame)을 카메라 행렬(K)과 왜곡 계수(D)를 이용해 왜곡이 제거된 영상으로 반환
    undistorted = cv2.undistort(frame, K, D)

    # ====================
    # 영상 출력
    # ====================

    # 원본 영상
    cv2.imshow("Original", frame)
    # 왜곡 보정된 영상
    cv2.imshow("Undistorted", undistorted)

    # esc키(27)를 누르면 종료
    if cv2.waitKey(ESC_KEY) & 0xFF == 27:
        break


# ====================
# 자원 해제(카메라 사용을 끝내고, 카메라 자원을 돌려준다.)
# ====================
cap.release()
cv2.destroyAllWindows()
