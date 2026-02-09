"""
pinky_cli.py

터미널에서 입력한 명령을
핑키 로봇 명령(JSON)으로 변환해서 서버로 전송하는 CLI 도구
"""

import requests

SERVER_URL = "http://192.168.0.52:8000/pinky/robot/cmd/pinky_01"

def main():
    print("핑키 CLI 시작")
    print("사용 가능한 명령: move")
    print("종료: exit")

    while True:
        # 터미널 입력 받기
        user_input = input(">> ").strip().lower()

        if user_input == "exit":
            print("종료합니다.")
            break

        # -------------------------
        # 명령 번역
        # -------------------------
        if user_input == "move":
            # 기계가 이해할 수 있는 명령
            cmd = {
                "type": "move",
                "distance_cm": 10
            }

            try:
                res = requests.post(SERVER_URL, json=cmd, timeout=1.0)
                print("명령 전송 완료:", res.json())
            except Exception as e:
                print("서버 전송 실패:", e)

        else:
            print("알 수 없는 명령입니다.")

if __name__ == "__main__":
    main()

