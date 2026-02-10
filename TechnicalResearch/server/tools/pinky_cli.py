"""
핑키에게 전달한 명령 입력창 실행
명령은 서버를 통해 핑키에게 전달된다
"""

import requests

SERVER_URL = "http://192.168.0.52:8000/pinky/robot/cmd/pinky_01"


def main():
    print("핑키 CLI 시작")
    print("사용 가능한 명령: move")
    print("종료: exit")

    while True:
        user_input = input(">> ").strip().lower()

        if user_input == "exit":
            print("종료합니다.")
            break

        if user_input == "move":
            cmd = {"type": "move", "distance_cm": 10}

            try:
                res = requests.post(SERVER_URL, json=cmd, timeout=1.0)
                print("명령 전송 완료:", res.json())
            except Exception as e:
                print("서버 전송 실패:", e)

        else:
            print("알 수 없는 명령입니다.")


if __name__ == "__main__":
    main()
