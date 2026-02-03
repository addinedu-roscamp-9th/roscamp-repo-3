import requests
from models.jetcobot_model import JetcobotData

SERVER_URL = "http://192.168.5.10:8000"


def connect_to_server():
    # TODO: refactor hardcoded values
    data = JetcobotData(robot_id=51, status="idle")

    try:
        response = requests.post(
            f"{SERVER_URL}/jetcobot",
            json=data.__dict__,  # convert data into dict
            timeout=5,
        )
        print("status:", response.status_code)
        print("body:", response.text)
    except requests.RequestException as e:
        print("connection failed:", e)


if __name__ == "__main__":
    connect_to_server()
