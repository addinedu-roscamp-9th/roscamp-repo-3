import json
from dataclasses import dataclass

import requests

with open("./config.json", encoding="UTF-8") as file:
    config = json.load(file)

SERVER_URL = config["server_url"]


@dataclass
class JetcobotData:
    robot_id: int
    status: str


def connect_to_server():
    # TODO: refactor ardcode name
    data = JetcobotData(robot_id=51, status="idle")

    try:
        response = requests.post(
            f"{SERVER_URL}/gui", json=data.__dict__, timeout=5  # convert to dict
        )
        print("status:", response.status_code)
        print("body:", response.text)
    except requests.RequestException as e:
        print("connection failed:", e)


if __name__ == "__main__":
    connect_to_server()
