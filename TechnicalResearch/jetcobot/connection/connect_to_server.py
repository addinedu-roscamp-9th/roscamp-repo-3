import json

import requests

with open("./config.json", encoding="UTF-8") as file:
    config = json.load(file)


SERVER_URL = config["server_url"]


def connect_to_server():
    try:
        response = requests.post(SERVER_URL + "/jetcobot", timeout=5)
        print("status:", response.status_code)
        print("body:", response.text)
    except requests.RequestException as e:
        print("connection failed:", e)


if __name__ == "__main__":
    connect_to_server()
