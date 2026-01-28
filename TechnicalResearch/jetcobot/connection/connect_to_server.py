import requests

SERVER_URL = "http://192.168.0.56:8000"


def connect_to_server():
    try:
        response = requests.post(SERVER_URL + "/jetcobot", timeout=5)
        print("status:", response.status_code)
        print("body:", response.text)
    except requests.RequestException as e:
        print("connection failed:", e)


if __name__ == "__main__":
    connect_to_server()
