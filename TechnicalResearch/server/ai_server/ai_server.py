import requests
from flask import Flask

app = Flask(__name__)

SERVER_URL = "http://192.168.0.56:8000"


def connect_to_server():
    try:
        response = requests.post(SERVER_URL + "/ai_server", timeout=5)
        print("status:", response.status_code)
        print("body:", response.text)
    except requests.RequestException as e:
        print("connection failed:", e)


if __name__ == "__main__":
    connect_to_server()  # connect first before running ai_server
    app.run(host="0.0.0.0", port=5000, debug=False)
