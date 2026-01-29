from flask import Flask

app = Flask(__name__)


# connect jetcobot to http://192.168.0.xx:8000/jetcobot
@app.route("/jetcobot", methods=["POST"])
def check_connection():
    print("jetcobot connected to /jetcobot")
    return "ok", 200


# connect ai_server to http://192.168.0.xx:8000/ai_server
@app.route("/ai_server", methods=["POST"])
def ai_server_connection():
    print("ai server connected to /ai_server")
    return "ok", 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=False)  # open at http://192.168.0.xx:8000
