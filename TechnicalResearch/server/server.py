from flask import Flask

app = Flask(__name__)


@app.route("/jetcobot", methods=["POST"])
def check_connection():
    print("jetcobot connected to /jetcobot")
    return "ok", 200


@app.route("/ai_server", methods=["POST"])
def ai_server_connection():
    print("ai server connected to /ai_server")
    return "ok", 200


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=False)
