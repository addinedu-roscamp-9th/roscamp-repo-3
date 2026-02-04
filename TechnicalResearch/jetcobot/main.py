from app.service.connect import Connect

GATEWAY_HOST = "172.20.10.4"
GATEWAY_PORT = 8000
ENDPOINT = "jetcobot"

INITIAL_ANGLES = [0, 0, 0, 0, 0, 0]
SPEED = 30


def main():
    print("connect to server")
    conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
    response = conn.gateway()
    print(response)


if __name__ == "__main__":
    main()
