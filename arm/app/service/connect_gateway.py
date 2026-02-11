import requests


class Connect:
    def __init__(self, host, port, endpoint):
        self.is_connected = False
        self.host = host
        self.port = port
        self.endpoint = endpoint

    def gateway(self):
        try:
            url = f"http://{self.host}:{self.port}/{self.endpoint}"

            response = requests.post(
                url, json={"msg": "connect", "data": {}}, timeout=5
            )

            self.is_connected = True
            result = response.json()

            print(f"Connected to {url}")
            print(f"Response: {result}")
            return result

        except Exception as e:
            print(f"Failed to connect to gateway: {e}")

        self.is_connected = False
        return None
