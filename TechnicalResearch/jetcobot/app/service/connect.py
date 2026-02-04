import requests


class Connect:
    def __init__(self, host, port, endpoint):
        self.is_connected = False
        self.host = host
        self.port = port
        self.endpoint = endpoint

    def gateway(self, robot_id="jetcobot-001", namespace="home", robot_type="mycobot280", robot_name="JetCobot"):
        try:
            url = f"http://{self.host}:{self.port}/{self.endpoint}"
            payload = {
                "robot_id": robot_id,
                "namespace": namespace,
                "robot_type": robot_type,
                "robot_name": robot_name
            }
            response = requests.post(url, json=payload, timeout=10)

            if response.status_code == 200:
                print(f"Successfully connected to JetCobot at {url}")
                print(f"Response: {response.json()}")
                self.is_connected = True
                return response.json()

        except Exception as e:
            print(f"Connection failed: {e}")
        return None
