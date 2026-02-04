import requests

from app.model.model import RobotsData

ROBOOT_ID = "id"
NAMESPACE = "namespace"
ROBOT_TYPE = "type"
ROBOT_NAME = "name"

data = RobotsData(
    robot_id=ROBOOT_ID,
    namespace=NAMESPACE,
    robot_type=ROBOT_TYPE,
    robot_name=ROBOT_NAME,
)


class Connect:
    def __init__(self, host, port, endpoint):
        self.is_connected = False
        self.host = host
        self.port = port
        self.endpoint = endpoint

    def gateway(self):
        try:
            url = f"http://{self.host}:{self.port}/{self.endpoint}"
            response = requests.post(url, json=data.model_dump(), timeout=10)

            if response.status_code == 200:
                print(f"Successfully connected to JetCobot at {url}")
                print(f"Response: {response.json()}")
                self.is_connected = True
                return response.json()

        except Exception as e:
            print(f"Connection failed: {e}")
        return None
