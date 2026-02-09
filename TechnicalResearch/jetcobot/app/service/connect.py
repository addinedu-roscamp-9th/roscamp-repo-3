import requests

from app.model.robot_model import RobotsData

ROBOT_ID = "id"
NAMESPACE = "namespace"
ROBOT_TYPE = "type"
ROBOT_NAME = "name"

data = RobotsData(
    robot_id=ROBOT_ID,
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
        """Connect to the gateway server via HTTP POST."""
        try:
            url = f"http://{self.host}:{self.port}/{self.endpoint}"
            response = requests.post(url, json=data.model_dump(), timeout=5)
            response.raise_for_status()

            self.is_connected = True
            result = response.json()
            print(f"Successfully connected to JetCobot at {url}")
            print(f"Response: {result}")
            return result

        except requests.exceptions.Timeout:
            print(f"Connection timeout: {url}")
        except requests.exceptions.ConnectionError as e:
            print(f"Connection failed: {e}")
        except requests.exceptions.HTTPError as e:
            print(f"HTTP error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

        self.is_connected = False
        return None
