import json

import websockets
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
        self.websocket = None

    async def gateway(self):
        try:
            url = f"ws://{self.host}:{self.port}/{self.endpoint}"
            async with websockets.connect(url) as websocket:
                self.websocket = websocket
                self.is_connected = True
                print(f"Successfully connected to JetCobot at {url}")

                # Send data
                await websocket.send(json.dumps(data.model_dump()))

                # Receive response
                response = await websocket.recv()
                result = json.loads(response)
                print(f"Response: {result}")
                return result

        except Exception as e:
            print(f"Connection failed: {e}")
        return None
