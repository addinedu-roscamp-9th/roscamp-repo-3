from app.models.robot_model import JetcobotData


def test_connection(data: JetcobotData):
    print(f"robot_id: {data.robot_id}")
    print(f"status: {data.status}")
    return "Hello from jetcobot_service.py"
