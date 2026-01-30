from app.models.robot_model import RobotData


def test_connection(data: RobotData):
    print(f"robot_id: {data.robot_id}")
    print(f"status: {data.status}")
    return "Hello from jetcobot_service.py"
