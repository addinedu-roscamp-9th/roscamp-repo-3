from app.models.robots_model import RobotsData


def test_connection(data: RobotsData):
    print(f'{data.robot_id}')
    print(f'{data.namespace}')
    print(f'{data.robot_type}')
    print(f'{data.robot_name}')
    return "Hello from jetcobot_service.py"
