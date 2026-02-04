from app.models.postures_model import Postures
from app.models.robots_model import RobotsData


def test_connection(data: RobotsData):
    print('\n=== Data from Jetcobot ===\n')
    print(f"robot_id:   {data.robot_id}")
    print(f"namespace:  {data.namespace}")
    print(f"robot_type: {data.robot_type}")
    print(f"robot_name: {data.robot_name}")
    print()

    pos = Postures(
        pos_id="foo",
        pos_name="bar",
        j1=0.0,
        j2=0.0,
        j3=0.0,
        j4=0.0,
        j5=0.0,
        j6=0.0,
        angle=-135, # camera to bottom
        gap=100,  # completely open
    )

    return pos.model_dump()
