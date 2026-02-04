from app.models.postures_model import Postures
from app.models.robots_model import RobotsData


def test_connection(data: RobotsData):
    print(f"{data.robot_id}")
    print(f"{data.namespace}")
    print(f"{data.robot_type}")
    print(f"{data.robot_name}")

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

    return pos.model_dump_json()
