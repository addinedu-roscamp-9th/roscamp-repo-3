from app.models.postures_model import Postures
from app.models.robots_model import RobotsData


def test_connection(data: RobotsData):
    print("\n=== Data from Jetcobot ===\n")
    print(f"robot_id:   {data.robot_id}")
    print(f"namespace:  {data.namespace}")
    print(f"robot_type: {data.robot_type}")
    print(f"robot_name: {data.robot_name}")
    print()

    # TODO: query from DB
    pos = Postures(
        pos_id="p260205003",
        pos_name="trash_side",
        j1=-90,
        j2=40,
        j3=-90,
        j4=5,
        j5=4,
        j6=135,  # camera to bottom
        gap=0,  # completely open
    )

    return pos.model_dump()
