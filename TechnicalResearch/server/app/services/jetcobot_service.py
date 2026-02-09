from app.database import jetcobot_mapper
from app.models.postures_model import Postures
from app.models.robots_model import RobotsData


def test_connection(data: RobotsData):
    print("\n=== Data from Jetcobot ===\n")
    print(f"robot_id:   {data.robot_id}")
    print(f"namespace:  {data.namespace}")
    print(f"robot_type: {data.robot_type}")
    print(f"robot_name: {data.robot_name}")
    print()

    poses = jetcobot_mapper.select_all_pos()

    # Convert ORM models to Pydantic models and return as list of dicts
    result = [Postures.model_validate(pose).model_dump() for pose in poses]

    return result
