from app.database import jetcobot_mapper
from app.models.postures_model import Postures


def jetcobot_connection():
    poses = jetcobot_mapper.select_all_pos()

    # Convert ORM models to Pydantic models and return as list of dicts
    result = [Postures.model_validate(pose).model_dump() for pose in poses]

    return result
