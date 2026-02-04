from pydantic import BaseModel


class RobotsData(BaseModel):
    robot_id: str
    namespace: str
    robot_type: str
    robot_name: str
