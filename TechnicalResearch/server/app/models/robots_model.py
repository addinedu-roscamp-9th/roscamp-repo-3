from dataclasses import dataclass

from pydantic import BaseModel


@dataclass
class RobotsData(BaseModel):
    robot_id: str
    namespace: str
    robot_type: str
    robot_name: str
