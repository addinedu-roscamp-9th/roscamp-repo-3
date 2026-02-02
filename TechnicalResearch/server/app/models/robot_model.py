from dataclasses import dataclass

from pydantic import BaseModel


@dataclass
class RobotData(BaseModel):
    robot_id: int
    status: str
