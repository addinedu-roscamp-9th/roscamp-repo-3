from dataclasses import dataclass

from pydantic import BaseModel


@dataclass
class JetcobotData(BaseModel):
    robot_id: int
    status: str
