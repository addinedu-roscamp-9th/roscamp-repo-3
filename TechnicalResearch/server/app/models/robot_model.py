from pydantic import BaseModel


class RobotData(BaseModel):
    robot_id: int
    status: str
