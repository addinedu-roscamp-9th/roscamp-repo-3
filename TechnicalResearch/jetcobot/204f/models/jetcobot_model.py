from dataclasses import dataclass


@dataclass
class JetcobotData:
    robot_id: int
    status: str
