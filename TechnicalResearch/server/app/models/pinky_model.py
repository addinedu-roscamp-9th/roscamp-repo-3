"""
Pinky Robot 데이터 모델
"""

import datetime

from pydantic import BaseModel
from sqlalchemy import TIMESTAMP, Column, Float, ForeignKey, Integer, String
from sqlalchemy.orm import declarative_base

Base = declarative_base()


# Pydantic models (for API request/response validation)
class RobotCommand(BaseModel):
    """로봇 명령 모델"""

    type: str
    distance_cm: int | None = None
    speed: float | None = None
    target_x: float | None = None
    target_y: float | None = None
    target_theta: float | None = None


class RobotStatus(BaseModel):
    """로봇 상태 모델"""

    robot_id: str
    battery: int
    x: float
    y: float


# SQLAlchemy model (for database)
class RobotStatusLog(Base):
    __tablename__ = "robot_status_logs"
    id = Column(Integer, primary_key=True, autoincrement=True)
    robot_id = Column(String(50), ForeignKey("robots.robot_id"))
    battery = Column(Integer)
    x = Column(Float)
    y = Column(Float)
    created_at = Column(TIMESTAMP, default=datetime.datetime.utcnow)
