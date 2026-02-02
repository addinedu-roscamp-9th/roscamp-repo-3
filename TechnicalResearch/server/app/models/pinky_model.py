import datetime
from dataclasses import dataclass

from pydantic import BaseModel
from sqlalchemy import TIMESTAMP, Column, Float, ForeignKey, Integer, String
from sqlalchemy.orm import declarative_base

Base = declarative_base()


# Pydantic model (for API request/response validation)
@dataclass
class RobotStatus(BaseModel):
    robot_id: str
    battery: int
    x: float
    y: float


# SQLAlchemy model (for database)
@dataclass
class RobotStatusLog(Base):
    __tablename__ = "robot_status_logs"
    id = Column(Integer, primary_key=True, autoincrement=True)
    robot_id = Column(String(50), ForeignKey("robots.robot_id"))
    battery = Column(Integer)
    x = Column(Float)
    y = Column(Float)
    created_at = Column(TIMESTAMP, default=datetime.datetime.utcnow)
