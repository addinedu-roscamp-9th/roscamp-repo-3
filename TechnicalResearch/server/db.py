from sqlalchemy import create_engine, Column, String, Integer, Float, TIMESTAMP, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import datetime

# -----------------------
# MySQL 연결
# -----------------------
DATABASE_URL = "mysql+pymysql://user:password@localhost:3306/robot_db"

engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(bind=engine)

Base = declarative_base()

# -----------------------
# robot_status_logs 모델
# -----------------------
class RobotStatusLog(Base):
    __tablename__ = "robot_status_logs"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    robot_id = Column(String(50), ForeignKey("robots.robot_id"))
    battery = Column(Integer)
    x = Column(Float)
    y = Column(Float)
    created_at = Column(TIMESTAMP, default=datetime.datetime.utcnow)
