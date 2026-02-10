from sqlalchemy import Boolean, Column, Float, ForeignKey, Integer, String, Time

from app.database.connection import Base


class User(Base):
    __tablename__ = "users"

    user_id = Column(String(30), primary_key=True)
    user_pw = Column(String(60), nullable=False)
    user_name = Column(String(30), nullable=False)


class RobotType(Base):
    __tablename__ = "robot_types"

    robot_type = Column(String(30), primary_key=True)


class Robot(Base):
    __tablename__ = "robots"

    robot_id = Column(String(11), primary_key=True)
    namespace = Column(String(20), nullable=False)
    robot_type = Column(
        String(30), ForeignKey("robot_types.robot_type"), nullable=False
    )
    robot_name = Column(String(30), nullable=False)


class Detection(Base):
    __tablename__ = "detections"

    detection_type = Column(String(11), primary_key=True)
    confidence = Column(Float, default=0.0)
    is_reported = Column(Boolean, default=False)
    img_path = Column(String(100))


class Item(Base):
    __tablename__ = "items"

    item_id = Column(String(11), primary_key=True)
    item_name = Column(String(30), nullable=False)
    amount = Column(Integer, default=0)
    frequency = Column(Integer, default=0)


class Command(Base):
    __tablename__ = "commands"

    cmd_id = Column(String(11), primary_key=True)
    cmd_type = Column(String(30), nullable=False)


class Position(Base):
    __tablename__ = "positions"

    position_id = Column(String(11), primary_key=True)
    position_name = Column(String(30), nullable=False)
    x = Column(Float, nullable=False)
    y = Column(Float, nullable=False)
    theta = Column(Float, nullable=False)


class Posture(Base):
    __tablename__ = "postures"

    pos_id = Column(String(11), primary_key=True)
    pos_name = Column(String(30), nullable=False)
    j1 = Column(Float, nullable=False)
    j2 = Column(Float, nullable=False)
    j3 = Column(Float, nullable=False)
    j4 = Column(Float, nullable=False)
    j5 = Column(Float, nullable=False)
    j6 = Column(Float, nullable=False)
    gap = Column(Integer, nullable=False)


class Schedule(Base):
    __tablename__ = "schedules"

    schedule_id = Column(String(11), primary_key=True)
    cmd_id = Column(String(11), ForeignKey("commands.cmd_id"))
    item_id = Column(String(11), ForeignKey("items.item_id"))
    position_id = Column(String(11), ForeignKey("positions.position_id"))
    execute_time = Column(Time, nullable=False)
    cycle = Column(Integer, default=1)
    on_weekends = Column(Boolean, default=False)


class History(Base):
    __tablename__ = "history"

    history_id = Column(String(11), primary_key=True)
    user_id = Column(String(30), ForeignKey("users.user_id"))
    item_id = Column(String(11), ForeignKey("items.item_id"))
    robot_1 = Column(String(11), ForeignKey("robots.robot_id"))
    robot_2 = Column(String(11), ForeignKey("robots.robot_id"))
    position_id = Column(String(11), ForeignKey("positions.position_id"))
    cmd_id = Column(String(11), ForeignKey("commands.cmd_id"))
    schedule_id = Column(String(11), ForeignKey("schedules.schedule_id"))
    detection_type = Column(String(11), ForeignKey("detections.detection_type"))
    time_start = Column(Time, nullable=False)
    time_end = Column(Time, nullable=False)
    is_successful = Column(Boolean, default=False)
