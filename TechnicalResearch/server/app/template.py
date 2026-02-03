from sqlalchemy import (
    TIMESTAMP,
    Column,
    Float,
    ForeignKey,
    Integer,
    String,
    create_engine,
)
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

# connect to db
DATABASE_URL = "mysql+pymysql://user:password@localhost:3306/robot_db"

# db name : debugcrew
# username: debugcrew
# pw      : debug_crew_1234
# DATABASE_URL = "mysql+pymysql://debugcrew:debug_crew_1234@localhost:3306/debugcrew"

engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(bind=engine)

Base = declarative_base()
