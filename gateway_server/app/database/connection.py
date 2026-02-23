from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

# Database configuration - CHANGE THESE IF NEEDED
DB_USER = "debugcrew"
DB_PASSWORD = "1234"
DB_NAME = "home_ai"

# Create connection string
DATABASE_URL = f"mysql+pymysql://{DB_USER}:{DB_PASSWORD}@localhost/{DB_NAME}"

# Create engine (connects to database)
engine = create_engine(DATABASE_URL)

# SessionLocal - use this to talk to the database
SessionLocal = sessionmaker(bind=engine)

# Base - use this for your models
Base = declarative_base()
