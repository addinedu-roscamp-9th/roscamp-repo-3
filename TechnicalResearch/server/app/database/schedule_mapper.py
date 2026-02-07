from app.database.connection import SessionLocal
from app.models.database import Schedule

db = SessionLocal()


def select_all_schedules():
    return db.query(Schedule).all()
