from app.database.connection import SessionLocal
from app.models.tables import Schedule


def select_all_schedules():
    db = SessionLocal()
    try:
        return db.query(Schedule).all()
    finally:
        db.close()
