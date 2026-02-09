from app.database.connection import SessionLocal
from app.models.database import Posture


def select_all_pos():
    db = SessionLocal()
    try:
        return db.query(Posture).all()
    finally:
        db.close()
