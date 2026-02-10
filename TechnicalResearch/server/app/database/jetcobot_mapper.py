from app.database.connection import SessionLocal
from app.models.tables import Posture


def select_all_pos():
    db = SessionLocal()
    try:
        return db.query(Posture).all()
    finally:
        db.close()
