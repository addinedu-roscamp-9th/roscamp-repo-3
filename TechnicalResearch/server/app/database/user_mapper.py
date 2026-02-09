from app.database.connection import SessionLocal
from app.models.database import User


def select_user_by_id(user_id):
    db = SessionLocal()

    try:
        return db.query(User).filter(User.id == user_id).first()
    finally:
        db.close()
