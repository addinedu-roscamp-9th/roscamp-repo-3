from app.database.connection import SessionLocal
from app.models.database import User

db = SessionLocal()


def select_user_by_id(user_id):
    return db.query(User).filter(User.id == user_id).first()
