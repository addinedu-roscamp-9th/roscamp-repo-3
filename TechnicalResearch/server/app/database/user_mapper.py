from app.database.connection import SessionLocal
from app.models.database import User


def login_user(user_id, user_pw):
    db = SessionLocal()
    try:
        user = (
            db.query(User)
            .filter(User.user_id == user_id, User.user_pw == user_pw)
            .first()
        )

        if user is None:
            return None

        return user.user_name

    finally:
        db.close()
