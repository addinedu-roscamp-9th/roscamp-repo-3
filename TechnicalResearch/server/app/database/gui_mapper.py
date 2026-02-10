from app.database.connection import SessionLocal
from app.models.tables import Item, Position, User


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


def fetch_info():
    db = SessionLocal()
    try:
        print("Inside fetch_info()")
        items = db.query(Item).all()
        positions = db.query(Position).all()

        if items and positions is None:
            return None, None

        return items, positions

    finally:
        db.close()
