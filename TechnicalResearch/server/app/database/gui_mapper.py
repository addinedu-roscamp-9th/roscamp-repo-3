from app.database.connection import SessionLocal
from app.models.tables import History, Item, Position, Posture, User


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


def take_info():
    db = SessionLocal()
    try:
        positions = db.query(Position).all()

        if positions is None:
            return None

        return positions

    finally:
        db.close()


def history_info():
    db = SessionLocal()
    try:
        return db.query(History).all()
    finally:
        db.close()


def select_posture_by_id(item_id):
    db = SessionLocal()
    try:
        return (
            db.query(
                Posture.j1,
                Posture.j2,
                Posture.j3,
                Posture.j4,
                Posture.j5,
                Posture.j6,
                Posture.gap,
            )
            .filter(item_id)
            .first()
        )
    finally:
        db.close()


def select_position_by_id(pos_id):
    db = SessionLocal()
    try:
        return (
            db.query(Position.x, Position.y, Position.w)
            .filter(Position.position_id == pos_id)
            .first()
        )
    finally:
        db.close()
