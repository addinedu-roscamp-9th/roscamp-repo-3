from app.database.connection import SessionLocal
from app.models.tables import Angle, Position


def select_dz_pos():
    db = SessionLocal()
    try:
        return (
            db.query(Position.x, Position.y, Position.w)
            .filter(Position.position_name == "drop zone")
            .first()
        )
    finally:
        db.close()


def select_charger_pos():
    db = SessionLocal()
    try:
        return (
            db.query(Position.x, Position.y, Position.w)
            .filter(Position.position_name == "charger")
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


def select_shelve_side_angle():
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.angle_name == "shelve side")
            .first()
        )
    finally:
        db.close()


def select_angle_by_item_id(item_id):
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.item_id == item_id)
            .first()
        )
    finally:
        db.close()


def select_pinky_side_angle():
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.angle_name == "pinky side")
            .first()
        )
    finally:
        db.close()


def select_drop_angle():
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.angle_name == "drop")
            .first()
        )
    finally:
        db.close()


def select_home_angle():
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.angle_name == "home")
            .first()
        )
    finally:
        db.close()

def select_trash_angle():
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.angle_name == "trash side")
            .first()
        )
    finally:
        db.close()

def select_trash_general():
    db = SessionLocal()
    try:
        return (
            db.query(
                Angle.j1,
                Angle.j2,
                Angle.j3,
                Angle.j4,
                Angle.j5,
                Angle.j6,
            )
            .filter(Angle.angle_name == "trash general")
            .first()
        )
    finally:
        db.close()
