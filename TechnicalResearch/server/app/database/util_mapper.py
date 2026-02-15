from app.database.connection import SessionLocal
from app.models.tables import Angle, Position


def select_dz_pos():
    db = SessionLocal()
    try:
        return db.query(Position).filter(Position.position_name == "drop zone").first()
    finally:
        db.close()


def select_position_by_id(pos_id):
    db = SessionLocal()
    try:
        return db.query(Position).filter(Position.position_id == pos_id).first()
    finally:
        db.close()


def select_shelve_side_angle():
    db = SessionLocal()
    try:
        return db.query(Angle).filter(Angle.pos_name == "shelve side").first()
    finally:
        db.close()


def select_angle_by_item_id(item_id):
    db = SessionLocal()
    try:
        return db.query(Angle).filter(Angle.item_id == item_id).first()
    finally:
        db.close()


def select_pinky_side_angle():
    db = SessionLocal()
    try:
        return db.query(Angle).filter(Angle.pos_name == "pinky_side").first()
    finally:
        db.close()

def select_drop_angle():
    db = SessionLocal()
    try:
        return db.query(Angle).filter(Angle.pos_name == "drop").first()
    finally:
        db.close()

def select_home_angle():
    db = SessionLocal()
    try:
        return db.query(Angle).filter(Angle.pos_name == "home").first()
    finally:
        db.close()
