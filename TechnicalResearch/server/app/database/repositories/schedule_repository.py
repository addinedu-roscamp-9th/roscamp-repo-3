import random
from datetime import datetime

from app.database.connection import SessionLocal
from app.models.tables import Schedule


def select_all_schedules():
    db = SessionLocal()
    try:
        return db.query(Schedule).all()
    finally:
        db.close()


def insert_schedule(data):
    db = SessionLocal()
    try:
        execute_time_str = data.get("execute_time")
        execute_time = datetime.strptime(execute_time_str, "%H:%M:%S").time()
        # schedule_id 자동 생성: s + YYMMDD + 4자리 랜덤 숫자 (예: s2602220001)
        date_part = datetime.now().strftime("%y%m%d")
        schedule_id = f"s{date_part}{random.randint(1000, 9999)}"

        new_schedule = Schedule(
            schedule_id=schedule_id,
            cmd_id=data.get("cmd_id"),
            item_id=data.get("item_id"),
            position_id=data.get("position_id"),
            execute_time=execute_time,
            cycle=data.get("cycle", 1),
            on_weekends=data.get("on_weekends", False),
        )

        db.add(new_schedule)
        db.commit()
        db.refresh(new_schedule)

        print("INSERT SUCCESS:", new_schedule.schedule_id)

        return new_schedule

    except Exception as e:
        db.rollback()
        print("INSERT ERROR:", e)
        return None

    finally:
        db.close()


def update_schedule(data):
    db = SessionLocal()
    try:
        schedule_id = data.get("schedule_id")
        if not schedule_id:
            return None

        schedule = (
            db.query(Schedule).filter(Schedule.schedule_id == schedule_id).first()
        )
        if not schedule:
            return None

        if "cmd_id" in data:
            schedule.cmd_id = data["cmd_id"]
        if "item_id" in data:
            schedule.item_id = data["item_id"]
        if "position_id" in data:
            schedule.position_id = data["position_id"]
        if "execute_time" in data:
            schedule.execute_time = data["execute_time"]
        if "cycle" in data:
            schedule.cycle = data["cycle"]
        if "on_weekends" in data:
            schedule.on_weekends = data["on_weekends"]

        db.commit()
        db.refresh(schedule)
        return schedule
    except Exception as e:
        db.rollback()
        raise e
    finally:
        db.close()


def delete_schedule(data):
    db = SessionLocal()
    try:
        schedule_id = data.get("schedule_id")
        if not schedule_id:
            return None

        schedule = (
            db.query(Schedule).filter(Schedule.schedule_id == schedule_id).first()
        )
        if not schedule:
            return None

        db.delete(schedule)
        db.commit()
        return schedule
    except Exception as e:
        db.rollback()
        raise e
    finally:
        db.close()
