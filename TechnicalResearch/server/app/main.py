from fastapi import FastAPI
from sqlalchemy.exc import OperationalError, SQLAlchemyError

from app.database import schedule_mapper
from app.routers import gui, jetcobot, pinky

# FastAPI 앱 생성
app = FastAPI(title="Roboto Server", redirect_slashes=False)

# 라우터 등록
app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
app.include_router(pinky.router, prefix="/pinky", tags=["pinky"])
app.include_router(gui.router, prefix="/gui", tags=["gui"])


@app.on_event("startup")
async def select_schedules():
    try:
        schedules = schedule_mapper.select_all_schedules()

        # TODO: execute the order when time comes

        print("=" * 30)
        print("query schedules")

        if not schedules:
            print("There is no schedule")
            return

        for schedule in schedules:
            print(f"schedule_id: {schedule.schedule_id}")
            print(f"cmd_id: {schedule.cmd_id}")
            print(f"item_id: {schedule.item_id}")
            print(f"position_id: {schedule.position_id}")
            print(f"execute_time: {schedule.execute_time}")
            print(f"cycle: {schedule.cycle}")
            print(f"on_weekends: {schedule.on_weekends}")

        print("=" * 30)
    except OperationalError as e:
        print(f"\nDatabase connection error: {e}")
    except SQLAlchemyError as e:
        print(f"\nDatabase error: {e}")
