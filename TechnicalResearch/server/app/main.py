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

schedules = []


@app.on_event("startup")
async def select_schedules():
    global schedules
    try:
        schedules = schedule_mapper.select_all_schedules()
    except OperationalError as e:
        print(f"\nDatabase connection error: {e}")
    except SQLAlchemyError as e:
        print(f"\nDatabase error: {e}")


def get_schedule():
    return schedules
