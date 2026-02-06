from fastapi import FastAPI

from app.routers import gui, jetcobot, pinky

# FastAPI 앱 생성
app = FastAPI(title="Roboto Server", redirect_slashes=False)

# 라우터 등록
app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
app.include_router(pinky.router, prefix="/pinky", tags=["pinky"])
app.include_router(gui.router, prefix="/gui", tags=["gui"])
