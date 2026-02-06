import os

# 서버 IP/PORT 환경 변수로 설정
os.environ["ROBOT_SERVER_HOST"] = "192.168.0.52"
os.environ["ROBOT_SERVER_PORT"] = "8000"

import uvicorn
from fastapi import FastAPI
from app.routers import gui, jetcobot, pinky

HOST = "0.0.0.0"
PORT = 8000
DEBUG = True

app = FastAPI(title="Roboto Server", redirect_slashes=False)

app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
app.include_router(pinky.router, prefix="/pinky", tags=["pinky"])
app.include_router(gui.router, prefix="/gui", tags=["gui"])


if __name__ == "__main__":
    uvicorn.run("main:app", host=HOST, port=PORT, reload=DEBUG)
