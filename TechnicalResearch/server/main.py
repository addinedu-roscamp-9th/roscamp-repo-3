import uvicorn
from fastapi import FastAPI

from app.routers import gui, jetcobot, pinky
from app.websocket import websocket

HOST = "0.0.0.0"
PORT = 8000
DEBUG = True

app = FastAPI(title="Roboto Server", redirect_slashes=False)

app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
app.include_router(pinky.router, prefix="/pinky", tags=["pinky"])
app.include_router(gui.router, prefix="/gui", tags=["gui"])
app.include_router(websocket.router, tags=["websocket"])


if __name__ == "__main__":
    uvicorn.run("main:app", host=HOST, port=PORT, reload=DEBUG)
