from app.routers import jetcobot, pinky, gui
from fastapi import FastAPI

app = FastAPI(title="Roboto Server", redirect_slashes=False)

app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
app.include_router(pinky.router, prefix="/pinky", tags=["pinky"])
app.include_router(gui.router, prefix="/gui", tags=["gui"])
