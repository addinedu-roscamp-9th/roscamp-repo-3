from app.routers import jetcobot
from fastapi import FastAPI

app = FastAPI(title="Roboto Server", redirect_slashes=False)

app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
