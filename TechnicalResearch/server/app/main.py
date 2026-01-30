from fastapi import FastAPI

from app.routers import jetcobot

app = FastAPI(title="Roboto Server")

app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
