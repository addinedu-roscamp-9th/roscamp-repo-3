import os

from dotenv import load_dotenv
from fastapi import FastAPI

from app.model.posture_model import PosturesData
from app.routers import api
from app.service.connect import Connect
from app.service.movement import Move

load_dotenv()

GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
ENDPOINT = os.getenv("ENDPOINT", "jetcobot")

app = FastAPI(title="Roboto Arm")

app.include_router(api.router, prefix="/api", tags=["api"])


@app.on_event("startup")
async def startup_event():
    try:
        Move()
        print("Arm initialized")
    except Exception as e:
        print(f"Failed to initialize arm: {e}")

    print(f"\nConnecting to gateway server at {GATEWAY_HOST}:{GATEWAY_PORT}...")
    try:
        conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
        response = conn.gateway()

        if response is None:
            print("Failed to fetch posture")
            return

        # Store postures
        postures = [PosturesData(**item) for item in response]

        print(f"Loaded {len(postures)} postures:")
        for posture in postures:
            print(f"  - {posture.pos_name} (ID: {posture.pos_id})")
        print()

    except Exception as e:
        print(f"Error fetching postures: {e}")

    print("\n" + "=" * 25)
    print("Jetcobot Controller Ready")
    print("=" * 25 + "\n")
