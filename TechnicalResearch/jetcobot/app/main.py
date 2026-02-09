import os

from dotenv import load_dotenv
from fastapi import FastAPI

from app.model.posture_model import PosturesData
from app.routers import router
from app.service.connect import Connect
from app.service.movement import Move

# Load environment variables
load_dotenv()

# Gateway server configuration
GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")
GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
ENDPOINT = os.getenv("ENDPOINT", "jetcobot")

# Create FastAPI app
app = FastAPI(
    title="Jetcobot Controller",
    description="FastAPI server for controlling MyCobot280 arm",
)

# Register router
app.include_router(jetcobot.router, prefix="/api", tags=["movement"])


@app.on_event("startup")
async def startup_event():
    try:
        move = Move()
        set_move_controller(move)
        print("Arm initialized")
    except Exception as e:
        print(f"Failed to initialize arm: {e}")
        move = None

    print(f"\nConnecting to gateway server at {GATEWAY_HOST}:{GATEWAY_PORT}...")
    try:
        conn = Connect(GATEWAY_HOST, GATEWAY_PORT, ENDPOINT)
        response = conn.gateway()

        if response is None:
            print("✗ Failed to fetch postures from gateway server")
            print("  Server will start but no postures will be available")
            return

        # Store postures
        postures = [PosturesData(**item) for item in response]
        set_postures(postures)

        print(f"✓ Loaded {len(postures)} postures:")
        for posture in postures:
            print(f"  - {posture.pos_name} (ID: {posture.pos_id})")

    except Exception as e:
        print(f"✗ Error fetching postures: {e}")
        print("  Server will start but no postures will be available")

    print("=" * 50)
    print("Jetcobot Controller Ready")
    print("=" * 50)
