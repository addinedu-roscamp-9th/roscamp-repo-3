import os

from dotenv import load_dotenv
from fastapi import FastAPI

from app.model.posture_model import PosturesData
from app.router import router, set_move_controller, set_postures
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
app.include_router(router, prefix="/api", tags=["movement"])


@app.on_event("startup")
async def startup_event():
    """Initialize robot and fetch postures from gateway server."""
    print("=" * 50)
    print("Jetcobot Controller Starting Up")
    print("=" * 50)

    # Initialize the arm
    print("Initializing MyCobot280 arm...")
    try:
        move = Move()
        set_move_controller(move)
        print("✓ Arm initialized successfully")
    except Exception as e:
        print(f"✗ Failed to initialize arm: {e}")
        print("  Server will start but movement commands will fail")
        move = None

    # Connect to gateway server and fetch postures
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


@app.get("/")
def root():
    """Root endpoint with API information."""
    return {
        "service": "Jetcobot Controller",
        "version": "1.0.0",
        "endpoints": {
            "status": "GET /api/status",
            "postures": "GET /api/postures",
            "execute": "POST /api/execute",
            "move": "POST /api/move",
            "gripper": "POST /api/gripper",
        },
    }


@app.get("/health")
def health():
    """Health check endpoint."""
    return {"status": "healthy"}
