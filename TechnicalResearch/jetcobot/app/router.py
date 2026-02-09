"""FastAPI router for jetcobot movement commands."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.model.posture_model import PosturesData

router = APIRouter()

# Global state (populated on startup)
move_controller = None
postures_db: dict[str, PosturesData] = {}


def set_move_controller(controller):
    """Set the global Move controller instance."""
    global move_controller
    move_controller = controller


def set_postures(postures: list[PosturesData]):
    """Store postures in memory by ID and name."""
    global postures_db
    postures_db.clear()
    for posture in postures:
        postures_db[posture.pos_id] = posture
        postures_db[posture.pos_name] = posture


class ExecutePostureRequest(BaseModel):
    """Request to execute a saved posture."""

    identifier: str  # pos_id or pos_name
    speed: int = 30


class MoveAnglesRequest(BaseModel):
    """Request to move to specific joint angles."""

    j1: int
    j2: int
    j3: int
    j4: int
    j5: int
    j6: int
    speed: int = 30


class SetGripperRequest(BaseModel):
    """Request to set gripper opening."""

    gap: int  # 0-100
    speed: int = 50


@router.post("/execute")
def execute_posture(request: ExecutePostureRequest):
    """Execute a saved posture by ID or name."""
    if move_controller is None:
        raise HTTPException(status_code=503, detail="Move controller not initialized")

    posture = postures_db.get(request.identifier)
    if posture is None:
        raise HTTPException(
            status_code=404, detail=f"Posture '{request.identifier}' not found"
        )

    move_controller.execute(posture, request.speed)
    return {
        "status": "success",
        "message": f"Executed posture: {posture.pos_name}",
        "posture": posture.model_dump(),
    }


@router.post("/move")
def move_angles(request: MoveAnglesRequest):
    """Move robot to specific joint angles."""
    if move_controller is None:
        raise HTTPException(status_code=503, detail="Move controller not initialized")

    # Create temporary PosturesData for movement
    temp_posture = PosturesData(
        pos_id="temp",
        pos_name="custom_move",
        j1=request.j1,
        j2=request.j2,
        j3=request.j3,
        j4=request.j4,
        j5=request.j5,
        j6=request.j6,
        gap=0,  # Don't change gripper
    )

    move_controller.send_angles(temp_posture, request.speed)
    return {
        "status": "success",
        "message": "Moved to custom angles",
        "angles": [
            request.j1,
            request.j2,
            request.j3,
            request.j4,
            request.j5,
            request.j6,
        ],
    }


@router.post("/gripper")
def set_gripper(request: SetGripperRequest):
    """Set gripper opening value."""
    if move_controller is None:
        raise HTTPException(status_code=503, detail="Move controller not initialized")

    if not 0 <= request.gap <= 100:
        raise HTTPException(
            status_code=400, detail="Gripper gap must be between 0 and 100"
        )

    move_controller.set_gripper(request.gap, request.speed)
    return {
        "status": "success",
        "message": f"Gripper set to {request.gap}",
        "gap": request.gap,
    }


@router.get("/postures")
def list_postures():
    """List all available postures."""
    # Return unique postures (since we store by both ID and name)
    unique_postures = {}
    for key, posture in postures_db.items():
        if posture.pos_id not in unique_postures:
            unique_postures[posture.pos_id] = posture

    return {
        "status": "success",
        "count": len(unique_postures),
        "postures": [p.model_dump() for p in unique_postures.values()],
    }


@router.get("/status")
def get_status():
    """Get jetcobot server status."""
    return {
        "status": "online",
        "controller_initialized": move_controller is not None,
        "postures_loaded": len(postures_db) > 0,
        "posture_count": len(set(p.pos_id for p in postures_db.values())),
    }
