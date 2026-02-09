from fastapi import APIRouter

router = APIRouter()


@router.post("/pose")
def execute_posture():
    return True
