# app/routers/pinky.py
import time

from fastapi import APIRouter
from pydantic import BaseModel

router = APIRouter()

# -------------------------
# 메모리 저장소(유지형)
# -------------------------
nav_goals: dict[str, dict] = (
    {}
)  # {robot_id: {"goal_id": int, "x":..., "y":..., "yaw":..., "ts":...}}
nav_results: dict[str, dict] = (
    {}
)  # {robot_id: {"goal_id": int, "status": "...", "message": "...", "ts":...}}


class NavGoal(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class NavResult(BaseModel):
    goal_id: int
    status: str  # "SUCCEEDED" / "FAILED" / "CANCELED" / "RUNNING"
    message: str | None = None


@router.post("/nav/goal/{robot_id}")
def set_nav_goal(robot_id: str, goal: NavGoal):
    """
    GUI가 목표 좌표를 보내면 서버가 저장한다.
    goal은 삭제하지 않고 유지한다.
    goal_id를 증가시켜서 '새 goal인지' 판별 가능하게 한다.
    """
    prev = nav_goals.get(robot_id)
    new_goal_id = (prev["goal_id"] + 1) if prev and "goal_id" in prev else 1

    nav_goals[robot_id] = {
        "goal_id": new_goal_id,
        "x": goal.x,
        "y": goal.y,
        "yaw": goal.yaw,
        "ts": time.time(),
    }
    return {"result": "ok", "robot_id": robot_id, "goal": nav_goals[robot_id]}


@router.get("/nav/goal/{robot_id}")
def get_nav_goal(robot_id: str):
    """
    Pinky가 주기적으로 가져간다.
    유지형이므로 항상 마지막 goal을 준다.
    """
    return nav_goals.get(robot_id, {"type": "idle"})


@router.post("/nav/result/{robot_id}")
def post_nav_result(robot_id: str, result: NavResult):
    """
    Pinky가 주행 결과를 서버로 되돌려준다(선택).
    """
    nav_results[robot_id] = {
        "goal_id": result.goal_id,
        "status": result.status,
        "message": result.message,
        "ts": time.time(),
    }
    return {"result": "ok"}


@router.get("/nav/result/{robot_id}")
def get_nav_result(robot_id: str):
    return nav_results.get(robot_id, {"type": "none"})
