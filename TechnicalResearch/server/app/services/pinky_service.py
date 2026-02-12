# app/services/pinky_service.py
from __future__ import annotations

from typing import Any, Mapping

# 서버 명령 저장소(robot_commands)에 접근하기 위해 라우터 모듈을 import
# (pinky.py 안에 robot_commands = {} 가 있음)
from app.routers import pinky as pinky_router

DEFAULT_ROBOT_ID = "pinky_01"


def _to_float(value: Any, field_name: str) -> float:
    """
    value를 float으로 안전 변환.
    - None이면 ValueError
    - 숫자/숫자문자열은 float으로 변환
    """
    if value is None:
        raise ValueError(f"'{field_name}' is missing (None)")
    try:
        return float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(
            f"'{field_name}' must be convertible to float. got={value!r}"
        ) from e


def send_position(
    position: Mapping[str, Any] | None, robot_id: str = DEFAULT_ROBOT_ID
) -> dict[str, Any]:
    """
    GUI/테스트에서 들어온 position(x,y,theta)을 Pinky가 polling으로 가져갈 cmd로 변환해 저장한다.

    기대 입력:
        position = {"x": 1.5, "y": 2.0, "theta": 0.0}

    저장 cmd(예):
        {"type": "goto_pose", "x": 1.5, "y": 2.0, "theta": 0.0}

    반환:
        {"result": "ok", "robot_id": "...", "cmd": {...}}
        또는 {"result":"error", ...}
    """
    # 1) position 존재 확인
    if position is None:
        return {"result": "error", "reason": "position is None"}

    # 2) 필수 키 확인 + float 변환
    try:
        x = _to_float(position.get("x"), "x")
        y = _to_float(position.get("y"), "y")
        theta = _to_float(position.get("theta", 0.0), "theta")
    except ValueError as e:
        return {"result": "error", "reason": str(e), "position": dict(position)}

    # 3) Pinky가 소비할 명령 포맷 생성
    #    (Pinky 쪽에서 type == "goto_pose" 처리를 추가해야 실제로 좌표 이동됨)
    cmd: dict[str, Any] = {"type": "goto_pose", "x": x, "y": y, "theta": theta}

    # 4) Polling 구조 A: 서버 저장소에 등록
    pinky_router.robot_commands[robot_id] = cmd

    return {"result": "ok", "robot_id": robot_id, "cmd": cmd}
