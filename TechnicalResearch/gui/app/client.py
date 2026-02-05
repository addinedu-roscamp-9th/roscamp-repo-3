"""HTTP client — every call to the FastAPI server lives here.

No other module should import ``requests`` directly.  Exceptions from the
transport layer (``requests.exceptions.*``) are intentionally not caught so
that callers can decide how to surface them in the UI.
"""

from __future__ import annotations

import requests

from app.config import SERVER_URL


def move(destination: str) -> requests.Response:
    """POST a move command for robot 1."""
    return requests.post(
        f"{SERVER_URL}/gui",
        json={
            "robot_id": 1,
            "status": "moving",
            "command": "move",
            "destination": destination,
        },
    )


def stop() -> requests.Response:
    """POST an emergency-stop command for robot 1."""
    return requests.post(
        f"{SERVER_URL}/gui",
        json={"robot_id": 1, "status": "stopped", "command": "stop"},
    )


def bring_item(item: str, destination: str) -> requests.Response:
    """POST a bring-item command for robot 1."""
    return requests.post(
        f"{SERVER_URL}/gui",
        json={
            "robot_id": 1,
            "status": "bringing_item",
            "command": "bring_item",
            "destination": destination,
            "item": item,
        },
    )


def put_item(from_room: str) -> requests.Response:
    """POST a put-item command for robot 1."""
    return requests.post(
        f"{SERVER_URL}/gui",
        json={
            "robot_id": 1,
            "status": "putting_item",
            "command": "put_item",
            "destination": "pickup_zone",
            "from_room": from_room,
        },
    )


def execute_schedule(name: str, tasks: list[str]) -> requests.Response:
    """POST a schedule-execution command for robot 1."""
    return requests.post(
        f"{SERVER_URL}/gui",
        json={
            "robot_id": 1,
            "status": "executing_schedule",
            "command": "execute_schedule",
            "schedule_name": name,
            "tasks": tasks,
        },
    )


def get_robot_status(robot_id: str) -> requests.Response:
    """GET the current status for a single robot."""
    return requests.get(f"{SERVER_URL}/pinky/status/{robot_id}")
