import os
from typing import List

import httpx
from dotenv import load_dotenv

from app.database.repositories import gui_repository, schedule_repository
from app.models.tables import History
from app.services import pinky_service

load_dotenv()

JETCOBOT_HOST = os.getenv("JETCOBOT_HOST", "192.168.0.56")
JETCOBOT_PORT = int(os.getenv("JETCOBOT_PORT", "8001"))

POS_DZ = "drop zone"
POS_CHARGER = "charger"

ANGLE_SHELVE_SIDE = "shelve side"
ANGLE_PINKY_SIDE = "pinky side"
ANGLE_DROP = "drop"
ANGLE_HOME = "home"
ANGLE_TRASH_SIDE = "trash side"
ANGLE_TRASH_GENERAL = "trash general"


def login(data):
    user_id = data["id"]
    user_pw = data["pw"]

    user_name = gui_repository.login_user(user_id, user_pw)

    return {"user_name": user_name}


def fetch_info():
    items, positions = gui_repository.fetch_info()
    print(f"items: {items}")
    print(f"positions: {positions}")

    if items is None or positions is None:
        return {"items": [], "positions": []}

    return {
        "items": [
            {
                "item_id": i.item_id,
                "item_name": i.item_name,
                "amount": i.amount,
                "frequency": i.frequency,
            }
            for i in items
        ],
        "positions": [
            {
                "position_id": p.position_id,
                "position_name": p.position_name,
            }
            for p in positions
        ],
    }


def _create_angles(angle, gap):
    return {
        "j1": angle.j1,
        "j2": angle.j2,
        "j3": angle.j3,
        "j4": angle.j4,
        "j5": angle.j5,
        "j6": angle.j6,
        "gap": gap,
    }


def _drop_sequence():
    drop_angle = gui_repository.sel_angle_by_angle_name(ANGLE_DROP)
    home_angle = gui_repository.sel_angle_by_angle_name(ANGLE_HOME)

    return [
        _create_angles(drop_angle, gap=30),
        _create_angles(home_angle, gap=30),
    ]


def _pick_sequence(item_id):
    shelve_side = gui_repository.sel_angle_by_angle_name(ANGLE_SHELVE_SIDE)
    pinky_side = gui_repository.sel_angle_by_angle_name(ANGLE_PINKY_SIDE)
    pick_angle = gui_repository.sel_angle_by_item_id(item_id)

    return [
        _create_angles(shelve_side, gap=100),
        _create_angles(pick_angle, gap=0),
        _create_angles(shelve_side, gap=0),
        _create_angles(pinky_side, gap=0),
    ]


NAV_TIMEOUT = 60  # seconds per navigation leg


def nav_pinky(pos) -> bool:
    """Send a navigation goal and block until the robot reaches it."""
    return pinky_service.nav_pinky(pos, timeout=NAV_TIMEOUT)


def cmd_arm(sequence) -> bool:
    jetcobot_url = f"http://{JETCOBOT_HOST}:{JETCOBOT_PORT}/pose"

    try:
        with httpx.Client(timeout=60.0) as client:
            response = client.post(jetcobot_url, json=sequence)
            response.raise_for_status()
            result = response.json()
            is_success: bool = result["success"]
            print(f"Jetcobot response: {result}")
            return is_success
    except httpx.HTTPError as e:
        print(f"Error sending to jetcobot: {e}")
        return False


def fetch_cmd(data):
    item_id = data.get("item_id")
    position_id = data.get("position_id")

    dz_pos = gui_repository.sel_pos_by_name(POS_DZ)
    if not nav_pinky(dz_pos):
        print("Pinky failed to reach DZ")
        return {"success": False}

    pick_seq = _pick_sequence(item_id)
    if not cmd_arm(pick_seq):
        print(f"Arm failed to pick up {item_id}")
        return {"success": False}

    drop_seq = _drop_sequence()
    if not cmd_arm(drop_seq):
        print("Arm failed to drop item to pinky")
        return {"success": False}

    target_pos = gui_repository.sel_position_by_id(position_id)
    if target_pos is None:
        print(f"{position_id} not found")
        return {"success": False}
    if not nav_pinky(target_pos):
        print("Pinky failed to reach target position")
        return {"success": False}

    return {"success": True}


def fetch_confirm():
    charger_pos = gui_repository.sel_pos_by_name(POS_CHARGER)
    if not nav_pinky(charger_pos):
        return {"success": False}
    return {"success": True}


def take_info():
    positions = gui_repository.take_info()
    if positions is None:
        return []
    return {
        "position": [
            {
                "position_id": p.position_id,
                "position_name": p.position_name,
            }
            for p in positions
        ]
    }


def take_cmd(data):
    position_id = data.get("position_id")
    target_pos = gui_repository.sel_position_by_id(position_id)
    if not nav_pinky(target_pos):
        print("Pinky failed to reach target")
        return {"success": False}
    return {"success": True}


def _trash_sequence():
    pinky_side = gui_repository.sel_angle_by_angle_name(ANGLE_PINKY_SIDE)
    drop_angle = gui_repository.sel_angle_by_angle_name(ANGLE_DROP)
    trash_side = gui_repository.sel_angle_by_angle_name(ANGLE_TRASH_SIDE)
    trash_general = gui_repository.sel_angle_by_angle_name(ANGLE_TRASH_GENERAL)
    home_angle = gui_repository.sel_angle_by_angle_name(ANGLE_HOME)

    return [
        _create_angles(pinky_side, 100),
        _create_angles(drop_angle, 0),
        _create_angles(pinky_side, 0),
        _create_angles(trash_side, 0),
        _create_angles(trash_general, 100),
        _create_angles(home_angle, 100),
    ]


def take_confirm():
    dz_pos = gui_repository.sel_pos_by_name(POS_DZ)

    take_res = nav_pinky(dz_pos)

    if not take_res:
        print("Pinky failed to reach DZ")
        return {"success": False}

    # TODO: trash dynamically
    trash_seq = _trash_sequence()
    if not cmd_arm(trash_seq):
        print("Arm failed to trash")
        return {"success": False}

    charger_pos = gui_repository.sel_pos_by_name(POS_CHARGER)
    if not nav_pinky(charger_pos):
        print("pinky failed to go to charger")
        return {"success": False}

    return {"success": True}


def schedule_info():
    schedules = schedule_repository.select_all_schedules()

    schedule_list = []

    for s in schedules:
        schedule_list.append(
            {
                "schedule_id": s.schedule_id,
                "cmd_id": s.cmd_id,
                "item_id": s.item_id,
                "position_id": s.position_id,
                "execute_time": s.execute_time.strftime("%H:%M:%S"),
                "cycle": s.cycle,
                "on_weekends": s.on_weekends,
            }
        )

    return {"schedules": schedule_list}


def schedule_edit(data):
    is_success = False
    result = None

    action = data["action"]
    match action:
        case "add":
            result = schedule_repository.insert_schedule(data)
        case "edit":
            result = schedule_repository.update_schedule(data)
        case "delete":
            result = schedule_repository.delete_schedule(data)
        case _:
            pass

    if result is not None:
        is_success = True

    return {
        "success": is_success,
    }


def history_info():
    histories: List[History] = gui_repository.history_info()

    return {
        "histories": [
            {
                "history_id": h.history_id,
                "user_id": h.user_id,
                "item_id": h.item_id,
                "robot_1": h.robot_1,
                "robot_2": h.robot_2,
                "position_id": h.position_id,
                "schedule_id": h.schedule_id,
                "detection_type": h.detection_type,
                "time_start": h.time_start,
                "time_end": h.time_end,
                "is_successful": h.is_successful,
            }
        ]
        for h in histories
    }
