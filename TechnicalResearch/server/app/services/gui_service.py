import os
from typing import List

import httpx
from dotenv import load_dotenv

from app.database import gui_mapper, schedule_mapper, util_mapper
from app.models.tables import History
from app.services import pinky_service

load_dotenv()

JETCOBOT_HOST = os.getenv("JETCOBOT_HOST", "192.168.0.56")
JETCOBOT_PORT = int(os.getenv("JETCOBOT_PORT", "8001"))


def login(data):
    user_id = data["id"]
    user_pw = data["pw"]

    user_name = gui_mapper.login_user(user_id, user_pw)

    return {"user_name": user_name}


def fetch_info():
    items, positions = gui_mapper.fetch_info()
    print(f"items: {items}")
    print(f"positions: {positions}")

    if items is None or positions is None:
        return {"items": [], "positions": []}

    return {
        "items": [
            {
                "item_id": item.item_id,
                "item_name": item.item_name,
                "amount": item.amount,
                "frequency": item.frequency,
            }
            for item in items
        ],
        "positions": [
            {
                "position_id": pos.position_id,
                "position_name": pos.position_name,
                "x": pos.x,
                "y": pos.y,
                "theta": pos.theta,
            }
            for pos in positions
        ],
    }


def create_angles(angle, gap):
    return {
        "j1": angle.j1,
        "j2": angle.j2,
        "j3": angle.j3,
        "j4": angle.j4,
        "j5": angle.j5,
        "j6": angle.j6,
        "gap": gap,
    }


def drop_sequence():
    drop_angle = util_mapper.select_drop_angle()
    home_angle = util_mapper.select_home_angle()

    return [
        create_angles(drop_angle, gap=100),
        create_angles(home_angle, gap=100),
    ]


def pick_sequence(item_id):
    shelve_side = util_mapper.select_shelve_side_angle()
    pinky_side = util_mapper.select_pinky_side_angle()
    pick_angle = util_mapper.select_angle_by_item_id(item_id)

    return [
        create_angles(shelve_side, gap=100),
        create_angles(pick_angle, gap=0),
        create_angles(shelve_side, gap=0),
        create_angles(pinky_side, gap=0),
    ]


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

    dz_pos = util_mapper.select_dz_pos()
    pinky_to_dz = pinky_service.cmd_pinky(dz_pos)
    if pinky_to_dz is False:
        print("Pinky failed to reach DZ")

    pick_seq = pick_sequence(item_id)
    arm_pickup_res = cmd_arm(pick_seq)
    if arm_pickup_res is False:
        print(f"Arm failed to pick up {item_id}")

    if pinky_to_dz and arm_pickup_res is False:
        return "foobar"

    drop_seq = drop_sequence()
    arm_drop_res = cmd_arm(drop_seq)
    if arm_drop_res is False:
        print("Arm failed to drop item to pinky")

    target_pos = util_mapper.select_position_by_id(position_id)
    if target_pos is None:
        print(f"{target_pos} not found")
    pinky_to_target_res = pinky_service.cmd_pinky(target_pos)
    if pinky_to_target_res is False:
        print("Pinky failed to reach target position")

    return {"success": True}


def fetch_confirm():
    charger_pos = util_mapper.select_charger_pos()
    pinky_to_charger = pinky_service.cmd_pinky(charger_pos)
    if pinky_to_charger is False:
        return {"success": False}
    return {"success": True}


def take_info():
    positions = gui_mapper.take_info()
    if positions is None:
        return []
    return {
        "position": [
            {
                "position_id": p.position_id,
                "position_name": p.position_name,
                "x": p.x,
                "y": p.y,
                "w": p.w,
            }
            for p in positions
        ]
    }


# TODO: implement take_cmd logic
def take_cmd(data):
    print(f"gui_service.take_cmd data: {data}")
    return "Hello, from take_cmd()"


def schedule_info():
    schedules = schedule_mapper.select_all_schedules()

    if not schedules:
        return []

    return {
        "schedules": [
            {
                "schedule_id": s.schedule_id,
                "cmd_id": s.cmd_id,
                "item_id": s.item_id,
                "position_id": s.position_id,
                "execute_time": s.execute_time.strftime("%H:%M:%S"),
                "cycle": s.cycle,
                "on_weekends": s.on_weekends,
            }
            for s in schedules
        ]
    }


def schedule_edit(data):
    is_success = False
    result = None

    action = data["action"]
    match action:
        case "add":
            result = schedule_mapper.insert_schedule(data)
        case "edit":
            result = schedule_mapper.update_schedule(data)
        case "delete":
            result = schedule_mapper.delete_schedule(data)
        case _:
            pass

    if result is not None:
        is_success = True

    return {
        "success": is_success,
    }


def history_info():
    histories: List[History] = gui_mapper.history_info()

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
