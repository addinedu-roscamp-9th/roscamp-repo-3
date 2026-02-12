import os
from typing import List

import httpx
from dotenv import load_dotenv

from app.database import gui_mapper, schedule_mapper
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


DEFAULT_POS = [5.36, 121.99, -140.09, -24.69, 6.41, -126.82]
READY_POS = [50.2, 116.0, 311.6, -160.96, -5.67, 48.23]
PICK_POS = [33.2, 241.5, 114.7, -168.73, 7.76, 45.65]
PLACE_POS = [265.2, 45.4, 42.8, -174.55, 14.04, -35.09]


def create_posture(angles, gap=50):
    """Convert position array to Posture format for jetcobot/arm"""
    return {
        "j1": angles[0],
        "j2": angles[1],
        "j3": angles[2],
        "j4": angles[3],
        "j5": angles[4],
        "j6": angles[5],
        "gap": gap,
    }


def send_postures_to_jetcobot(postures) -> bool:
    """Send list of postures to jetcobot arm service"""
    jetcobot_url = f"http://{JETCOBOT_HOST}:{JETCOBOT_PORT}/pose"

    try:
        with httpx.Client(timeout=60.0) as client:
            response = client.post(jetcobot_url, json=postures)
            response.raise_for_status()
            result = response.json()
            is_success: bool = result["success"]
            print(f"Jetcobot response: {result}")
            return is_success
    except httpx.HTTPError as e:
        print(f"Error sending to jetcobot: {e}")
        return False


def get_pos_grid(pos_id):
    result = gui_mapper.select_position_by_id(pos_id)
    if result is None:
        return None
    x, y, w = result
    return x, y, w


def fetch_cmd(data):
    msg = "fetch_cmd"
    item = data["item"]
    position = data["position"]

    print("gui_service.py")
    print(f"item: {item}")
    print(f"position: {position}")

    # Create pick sequence for fetching item
    pick_sequence = [
        create_posture(DEFAULT_POS, gap=100),
        create_posture(READY_POS, gap=100),
        create_posture(PICK_POS, gap=100),
        create_posture(PICK_POS, gap=50),
        create_posture(READY_POS, gap=50),
    ]

    # Send postures to jetcobot
    is_success: bool = send_postures_to_jetcobot(pick_sequence)

    if is_success is False:
        print("Failed to pick up the item")
        return {"success": False}

    # TODO: first send pinky to DZ
    if is_success is False:
        print("Failed to drop the item")
        return {"success": False}

    # TODO: send position to pinky
    pinky_res = pinky_service.send_position(position)
    print(pinky_res)

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
