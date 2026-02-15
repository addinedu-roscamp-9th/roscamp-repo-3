import os
import threading
from typing import List, Tuple

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


def create_posture(angles, gap=50):
    return {
        "j1": angles[0],
        "j2": angles[1],
        "j3": angles[2],
        "j4": angles[3],
        "j5": angles[4],
        "j6": angles[5],
        "gap": gap,
    }


def cmd_arm(postures) -> bool:
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
    position = gui_mapper.select_position_by_id(pos_id)
    if position is None:
        return None
    return position


def execute_parallel(pinky_position, arm_sequence) -> Tuple[bool, bool]:
    pinky_result = [False]
    arm_result = [False]

    def run_pinky():
        if pinky_service.cmd_pinky(pinky_position):
            pinky_result[0] = pinky_service.wait_pinky(timeout=60)
        else:
            pinky_result[0] = False

    def run_arm():
        arm_result[0] = cmd_arm(arm_sequence)

    pinky_thread = threading.Thread(target=run_pinky, name="PorterThread")
    arm_thread = threading.Thread(target=run_arm, name="ArmThread")

    pinky_thread.start()
    arm_thread.start()

    pinky_thread.join()
    arm_thread.join()

    return pinky_result[0], arm_result[0]


def fetch_cmd(data):
    item_id = data.get("item_id")
    position_id = data.get("position_id")
    dz_pos_id = "p2602100001"

    target_pos = get_pos_grid(position_id)
    dz_pos = get_pos_grid(dz_pos_id)

    if position_id is None:
        print("Target position not found")
        return {"success": False, "error": "Target position not found"}

    pick_sequence = [
        create_posture(SHELVE_SIDE, gap=100),
        create_posture(PICK1, gap=0),
        create_posture(SHELVE_SIDE, gap=0),
        create_posture(PINKY_SIDE, gap=0),
    ]

    pinky_success, arm_success = execute_parallel(dz_pos, pick_sequence)

    if not pinky_success:
        print("Porter: failed to reach DZ")
        return {"success": False, "error": "Porter: failed to reach DZ"}
    if not arm_success:
        print("Arm: failed to pick up the item")
        return {"success": False, "error": "Arm: failed to pick up the item"}

    print("Commaned porter and arm")

    drop_sequence = [
        create_posture(DROP, gap=100),
        create_posture(PINKY_SIDE, gap=100),
        create_posture(HOME, gap=100),
    ]

    is_success = cmd_arm(drop_sequence)

    if is_success is False:
        print("Arm: failed to drop")
        return {"success": False, "error": "Arm: failed to drop"}

    print("Arm: drop successful")

    # PHASE 3: Pinky goes to final destination
    print(f"=== PHASE 3: Pinky to Final Position ({position_id}) ===")

    result = pinky_service.cmd_pinky(target_pos)
    if result is False:
        print("final position not found")
        return {"success": False, "error": "final position not found"}

    if not pinky_service.wait_pinky(timeout=60):
        print("pinky failed to reach target")
        return {"success": False, "error": "pinky failed to reach target"}

    print("pinky reach target")

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
