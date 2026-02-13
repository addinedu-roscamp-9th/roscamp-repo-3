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


HOME = [1.58, 49.92, -147.39, 91.14, 4.65, -44.64]
SHELVE_SIDE = [92.02, 6.85, -18.28, -42.09, 9.84, -45.52]

PICK1 = [111.79, -60.11, -18.72, -1.93, 3.42, -25.48]
PICK2 = [93.95, -62.13, -20.03, -1.93, 4.57, -41.39]
PICK3 = [76.99, -67.32, -20.47, 7.47, 3.33, -61.08]

PINKY_SIDE = [0.7, 7.11, -20.47, -46.14, 6.24, -48.42]
DROP = [3.6, -66.97, -21.35, -3.25, 2.63, -34.54]

TRASH_SIDE = [-89.2, -7.47, -21.7, -20.3, 3.86, -39.11]
TRASH1 = [-58.71, -67.58, -22.14, 15.82, 1.58, -20.39]
TRASH2 = [-88.24, -66.53, -22.5, 6.76, 6.94, -41.48]
TRASH3 = [-111.0, -71.19, -22.85, 11.07, 8.96, -61.17]


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


def cmd_arm(postures) -> bool:
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
    position = gui_mapper.select_position_by_id(pos_id)
    if position is None:
        return None
    return position


def fetch_cmd(data):
    item_id = data.get("item_id")
    position_id = data.get("position_id")

    # Step 1: Get DZ (drop zone) position from database
    dz_position = get_pos_grid(position_id)
    if dz_position is None:
        print("Position not found")
        return {"success": False, "error": "Position not found"}

    # Step 2: Send Pinky to DZ (non-blocking - just send the command)
    print("Step 1: Sending Pinky to DZ position")
    result = pinky_service.cmd_pinky(dz_position)
    if result is False:
        print("Failed to send Pinky to DZ")
        return {"success": False, "error": "Failed to send Pinky to DZ"}

    # Step 3: Jetcobot picks up item (while Pinky travels to DZ)
    print("Step 2: Jetcobot picking up item (while Pinky travels to DZ)")
    pick_sequence = [
        create_posture(SHELVE_SIDE, gap=100),
        create_posture(PICK1, gap=0),
        create_posture(SHELVE_SIDE, gap=0),
        create_posture(PINKY_SIDE, gap=0),
    ]

    is_success: bool = cmd_arm(pick_sequence)

    if is_success is False:
        print("Failed to pick up the item")
        return {"success": False, "error": "Jetcobot pick failed"}

    # Step 4: Wait for Pinky to arrive at DZ (blocking)
    print("Step 3: Waiting for Pinky to arrive at DZ...")
    if not pinky_service.wait_pinky(timeout=60):
        print("Pinky failed to reach DZ")
        return {"success": False, "error": "Pinky failed to reach DZ"}

    # Step 5: Jetcobot drops item onto Pinky at DZ
    print("Step 4: Jetcobot dropping item onto Pinky")
    drop_sequence = [
        create_posture(DROP, gap=100),
        create_posture(PINKY_SIDE, gap=100),
        create_posture(HOME, gap=100),
    ]

    is_success = cmd_arm(drop_sequence)

    if is_success is False:
        print("Failed to drop the item")
        return {"success": False, "error": "Jetcobot drop failed"}

    # Step 6: Get final target position
    position = get_pos_grid(position_id)
    if position is None:
        print(f"Target position '{position_id}' not found in database")
        return {"success": False, "error": "Position not found"}

    # Step 7: Pinky goes to final target position
    print(f"Step 5: Sending Pinky to final position {position_id}")
    result = pinky_service.cmd_pinky(position)
    if result is False:
        print("Failed to send Pinky to target position")
        return {"success": False, "error": "Failed to send command to Pinky"}

    print("Step 6: Waiting for Pinky to reach final destination...")
    if not pinky_service.wait_pinky(timeout=60):
        print("Pinky did not reach final destination")
        return {"success": False, "error": "Pinky navigation failed or timeout"}

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
