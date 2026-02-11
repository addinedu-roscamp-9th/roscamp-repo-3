import os

import httpx
from dotenv import load_dotenv

from app.database import gui_mapper, schedule_mapper
from app.services import pinky_service

load_dotenv()

JETCOBOT_HOST = os.getenv("JETCOBOT_HOST", "192.168.0.56")
JETCOBOT_PORT = int(os.getenv("JETCOBOT_PORT", "8001"))


def login(data):
    user_id = data["id"]
    user_pw = data["pw"]

    user_name = gui_mapper.login_user(user_id, user_pw)

    if user_name is not None:
        return user_name
    return False


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


def send_postures_to_jetcobot(postures):
    """Send list of postures to jetcobot arm service"""
    jetcobot_url = f"http://{JETCOBOT_HOST}:{JETCOBOT_PORT}/pose"

    try:
        with httpx.Client(timeout=60.0) as client:
            response = client.post(jetcobot_url, json=postures)
            response.raise_for_status()
            result = response.json()
            print(f"Jetcobot response: {result}")
            return result
    except httpx.HTTPError as e:
        print(f"Error sending to jetcobot: {e}")
        return {"success": False, "error": str(e)}


def fetch_cmd(data):
    item = data["item"]
    position = data["position"]

    print("gui_service.py")
    print(f"item: {item}")
    print(f"position: {position}")

    # Create pick sequence for fetching item
    pick_sequence = [
        create_posture(DEFAULT_POS, gap=100),  # Start at default, gripper open
        create_posture(READY_POS, gap=100),    # Move to ready position
        create_posture(PICK_POS, gap=100),     # Move to pick position, gripper open
        create_posture(PICK_POS, gap=50),      # Close gripper to grab item
        create_posture(READY_POS, gap=50),     # Lift item to ready position
    ]

    # Send postures to jetcobot
    jetcobot_result = send_postures_to_jetcobot(pick_sequence)

    if not jetcobot_result.get("success"):
        error_msg = jetcobot_result.get("error", "Unknown error")
        return {"status": "error", "message": f"Failed to execute pick sequence: {error_msg}"}

    # TODO: first send pinky to DZ

    # TODO: send position to pinky
    pinky_res = pinky_service.send_position(position)
    print(pinky_res)

    return {
        "status": "success",
        "message": "Pick sequence completed and position sent to pinky",
        "jetcobot_response": jetcobot_result,
    }


# position_id = Column(String(11), primary_key=True)
# position_name = Column(String(30), nullable=False)
# x = Column(Float, nullable=False)
# y = Column(Float, nullable=False)
# theta = Column(Float, nullable=False)


def take_info():
    positions = gui_mapper.take_info()
    if positions is None:
        return []
    return [
        {
            "position_id": p.position_id,
            "position_name": p.position_name,
            "x": p.x,
            "y": p.y,
            "theta": p.theta,
        }
        for p in positions
    ]


# TODO: implement take_cmd logic
def take_cmd():
    return "Hello, from take_cmd()"


def schedule_info():
    schedules = schedule_mapper.select_all_schedules()

    if not schedules:
        return []

    return [
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


# {
#     msg_type: schedule_edit,
#     data: {
#         action: edit,
#         schedule_id: s2602100001,
#         cmd_id: c2602100001,
#         item_id: i2602100001,
#         position_id: p2602100001,
#         ececute_time: 15:40:00,
#         cycle: 1,
#         on_weekends: false,
#     }
# }


def schedule_edit(data):
    action = data["action"]
    success = "false"
    match action:
        case "add":
            result = schedule_mapper.insert_schedule(data)
            if result is not None:
                success = "true"
            return {
                "msg_type": "add",
                "success": success,
            }

        case "edit":
            result = schedule_mapper.update_schedule(data)
            if result is not None:
                success = "true"
            return {
                "msg_type": "edit",
                "success": success,
            }

        case "delete":
            result = schedule_mapper.delete_schedule(data)
            if result is not None:
                success = "true"
            return {
                "msg_type": "delete",
                "success": success,
            }

        case _:
            return None
