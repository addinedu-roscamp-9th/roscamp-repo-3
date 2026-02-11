import os

import httpx
from app.database import gui_mapper, schedule_mapper
from app.services import pinky_service
from dotenv import load_dotenv

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


def fetch_cmd(data):
    item = data["item"]
    position = data["position"]

    print("gui_service.py")
    print(f"item: {item}")
    print(f"position: {position}")

    # Send item to jetcobot
    jetcobot_url = f"http://{JETCOBOT_HOST}:{JETCOBOT_PORT}/pose"

    try:
        msg_type = "fetch"
        with httpx.Client(timeout=60.0) as client:
            response = client.post(
                jetcobot_url, json={"msg_type": msg_type, "item": item}
            )
            response.raise_for_status()
            jetcobot_result = response.json()
            print(f"Jetcobot response: {jetcobot_result}")
    except httpx.HTTPError as e:
        print(f"Error sending to jetcobot: {e}")
        return {"status": "error", "message": f"Failed to send to jetcobot: {str(e)}"}

    # TODO: first send pinky to DZ

    # TODO: send position to pinky
    pinky_res = pinky_service.send_position(position)
    print(pinky_res)

    return {
        "status": "success",
        "message": "Item sent to jetcobot",
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
