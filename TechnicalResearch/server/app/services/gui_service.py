import httpx

from app.database import gui_mapper


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

    print(f"item: {item}")
    print(f"position: {position}")

    # Send item to jetcobot
    jetcobot_url = "http://192.168.0.22:8080/pose"

    try:
        with httpx.Client(timeout=60.0) as client:
            response = client.post(jetcobot_url, json={"item": item})
            response.raise_for_status()
            jetcobot_result = response.json()
            print(f"Jetcobot response: {jetcobot_result}")
    except httpx.HTTPError as e:
        print(f"Error sending to jetcobot: {e}")
        return {"status": "error", "message": f"Failed to send to jetcobot: {str(e)}"}

    # TODO: send position to pinky

    return {
        "status": "success",
        "message": "Item sent to jetcobot",
        "jetcobot_response": jetcobot_result,
    }
