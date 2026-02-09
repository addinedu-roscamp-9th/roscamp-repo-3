from app.services import gui_service

# msg example from gui
#
# {
#     "msg_type": "login",
#     "data": {
#         "id": "admin",
#         "pw": "1234"
#     }
# }


def gui_controller(msg):
    msg_type = msg["msg_type"]
    data = msg["data"]

    match msg_type:
        case "connect":
            return True

        case "login":
            user_name = gui_service.login(data)
            print(user_name)
            return user_name
        case _:
            return {"status": "error", "error": f"Unknown msg_type: {msg_type}"}
