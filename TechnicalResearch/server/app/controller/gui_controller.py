def gui_controller(msg):
    msg_type = msg["msg_type"]
    data = msg["data"]

    match msg_type:
        case "connect":
            return True

        case _:
            return {"status": "error", "error": f"Unknown msg_type: {msg_type}"}
