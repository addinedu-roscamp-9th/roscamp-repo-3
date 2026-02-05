from fastapi import APIRouter

router = APIRouter()

robot_commands = {}

@router.post("/robot/cmd/{robot_id}")
def set_robot_cmd(robot_id: str, cmd: dict):
    """
    서버에서 로봇에게 명령 등록
    """
    robot_commands[robot_id] = cmd
    return {"result": "ok", "robot_id": robot_id, "cmd": cmd}


@router.get("/robot/cmd/{robot_id}")
def get_robot_cmd(robot_id: str):
    """
    핑키가 명령을 가져가면 '소비' 후 제거
    """
    cmd = robot_commands.get(robot_id, {"linear": 0.0, "angular": 0.0})
    if robot_id in robot_commands:
        robot_commands.pop(robot_id)
    return cmd

@router.post("/robot/status")
def update_robot_status(data: dict):
    return {"result": "ok", "data": data}

