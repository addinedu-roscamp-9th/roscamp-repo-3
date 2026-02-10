from fastapi import APIRouter

from ..models.pinky_model import RobotCommand, RobotStatus

router = APIRouter()

# 로봇별 명령 저장소
robot_commands = {}


@router.post("/robot/cmd/{robot_id}")
def set_robot_cmd(robot_id: str, cmd: RobotCommand):
    """
    로봇에게 실행할 명령을 등록
    예:
    {
        "type": "move",
        "distance_cm": 10,
        "speed": 0.2
    }
    """
    robot_commands[robot_id] = cmd
    return {"result": "ok", "robot_id": robot_id, "cmd": cmd}


@router.get("/robot/cmd/{robot_id}")
def get_robot_cmd(robot_id: str):
    """
    핑키가 명령을 가져가면 1회성으로 소비
    """
    cmd = robot_commands.get(robot_id)

    if robot_id in robot_commands:
        robot_commands.pop(robot_id)

    # 명령이 없으면 idle 상태 전달
    return cmd or {"type": "idle"}


@router.post("/robot/status")
def update_robot_status(data: RobotStatus):
    """
    핑키 상태 수신용
    (위치, 동작 상태 등)
    """
    return {"result": "ok"}


@router.get("/ping")
def ping():
    return {"msg": "pinky pong"}
