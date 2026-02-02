import os
import time

from fastapi import APIRouter, FastAPI, File, UploadFile
from pydantic import BaseModel

from app.models.pinky_model import RobotStatus
from app.services import pinky_service

router = APIRouter()

gui_connections: list = []


# =========================
# 데이터 저장소 (메모리)
# =========================
robot_states = {}  # {robot_id: status_dict}
robot_commands = {}  # {robot_id: command_dict}

# =========================
# 이미지 저장 폴더
# =========================
IMAGE_FOLDER = "robot_images"
os.makedirs(IMAGE_FOLDER, exist_ok=True)


# http://192.168.5.10:8000/pinky


# =========================
# 1️⃣ 서버 상태 확인
# =========================
@router.get("/")
def root():
    return {"status": "server running"}


# =========================
# 2️⃣ 로봇 상태 수신 (POST)
# =========================
@router.post("/robot/status")
def robot_status(status: RobotStatus):
    robot_states[status.robot_id] = status.dict()
    return {"result": "ok"}


# =========================
# 3️⃣ 특정 로봇 상태 조회 (GET)
# =========================
@router.get("/robot/status/{robot_id}")
def get_robot_status(robot_id: str):
    return robot_states.get(robot_id, {})


# =========================
# 4️⃣ 전체 로봇 상태 조회 (GET)
# =========================
@router.get("/robots")
def get_all_robots():
    return robot_states


# =========================
# 5️⃣ 로봇 명령 가져오기 (GET)
# =========================
@router.get("/robot/cmd/{robot_id}")
def get_robot_cmd(robot_id: str):
    # 명령이 없으면 기본값
    return robot_commands.get(robot_id, {"cmd": "idle", "speed": 0.0})


# =========================
# 6️⃣ 로봇 명령 설정 (POST)
# =========================
@router.post("/robot/cmd/{robot_id}")
def set_robot_cmd(robot_id: str, cmd: dict):
    robot_commands[robot_id] = cmd
    return {"result": "command updated", "robot_id": robot_id}


# =========================
# 7️⃣ 이미지 업로드
# =========================
@router.post("/robot/upload")
async def upload_image(robot_id: str, file: UploadFile = File(...)):
    timestamp = int(time.time())
    filename = f"{robot_id}_{timestamp}_{file.filename}"
    filepath = os.path.join(IMAGE_FOLDER, filename)

    contents = await file.read()
    with open(filepath, "wb") as f:
        f.write(contents)

    print(f"[IMAGE] {robot_id} -> {filepath}")

    return {"result": "saved", "robot_id": robot_id, "filename": filename}
