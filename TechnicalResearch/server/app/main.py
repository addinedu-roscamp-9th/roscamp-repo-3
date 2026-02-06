"""
app/main.py

FastAPI 애플리케이션 정의 파일
- 서버 실행 로직은 포함하지 않는다
- 라우터 등록 및 공통 환경 설정만 담당한다
"""

import os
from fastapi import FastAPI
from app.routers import gui, jetcobot, pinky

# =========================
# 환경 변수 기본값 설정
# =========================
# 이미 외부에서 환경 변수가 설정되어 있으면 그대로 사용
# 설정되지 않았을 경우에만 기본값 적용
os.environ.setdefault("ROBOT_SERVER_HOST", "192.168.0.52")
os.environ.setdefault("ROBOT_SERVER_PORT", "8000")

# =========================
# FastAPI 앱 생성
# =========================
app = FastAPI(
    title="Roboto Server",
    redirect_slashes=False
)

# =========================
# 라우터 등록
# =========================
# 각 로봇 및 GUI 기능을 분리된 라우터로 관리
app.include_router(jetcobot.router, prefix="/jetcobot", tags=["jetcobot"])
app.include_router(pinky.router, prefix="/pinky", tags=["pinky"])
app.include_router(gui.router, prefix="/gui", tags=["gui"])

