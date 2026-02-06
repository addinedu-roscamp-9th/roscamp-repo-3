"""
run.py

FastAPI 서버 실행 전용 파일
- 외부(ROS2 로봇)에서 접근 가능하도록 서버를 실행한다
- app/main.py에 정의된 FastAPI 앱을 불러와 실행한다
"""

import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        # FastAPI 앱 위치
        # app/main.py 안의 app 객체
        "app.main:app",

        # 외부 접속 허용 (라즈베리파이, 젯코봇 등)
        host="0.0.0.0",

        # 서버 포트
        port=8000,

        # ROS 통신 안정성을 위해 reload 비활성화
        reload=False
    )

