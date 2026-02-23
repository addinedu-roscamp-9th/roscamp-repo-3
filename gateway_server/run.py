import uvicorn

HOST = "0.0.0.0"  # 외부에서 연결 가능
PORT = 8000
DEBUG = False  # ROS 통신 안정성을 위해 비활성화

if __name__ == "__main__":
    uvicorn.run(
        "app.main:app",  # app/main.py 호출
        host=HOST,
        port=PORT,
        reload=DEBUG,
    )
