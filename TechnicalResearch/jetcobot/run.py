import os

import uvicorn
from dotenv import load_dotenv

load_dotenv()

JETCOBOT_HOST = os.getenv("JETCOBOT_HOST", "0.0.0.0")
JETCOBOT_PORT = int(os.getenv("JETCOBOT_PORT", "8080"))
JETCOBOT_DEBUG = bool(os.getenv("JETCOBOT_DEBUG", "True"))


if __name__ == "__main__":
    uvicorn.run(
        "app.main:app",
        host=JETCOBOT_HOST,
        port=JETCOBOT_PORT,
        reload=JETCOBOT_DEBUG,
    )
