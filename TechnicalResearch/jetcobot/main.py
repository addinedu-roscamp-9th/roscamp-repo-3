import os

import uvicorn
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Jetcobot server configuration
HOST = os.getenv("JETCOBOT_HOST", "0.0.0.0")
PORT = int(os.getenv("JETCOBOT_PORT", "8001"))
DEBUG = os.getenv("JETCOBOT_DEBUG", "False").lower() == "true"


def main():
    """Start the FastAPI server."""
    uvicorn.run(
        "app.server:app",
        host=HOST,
        port=PORT,
        reload=DEBUG,
    )


if __name__ == "__main__":
    main()
