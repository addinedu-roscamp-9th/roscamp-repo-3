import asyncio
from pathlib import Path

from inference.yolo_model import YOLODetector
from server.udp_server import UDPServer

BASE_DIR = Path(__file__).parent
MODEL_PATH = BASE_DIR / "model" / "best.pt"

UDP_HOST = "0.0.0.0"
UDP_PORT = 9000


async def main():
    detector = YOLODetector(MODEL_PATH)
    server = UDPServer(detector, UDP_HOST, UDP_PORT)

    await server.start()

    try:
        await asyncio.sleep(float("inf"))
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.stop()


if __name__ == "__main__":
    asyncio.run(main())
