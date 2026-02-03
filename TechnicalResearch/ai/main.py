import asyncio
from pathlib import Path

from inference.yolo_model import YOLODetector
from server.udp_server import UDPServer

# figure out where our files live
BASE_DIR = Path(__file__).parent
MODEL_PATH = BASE_DIR / "model" / "best.pt"

# network config for the UDP server
UDP_HOST = "0.0.0.0"  # listen on all interfaces
UDP_PORT = 9000


async def main():
    """Fire up the detector and keep the server running until interrupted."""
    detector = YOLODetector(MODEL_PATH)  # load the YOLO model into memory
    server = UDPServer(detector, UDP_HOST, UDP_PORT)
    await server.start()  # bind to the port and start listening

    try:
        await asyncio.sleep(float("inf"))  # hang out forever until someone hits Ctrl+C
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.stop()  # clean up the socket


if __name__ == "__main__":
    asyncio.run(main())
