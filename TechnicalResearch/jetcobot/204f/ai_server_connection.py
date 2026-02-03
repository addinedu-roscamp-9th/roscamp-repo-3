import base64
import json
import socket
from pathlib import Path

import cv2


def connect_to_ai_server():
    """Send a test image to the detection server and print what it finds."""
    img_path = Path(__file__).parent / "test_img.jpg"

    # read and shrink the image to keep the UDP packet manageable
    img = cv2.imread(str(img_path))
    img = cv2.resize(img, (640, 480))

    # compress as JPEG at 50% quality to reduce payload size
    _, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 50])
    img_b64 = base64.b64encode(encoded).decode()

    # create a UDP socket for the request
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5.0)  # don't wait forever if the server is down

    try:
        request = json.dumps({"image": img_b64})
        print(f"Payload size: {len(request)} bytes")  # useful for debugging UDP limits

        sock.sendto(request.encode(), ("localhost", 9000))  # fire off the request
        data, _ = sock.recvfrom(65535)  # wait for the server's response
        response = json.loads(data.decode())
        print("Response:", response)
        return response
    except socket.timeout:
        print("Timeout - no response from server")  # server might be offline or slow
    finally:
        sock.close()  # always clean up the socket


if __name__ == "__main__":
    connect_to_ai_server()
