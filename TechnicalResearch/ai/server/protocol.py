import asyncio
import json

from inference.utils import decode_image


class InferenceProtocol(asyncio.DatagramProtocol):
    def __init__(self, detector):
        self.detector = detector
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        asyncio.create_task(self.handle_request(data, addr))

    async def handle_request(self, data, addr):
        try:
            request = json.loads(data.decode())
            image = decode_image(request["image"])
            detections = self.detector.predict(image)
            response = {"status": "ok", "detections": detections}
        except Exception as e:
            response = {"status": "error", "message": str(e)}

        self.transport.sendto(json.dumps(response).encode(), addr)
