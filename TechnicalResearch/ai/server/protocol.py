import asyncio
import json

from inference.utils import decode_image


class InferenceProtocol(asyncio.DatagramProtocol):
    """UDP protocol handler for running object detection on incoming images."""

    def __init__(self, detector):
        self.detector = detector  # the ML model that does the actual detection work
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport  # store the transport so we can send responses later

    def datagram_received(self, data, addr):
        asyncio.create_task(
            self.handle_request(data, addr)
        )  # handle each request without blocking

    async def handle_request(self, data, addr):
        """Process an incoming detection request and send back results."""
        try:
            request = json.loads(data.decode())  # parse the incoming JSON payload
            image = decode_image(
                request["image"]
            )  # convert base64/bytes back to an image
            detections = self.detector.predict(
                image
            )  # run the model and get bounding boxes
            response = {"status": "ok", "detections": detections}
        except Exception as e:
            response = {
                "status": "error",
                "message": str(e),
            }  # something went wrong, let the client know

        self.transport.sendto(
            json.dumps(response).encode(), addr
        )  # send the results back to the caller
