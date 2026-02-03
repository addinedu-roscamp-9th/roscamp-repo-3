import asyncio

from .protocol import InferenceProtocol


class UDPServer:
    def __init__(self, detector, host: str, port: int):
        self.detector = detector
        self.host = host
        self.port = port
        self.transport = None

    async def start(self):
        loop = asyncio.get_event_loop()
        self.transport, _ = await loop.create_datagram_endpoint(
            lambda: InferenceProtocol(self.detector), local_addr=(self.host, self.port)
        )
        print(f"UDP server listening on {self.host}:{self.port}")

    def stop(self):
        if self.transport:
            self.transport.close()
            print("Server stopped")
