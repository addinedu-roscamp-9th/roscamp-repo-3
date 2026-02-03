import asyncio

from .protocol import InferenceProtocol


class UDPServer:
    def __init__(self, detector, host: str, port: int):
        self.detector = detector  # ML model for inference
        self.host = host  # IP address to bind to
        self.port = port  # Port to listen on
        self.transport = None  # Will hold the UDP transport

    async def start(self):
        # Get the current event loop
        loop = asyncio.get_event_loop()
        # Create UDP endpoint and bind to host:port
        self.transport, _ = await loop.create_datagram_endpoint(
            lambda: InferenceProtocol(self.detector), local_addr=(self.host, self.port)
        )
        print(f"UDP server listening on {self.host}:{self.port}")

    def stop(self):
        # Clean up transport if it exists
        if self.transport:
            self.transport.close()
            print("Server stopped")
