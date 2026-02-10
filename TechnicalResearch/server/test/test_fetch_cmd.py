"""Test script to send fetch_cmd request to /gui WebSocket endpoint."""

import asyncio
import json

import websockets


async def test_fetch_cmd():
    """Connect to GUI WebSocket and send fetch_cmd."""
    uri = "ws://localhost:8000/gui"

    async with websockets.connect(uri) as websocket:
        # Send fetch_cmd request
        message = {
            "msg_type": "fetch_cmd",
            "data": {"item": "choco", "position": {"x": 1.5, "y": 2.0, "theta": 0.0}},
        }

        await websocket.send(json.dumps(message))
        print(f"Sent: {json.dumps(message, indent=2)}")

        # Receive response
        response = await websocket.recv()
        print(f"\nReceived: {json.dumps(json.loads(response), indent=2)}")


if __name__ == "__main__":
    asyncio.run(test_fetch_cmd())
