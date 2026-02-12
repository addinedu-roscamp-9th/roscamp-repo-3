import asyncio
import json
import sys

import websockets


async def test_fetch_cmd(item_id, position_id):
    uri = "ws://localhost:8000/gui"

    try:
        async with websockets.connect(uri) as websocket:
            message = {
                "msg": "fetch_cmd",
                "data": {
                    "item_id": item_id,
                    "position_id": position_id,
                },
            }

            await websocket.send(json.dumps(message))
            print(f"Sent: {json.dumps(message, indent=2)}")

            # Receive response
            response = await websocket.recv()
            response_data = json.loads(response)
            print(f"\nReceived: {json.dumps(response_data, indent=2)}")

            # Check if there was an error
            if response_data.get("msg") == "error":
                print("\n❌ Error:", response_data.get("data"))
                return False

            print("\n✓ Fetch command sent successfully!")
            return True

    except websockets.exceptions.WebSocketException as e:
        print(f"\n❌ WebSocket error: {e}")
        print("Make sure the server is running on localhost:8000")
        return False
    except json.JSONDecodeError as e:
        print(f"\n❌ JSON decode error: {e}")
        return False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False


if __name__ == "__main__":
    success = asyncio.run(test_fetch_cmd("2602070001", "p2602070001"))
    sys.exit(0 if success else 1)
