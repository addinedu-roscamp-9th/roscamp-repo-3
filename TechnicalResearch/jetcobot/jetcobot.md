# Jetcobot

- [Dot Env](#dot-env)
- [Environment](#environment)
- [WebSocket Usage](#websocket-usage)

---

## Dot Env

Create `.env` file at same path as `main.py`

```sh
touch .env
```

Add the following inside the `.env`:

```sh
GATEWAY_HOST=192.168.0.56
GATEWAY_PORT=8000
ENDPOINT=jetcobot
```

Run `ip a` command from terminal to where the server is running

```sh
ip a
```

> Change the `GATEWAY_HOST` value as you needed

---

## Environment

Source the `venv_setup.sh` script to effect the current shell:

```sh
source ./venv_setup.sh
```

> Creates `.venv` and install dependencies

---

## WebSocket Usage

A WebSocket client for connecting Jetcobot robots to the control server.

### Setup

Install dependencies:

```bash
pip install websockets
```

### Basic Connection

```python
import asyncio
import json
from websockets import connect

async def connect_to_jetcobot():
    robot_id = "jetcobot_001"
    uri = f"ws://localhost:8000/jetcobot/ws/{robot_id}"

    async with connect(uri) as websocket:
        print(f"Connected as {robot_id}")

        # Your code here
```

### Sending Messages

```python
# declare msg to send
ping_msg = {
    "type": "ping",
    "timestamp": "2024-02-06T10:00:00Z"
}

# msg into json and send
# await: wait until sent
await websocket.send(json.dumps(ping_msg))
```

### Receiving Messages

```python
# Receive a single message
response = await websocket.recv()
response_data = json.loads(response)
print(f"Received: {response_data}")

# Listen with timeout
try:
    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
    response_data = json.loads(response)
    print(f"Received: {response_data}")
except asyncio.TimeoutError:
    print("No messages received (timeout)")
```
