# AGENTS.md — ARM Module (MyCobot280)

## Overview

FastAPI service for controlling a MyCobot280 robotic arm. Receives posture sequences via HTTP POST to `/pose` endpoint and executes them on the physical hardware.

```
arm/
├── app/
│   ├── model/      – Pydantic data models (Posture)
│   ├── service/    – Business logic (Move, Connect)
│   └── controller/ – Execution controllers (optional layer)
├── main.py         – FastAPI entry point + HTTP server
├── .env            – Environment configuration (not committed)
├── .pylintrc       – Pylint configuration
└── pyrightconfig.json – Type checking configuration
```

---

## Build / Run / Lint

### Environment Setup

```bash
cd arm
bash venv_setup.sh          # creates .venv, installs dependencies
source .venv/bin/activate
python main.py              # runs FastAPI server on port 8001
```

### Common Commands

| Task             | Command                                                 |
| ---------------- | ------------------------------------------------------- |
| Run arm service  | `source .venv/bin/activate && python main.py`           |
| Lint all code    | `source .venv/bin/activate && python -m pylint app/`    |
| Lint single file | `source .venv/bin/activate && python -m pylint main.py` |
| Type check       | `source .venv/bin/activate && pyright`                  |

### Tests

**No test suite exists.** If adding tests:

- Place in `tests/` directory at project root
- Use `pytest`: `python -m pytest tests/`
- Run single test: `python -m pytest tests/test_move.py::test_send_angles -v`
- Add `pytest` to `venv_setup.sh` dependencies

---

## Code Style Guidelines

### General

- **Language**: Python 3.12. Use modern syntax (`str | None`, `list[Posture]`).
- **Line length**: 88 characters (Black default). PEP 8 recommends 79 but 88 is acceptable.
- **Encoding**: UTF-8.
- **Type checking**: Basic mode via Pyright. Annotate all function signatures.

### Imports

Follow **PEP 8 grouping**:

1. Standard library imports
2. Third-party imports
3. Local application imports

Separate each group with a blank line.

```python
# Good
import os
import threading
from typing import List

import uvicorn
from dotenv import load_dotenv
from fastapi import FastAPI

from app.controller.controller import Controller
from app.model.posture import Posture
from app.service.move import Move
```

**Rules:**

- Sort alphabetically within each group
- Use absolute imports for app modules (`from app.model.posture import Posture`)
- No wildcard imports (`from module import *`)
- Group `from X import Y` statements together

### Naming Conventions

| Kind                | Convention       | Examples                             |
| ------------------- | ---------------- | ------------------------------------ |
| Classes             | PascalCase       | `Posture`, `Move`, `Controller`      |
| Functions / methods | snake_case       | `send_angles`, `set_gripper`         |
| Constants           | UPPER_SNAKE_CASE | `GATEWAY_HOST`, `HTTP_PORT`, `SPEED` |
| Private members     | leading `_`      | `_mc`, `_internal_state`             |
| Pydantic models     | PascalCase       | `Posture` (fields are snake_case)    |

### Types & Data Models

- Use **Pydantic `BaseModel`** for all API contracts and data structures
- Always annotate function signatures with types:
  ```python
  def send_angles(self, data: Posture, speed: int = SPEED) -> None:
  ```
- Use modern union syntax: `str | None` instead of `Optional[str]`
- Use generic types: `List[Posture]` or `list[Posture]` (Python 3.9+)

**Pydantic models:**

```python
class Posture(BaseModel):
    pose_name: str
    j1: float
    j2: float
    # ... more fields
    gap: int  # milliseconds
```

### Error Handling

- **FastAPI endpoints**: Return structured error responses

  ```python
  try:
      controller.execute()
      return {"success": True}
  except Exception as e:
      return {"success": False, "error": str(e)}
  ```

- **Service layer**: Log errors but don't silently swallow

  ```python
  try:
      self.move_service.execute(posture)
  except Exception as e:
      print(f"Error: {posture.pose_name} - {e}")
  ```

- **Avoid bare `except:`** — always specify exception type
- Only use `except Exception: pass` for non-fatal operations with a comment explaining why

### Logging

- **Keep logging minimal and informative**
- Don't log every step of execution (only errors or important state changes)
- Use `print()` for simple logging (no logger setup currently)
- Format: `print(f"Error: {context} - {error_message}")`

### Configuration

- Use **`python-dotenv`** with `.env` file for configuration
- Load at application start: `load_dotenv()`
- Access via `os.getenv()` with defaults:
  ```python
  GATEWAY_HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")
  GATEWAY_PORT = int(os.getenv("GATEWAY_PORT", "8000"))
  ```
- **Never commit `.env` files** — keep in `.gitignore`

### Docstrings

- Write docstrings for all **public** classes and non-trivial functions
- Use single-line format for simple functions:
  ```python
  def execute(self):
      """Execute all postures in sequence"""
  ```
- Use multi-line Google-style for complex functions:
  ```python
  def send_angles(self, data: Posture, speed: int = SPEED) -> None:
      """Send joint angles to the robotic arm.

      Args:
          data: Posture containing joint angles (j1-j6)
          speed: Movement speed (0-100)
      """
  ```

---

## Architecture

### Layers

```
HTTP Layer (main.py)
    ↓ receives List[Posture]
Controller Layer (controller/)
    ↓ orchestrates execution
Service Layer (service/)
    ↓ hardware interaction
Hardware (MyCobot280 via pymycobot)
```

### Components

- **main.py**: FastAPI application, HTTP server, gateway connection
- **app/model/posture.py**: Pydantic data models
- **app/service/move.py**: Direct hardware control (MyCobot280)
- **app/service/connect_gateway.py**: Gateway server registration
- **app/controller/**: Execution orchestration (if present)

### Threading

- HTTP server runs in daemon thread
- Main thread handles gateway connection and keeps process alive
- Hardware operations are synchronous (blocking)

---

## Pylint Configuration

Current `.pylintrc` disables:

- `missing-docstring` — still write them for public APIs
- `too-few-public-methods` — acceptable for simple classes
- `wrong-import-order` — follow PEP 8 manually
- `broad-except` — catching `Exception` is allowed

Use inline suppression sparingly:

```python
# pylint: disable=broad-except
```

---

## Commit Style

Follow conventional commits with module scope:

```
verb(arm): short description

Examples:
add(arm): implement posture execution in controller
fix(arm): handle connection timeout to gateway
refactor(arm): extract gripper control to separate method
update(arm): simplify logging in pose endpoint
```

**Verbs**: `add`, `fix`, `update`, `remove`, `refactor`, `change`, `chore`, `test`

---

## Known Patterns & Conventions

1. **Global state**: `move` is initialized globally in `main.py` and shared with endpoints
2. **Async endpoints**: FastAPI endpoints use `async def` even though operations are sync
3. **Time delays**: Use `time.sleep()` for hardware settling times (already in Move service)
4. **Port configuration**: Hardware port is `/dev/ttyJETCOBOT`, baud `1000000`
5. **Gateway registration**: Service registers with central gateway on startup

## Best Practices

- Test hardware operations carefully — real robot can cause physical damage
- Always validate posture data before sending to hardware
- Keep execution logs minimal to avoid console spam
- Use constants for hardware parameters (PORT, BAUD, SPEED)
- Handle connection failures gracefully (don't crash on hardware init failure)
