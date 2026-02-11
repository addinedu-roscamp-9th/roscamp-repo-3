# AGENTS.md — ROS2 + AI Autonomous Robot (Team 3)

## Repository Layout

Multi-module Python monorepo. Each module owns its virtualenv, entry point, and `.pylintrc`.

```
roscamp-repo-3/
├── arm/                   – MyCobot280 arm service (FastAPI, registers with gateway)
└── TechnicalResearch/
    ├── ai/                – YOLO inference server (async UDP, ultralytics)
    ├── camera/            – OpenCV calibration / undistortion scripts
    ├── gui/               – PyQt5 desktop GUI (HTTP client → server)
    ├── jetcobot/          – MyCobot280 arm driver (HTTP/REST service)
    ├── pinky/             – ROS2 mobile robot control scripts
    └── server/            – FastAPI central hub (routers → services → models)
```

**Note**: The `arm/` module is a standalone FastAPI service at the repository root. It receives posture sequences via `/pose` endpoint and executes them on MyCobot280 hardware. See `arm/AGENTS.md` for module-specific guidelines.

---

## Build / Run / Lint

### Environment Setup

Each module with dependencies has `venv_setup.sh`:

```bash
cd TechnicalResearch/<module>
bash venv_setup.sh          # creates .venv, installs deps
source .venv/bin/activate
python main.py
```

### Common Commands

| Task                 | Command                                                               |
| -------------------- | --------------------------------------------------------------------- |
| Run AI server        | `cd ai && source .venv/bin/activate && python main.py`               |
| Run FastAPI server   | `cd server && source .venv/bin/activate && python main.py`           |
| Run GUI              | `cd gui && source .venv/bin/activate && python main.py`              |
| Run Jetcobot service | `cd jetcobot && source .venv/bin/activate && python main.py`         |
| Run Arm service      | `cd ../arm && source .venv/bin/activate && python main.py`           |
| Lint module          | `cd <module> && source .venv/bin/activate && python -m pylint app/`  |
| Lint single file     | `cd <module> && source .venv/bin/activate && python -m pylint <file>` |
| Type check (arm)     | `cd ../arm && source .venv/bin/activate && pyright`                  |

### Tests

**No test suite exists.** If adding tests:
- Place in `tests/` inside the module
- Use `pytest`: `python -m pytest tests/`
- Run single test: `python -m pytest tests/test_foo.py::test_bar -v`
- Add `pytest` to `venv_setup.sh`

### Pylint

Shared `.pylintrc` in each module:
```ini
[MESSAGES CONTROL]
disable = missing-docstring, too-few-public-methods, wrong-import-order
```

The `arm/` module additionally disables `broad-except` to allow catching generic `Exception`.

Use inline `# pylint: disable=<code>` for one-off suppressions.

---

## Code Style Guidelines

### General

- **Language**: Python 3.12. Use modern syntax (`str | None`, `list[str]`, etc.).
- **Line length**: PEP 8 recommends 79 chars. The `arm/` module uses 88 (Black default). Both are acceptable.
- **Encoding**: UTF-8. Korean inline comments are common and acceptable.

### Imports

- Follow **PEP 8 grouping**: stdlib → third-party → local, separated by blank lines.
- Use **relative imports** within packages (e.g. `from .protocol import InferenceProtocol`).
  The `arm/` module uses **absolute imports** (e.g. `from app.model.posture import Posture`).
- Use `from __future__ import annotations` only when forward references are needed.
- Do not wildcard-import (`from module import *`).

### Naming Conventions

| Kind                  | Convention       | Examples                             |
| --------------------- | ---------------- | ------------------------------------ |
| Classes               | PascalCase       | `YOLODetector`, `MainWindow`, `Move` |
| Functions / methods   | snake_case       | `decode_image`, `pick_and_place`     |
| Constants             | UPPER_SNAKE_CASE | `UDP_HOST`, `GRIPPER_OPEN`           |
| Private members       | leading `_`      | `_init_ui`, `_stack`, `_config`      |
| Pydantic / ORM models | PascalCase       | `GuiData`, `RobotsData`, `Postures`  |

### Types & Data Models

- Use **Pydantic `BaseModel`** for all API request/response contracts (already the project
  standard across `server/`, `jetcobot/`, etc.).
- Do **not** mix `@dataclass` with `BaseModel` — pick one.
- Annotate function signatures, especially return types (`-> None`, `-> str`).
- Prefer `str | None` over `Optional[str]` (Python 3.10+ union syntax).

### Error Handling

- **HTTP / UI layer (gui screens)**: wrap client calls in `try/except Exception` and
  surface errors via `QMessageBox.critical`. Do not catch in the transport client itself.
- **Protocol / server handlers**: catch exceptions, log or return a structured error
  response (e.g. `{"status": "error", "message": ...}`). Do not silently swallow.
- **Avoid bare `except:`** — always specify the exception type at minimum (`except Exception`).
- **Background pollers** (e.g. battery status): `except Exception: pass` is acceptable
  only for non-fatal, high-frequency polls where failure is transient. Add a comment
  explaining why.
- Do not re-raise inside a handler unless you also log the error first.

### Configuration

- **server / ai**: module-level constants or environment variables.
- **gui**: JSON config file (`config.json`) loaded via `config.py`.
- **jetcobot / arm**: `python-dotenv` loading a `.env` file.
- Do **not** commit `.env` files. They must remain in `.gitignore`.

### Docstrings

- Write docstrings on all **public** classes and non-trivial functions.
  (`missing-docstring` is suppressed in pylint, but that is a safety net, not a licence
  to skip them.)
- Use concise, single-line docstrings when the function is self-evident;
  multi-line Google-style when parameters or return values need explanation.

---

## Architectural Conventions

- **Hub-and-spoke networking**: the `server` (FastAPI) is the central hub; `gui` and
  `jetcobot` are HTTP clients that POST commands and GET status through it.
- **Arm module**: standalone FastAPI service that registers with a gateway server on startup.
  Receives posture sequences at `/pose` endpoint and executes them directly on hardware.
- **AI module is independent**: communicates via async UDP (base64-encoded frames),
  not through the FastAPI server.
- **Layered structure** inside `server`, `jetcobot`, and `arm`:
  `routers/` (HTTP endpoints) → `services/` (business logic) → `models/` (data contracts).
- **GUI screens** map 1-to-1 to top-level views (`home.py`, `navigation.py`, `schedule.py`);
  shared HTTP calls live in `client.py`.

---

## Commit Style

Follow conventional-commit format with a module scope:

```
verb(scope): short description

# Examples:
add(server): register new /pinky/schedule endpoint
fix(gui): handle connection timeout in navigation screen
refactor(jetcobot): extract gripper logic into helper
add(arm): implement posture execution in controller
update(arm): simplify logging in pose endpoint
chore(ai): pin ultralytics to 8.4.11
```

Verbs: `add`, `fix`, `update`, `remove`, `refactor`, `change`, `chore`, `test`.

---

## Known Pitfalls — Do Not Repeat

1. **Committed secrets**: `jetcobot/.env` is committed to git. Never re-commit secrets or
   environment files. They should remain in `.gitignore`.
2. **Mixed decorators**: `pinky_model.py` incorrectly stacks `@dataclass` on a `BaseModel`.
   Remove the `@dataclass` decorator — Pydantic models should not be dataclasses.
3. **Untyped dicts**: `pinky.py` router uses raw `dict` type hints for request bodies.
   Replace with proper Pydantic `BaseModel` classes for validation.
4. **Version pinning**: Dependencies in `venv_setup.sh` are **not** version-pinned. When
   adding new packages, consider pinning exact versions to avoid breakage (e.g. use
   `"fastapi==0.128.0"` instead of `"fastapi"`). Current setup uses unpinned packages.
5. **Excessive logging**: Keep logging minimal. Only log errors or critical state changes.
   Verbose execution logs create noise and are hard to debug. Use format: `print(f"Error: {context} - {error}")`.