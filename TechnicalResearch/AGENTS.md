# AGENTS.md — ROS2 + AI Autonomous Robot (Team 3)

## Repository Layout

The repo is a **multi-module Python monorepo**. Each module owns its own virtualenv,
entry point, and `.pylintrc`. There is no shared build system or top-level package manager.

```
roscamp-repo-3/
├── TechnicalResearch/
│   ├── ai/            – YOLO inference server (async UDP, ultralytics)
│   ├── camera/        – Standalone OpenCV calibration / undistortion scripts
│   ├── gui/           – PyQt5 desktop control panel (HTTP client → server)
│   ├── jetcobot/      – MyCobot280 arm driver (HTTP client → server)
│   └── server/        – FastAPI central hub (routers → services → models)
```

---

## Environment Setup (per-module)

Each module that needs dependencies ships a `venv_setup.sh`. Activate before running:

```bash
cd TechnicalResearch/<module>
bash venv_setup.sh          # creates .venv and pip-installs pinned deps
source .venv/bin/activate
python main.py
```

All packages are **exact-version pinned** (e.g. `fastapi==0.128.0`). Do not loosen pins
without a documented reason.

---

## Running / Linting (no formal build step)

| Action                        | Command                                          |
|-------------------------------|--------------------------------------------------|
| Start AI inference server     | `cd ai && source .venv/bin/activate && python main.py` |
| Start FastAPI server          | `cd server && source .venv/bin/activate && python main.py` |
| Start GUI                     | `cd gui && source .venv/bin/activate && python main.py` |
| Start Jetcobot client         | `cd jetcobot && source .venv/bin/activate && python main.py` |
| Lint a single module          | `cd <module> && source .venv/bin/activate && python -m pylint app/` |
| Lint a single file            | `cd <module> && source .venv/bin/activate && python -m pylint path/to/file.py` |

### Tests

There is **no test suite** in this repository at present. No `pytest`, `unittest`, or
`conftest.py` exist. If you add tests, follow these conventions:
- Place them in a `tests/` directory inside the relevant module.
- Use `pytest` as the runner: `source .venv/bin/activate && python -m pytest tests/`
- Run a single test: `python -m pytest tests/test_foo.py::test_bar -v`
- Add `pytest` to that module's `venv_setup.sh`.

### Pylint configuration

Every module shares the same `.pylintrc` suppressions:

```ini
[MESSAGES CONTROL]
disable =
    missing-docstring,
    too-few-public-methods,
    wrong-import-order,
```

Do **not** add new global suppressions. Silence individual violations with inline
`# pylint: disable=<code>` comments instead.

---

## Code Style Guidelines

### General
- **Language**: Python 3.12. Use modern syntax (`str | None`, `list[str]`, etc.).
- **Line length**: Follow PEP 8 (79 chars for code, 72 for docstrings/comments).
- **Encoding**: UTF-8. Korean inline comments are common and acceptable.

### Imports
- Follow **PEP 8 grouping**: stdlib → third-party → local, separated by blank lines.
- Use **relative imports** within packages (e.g. `from .protocol import InferenceProtocol`).
- Use `from __future__ import annotations` only when forward references are needed.
- Do not wildcard-import (`from module import *`).

### Naming Conventions
| Kind                   | Convention        | Examples                              |
|------------------------|-------------------|---------------------------------------|
| Classes                | PascalCase        | `YOLODetector`, `MainWindow`, `Move`  |
| Functions / methods    | snake_case        | `decode_image`, `pick_and_place`      |
| Constants              | UPPER_SNAKE_CASE  | `UDP_HOST`, `GRIPPER_OPEN`            |
| Private members        | leading `_`       | `_init_ui`, `_stack`, `_config`       |
| Pydantic / ORM models  | PascalCase        | `GuiData`, `RobotsData`, `Postures`   |

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
- **jetcobot**: `python-dotenv` loading a `.env` file.
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
- **AI module is independent**: communicates via async UDP (base64-encoded frames),
  not through the FastAPI server.
- **Layered structure** inside `server` and `jetcobot`:
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
chore(ai): pin ultralytics to 8.4.11
```

Verbs: `add`, `fix`, `update`, `remove`, `refactor`, `change`, `chore`, `test`.

---

## Known Pitfalls — Do Not Repeat

1. `jetcobot/.env` is committed to git — never re-commit secrets/env files.
2. `server/app/template.py` references an undefined `DATABASE_URL` — wire it before use.
3. `pinky_model.py` incorrectly stacks `@dataclass` on a `BaseModel` — remove the decorator.
4. `jetcobot/venv_setup.sh` lists `doeenv` — the correct package is `python-dotenv`.
5. Use Pydantic models in `pinky.py` router instead of raw `dict` type hints.
