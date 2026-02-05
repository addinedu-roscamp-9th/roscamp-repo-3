 GUI

- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Dependency](#dependency)

---

## Project Structure

```
gui/
├── main.py              ← Entry point (QApplication setup + launch)
├── config.json          ← Server URL
└── app/
    ├── config.py        ← Loads config.json, exposes SERVER_URL
    ├── styles.py        ← Shared stylesheets and dark-mode palette
    ├── client.py        ← All HTTP calls to the FastAPI server
    ├── window.py        ← Main window, screen stack, battery poller
    └── screens/
        ├── home.py      ← Home screen (Pinky / Schedule menu)
        ├── navigation.py← Robot status, room nav, bring/put/stop
        └── schedule.py  ← Task-preset manager (add / run / delete)
```

| Layer       | File(s)            | Role                                                                                           |
| ----------- | ------------------ | ---------------------------------------------------------------------------------------------- |
| Entry point | `main.py`          | Creates `QApplication`, applies dark palette, shows `MainWindow`                               |
| Window      | `app/window.py`    | Owns `QStackedWidget`, wires screen navigation, runs the battery-status poller every 2 s       |
| Screens     | `app/screens/*.py` | One `QWidget` subclass per screen; handle layout and user dialogs                              |
| Client      | `app/client.py`    | Every `requests` call in one place — move, stop, bring, put, schedule, status                  |
| Styles      | `app/styles.py`    | Colour map, parameterised stylesheet generators, static stylesheet constants, `dark_palette()` |
| Config      | `app/config.py`    | Reads `config.json` once at import time; exports `SERVER_URL`                                  |

---

## Configuration

`config.json` sits next to `main.py` and is loaded automatically on startup:

```json
{
  "server_url": "http://192.168.0.56:8000"
}
```

> Change `server_url` to match the host and port of the FastAPI server.

---

## Dependency

Installed dependencies

- PyQt5
- requests
