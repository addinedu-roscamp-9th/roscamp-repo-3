# Server

- [Project Structure](#project-structure)
- [Environments](#environments)
- [API Endpoints](#api-endpoints)

---

## Project Structure

```
project/
├── app/
│   ├── routers/     ← API endpoints (handles requests)
│   ├── services/    ← Business logic
│   └── models/      ← Data structures (Pydantic)
├── main.py          ← App entry point
├── run.py           ← Server runner
├── config.py        ← Settings
├── requirements.txt
└── .env             ← Environment variables (not in git)
```

---

## Dependencies

If reading from `requirements.txt` doesn't work install it manullay

### PIP

install with pip

```sh
pip install uvicorn fastapi pyyaml numpy sqlalchemy
```

### APT

- ros2-jazzy-nav2-msgs

```sh
sudo apt install ros2-jazzy-nav2-msgs
```

---

## Environments

Create new venv:

```sh
python3 -m venv .venv
```

Activate venv:

```sh
. .venv/bin/activate
```

Install environments:

```sh
pip install -r requirements.txt
```

Run the server:

```sh
python run.py
```

---

## API Endpoints

| Method | URL          | Description        |
| ------ | ------------ | ------------------ |
| POST   | `/jetcobot/` | Robot sends status |
