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
└── requirements.txt
```

---

## Environments

Source the `venv_setup.sh` script to effect the current shell:

```sh
source ./venv_setup.sh
```

1. Create venv named `.venv`
2. Start venv
3. Install PIP modules
4. Install APT package

> Just running the script will execute is in the subshell which will not effect current shell.
> Which means the venv will not be activated on current shell when ran with `./venv_setup.sh`.

---

## API Endpoints

| Method | URL          | Description        |
| ------ | ------------ | ------------------ |
| POST   | `/jetcobot/` | Robot sends status |
