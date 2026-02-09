# Server

- [Project Structure](#project-structure)
- [Environments](#environments)

---

## Project Structure

```
project/
├── app/
│   ├── routers/     ←  API endpoints (handles requests)
│   ├── services/    ←  Business logic
│   ├── models/      ←  Data structures (Pydantic)
│   └── main.py      ←  App entry point
└── run.py       ←  Run this to start the server
```

---

## Environment

Run the `venv_setup.sh` script

```sh
./venv_setup.sh
```

1. Creates `.venv` at current directory
2. Updates PIP
3. Install required packages

Check `.venv` created:

```sh
ls -a # or $ la
```

Start the venv:

```sh
source .venv/bin/activate
```
