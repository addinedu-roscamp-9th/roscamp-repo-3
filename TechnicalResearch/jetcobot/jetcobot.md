# Jetcobot

- [Dot Env](#dot-env)
- [Environment](#environment)

---

## Dot Env

Create `.env` file at same path as `main.py`:

```sh
touch .env
```

Open it with your editor of choice:

```sh
code .env
```

Add the following inside the `.env`:

```sh
GATEWAY_HOST="192.168.0.56"
GATEWAY_PORT=8000
ENDPOINT="jetcobot"

AI_HOST="192.168.0.56"
AI_PORT=9000
```

Find where the `GATEWAY` or `AI` server is running at:

```sh
ip a
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
