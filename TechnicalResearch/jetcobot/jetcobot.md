# Jetcobot

- [Dot Env](#dot-env)
- [Dependency](#dependency)
- [Environment](#environment)

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

> Change the `GATEWAY_HOST` value as you needed

Or with echo command:

```sh
echo -e 'GATEWAY_HOST=192.168.0.56\nGATEWAY_PORT=8000\nENDPOINT=jetcobot' >> .env
```

---

## Dependency

Installed dependencies

- pymycobot
- numpy
- pydantic
- requests

---

## Environment

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
