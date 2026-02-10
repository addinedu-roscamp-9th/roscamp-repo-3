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
GATEWAY_HOST=192.168.0.56
GATEWAY_PORT=8000
ENDPOINT=jetcobot
```

Run `ip a` command from terminal to where the gateway server is running

```sh
ip a
```

> Change the `GATEWAY_HOST` value as you needed

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
