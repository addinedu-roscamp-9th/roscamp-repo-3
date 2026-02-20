# AI Server

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

---

## Dot Env

```sh
GATEWAY_IP="192.168.0.22"
GATEWAY_PORT="8000"
GATEWAY_ENDPOINT="detection"

ARM_IP="192.168.5.1"
ARM_PORT="9001"
```
