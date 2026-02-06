#!/usr/bin/env bash

pip_packages=(
  "pymycobot==4.0.4"
  "numpy==2.4.2"
  "pydantic==2.12.5"
  "requests==2.32.5"
  "dotenv==0.9.9"
  "websockets"
)

# create venv if not already created
if [ ! -d ./.venv ]; then
  python3 -m venv .venv
fi

# activate venv
. .venv/bin/activate

# update pip
pip install -U pip

# install required dependencies with pip
pip install "${pip_packages[@]}"
