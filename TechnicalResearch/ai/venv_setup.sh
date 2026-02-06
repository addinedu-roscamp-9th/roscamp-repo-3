#!/usr/bin/env bash

pip_packages=(
  "ultralytics==8.4.11"
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
