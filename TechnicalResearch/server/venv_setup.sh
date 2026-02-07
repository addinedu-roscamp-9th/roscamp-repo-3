#!/usr/bin/env bash

pip_packages=(
  "cryptography"
  "fastapi"
  "jinja2"
  "multipart"
  "numpy"
  "opencv-python"
  "pymysql"
  "pyyaml"
  "setuptools"
  "sqlalchemy"
  "typeguard"
  "uvicorn"
  "websockets"
)

apt_packages=(
  "ros-jazzy-nav2-msgs"
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

# install required dependencies with apt
sudo apt install -y "${apt_packages[@]}"
