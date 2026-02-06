#!/usr/bin/env bash

pip_packages=(
  "uvicorn==0.40.0"
  "fastapi==0.128.0"
  "pyyaml==6.0.3"
  "numpy==2.4.2"
  "sqlalchemy==2.0.46"
  "jinja2==3.1.6"
  "setuptools==80.10.2"
  "typeguard==4.4.4"
  "websockets"
  "multipart"
  "opencv-python"
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
