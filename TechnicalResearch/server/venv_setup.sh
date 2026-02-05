#!/usr/bin/env bash

# stop if compiling fail
# -e: errexti - exit immediately on error
# -u: nounset - error on unset variable
# -o pipefaul: fail pipelines correctly
set -euo pipefail

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

# installed required dependencies with pip
pip install "${pip_packages[@]}"

# installed required dependencies with apt
sudo apt install -y "${apt_packages[@]}"
