#!/usr/bin/env bash

# stop if compiling fail
# -e: errexit - exit immediately on error
# -u: nounset - error on unset variable
# -o pipefaul: fail pipelines correctly
set -euo pipefail

pip_packages=(
  "cryptography"
  "fastapi"
  "httpx"
  "jinja2"
  "multipart"
  "dotenv"
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
