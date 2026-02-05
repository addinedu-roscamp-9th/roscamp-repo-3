#!/usr/bin/env bash

# stop if compiling fail
# -e: errexti - exit immediately on error
# -u: nounset - error on unset variable
# -o pipefaul: fail pipelines correctly
set -euo pipefail

pip_packages=(
  "ultralytics==8.4.11"
)

# create venv if not already created
if [ ! -d ./.venv ]; then
  python3 -m venv .venv
  echo 'Python virtual environment ".venv" created.'
fi

# activate venv
. .venv/bin/activate

# update pip
pip install -U pip

# install required dependencies with pip
pip install "${pip_packages[@]}"
