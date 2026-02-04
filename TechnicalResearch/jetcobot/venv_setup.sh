#!/usr/bin/env bash

# stop if compiling fail
# -e: errexti - exit immediately on error
# -u: nounset - error on unset variable
# -o pipefaul: fail pipelines correctly
set -euo pipefail

pip_packages=(
  "pymycobot==4.0.4"
  "numpy==2.4.2"
  "pydantic==2.12.5"
  "requests==2.32.5"
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
