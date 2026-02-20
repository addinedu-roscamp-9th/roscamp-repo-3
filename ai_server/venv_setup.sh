#!/usr/bin/env bash

# stop if compiling fail
# -e: errexit - exit immediately on error
# -u: nounset - error on unset variable
# -o pipefaul: fail pipelines correctly
set -euo pipefail

pip_packages=(
  "dotenv"
  "mediapipe"
  "opencv-python"
  "ultralytics"
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

if [ ! -f ./mediapipe/hand_landmarker.task ]; then
  mkdir -p mediapipe
  wget -O mediapipe/hand_landmarker.task \
    https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
fi
