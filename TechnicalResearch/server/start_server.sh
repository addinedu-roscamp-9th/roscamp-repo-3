#!/usr/bin/env bash

# Script to start the FastAPI server with ROS2 environment

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source the porter workspace (for debugcrew_msgs)
source /home/lemon/dev/roscamp-repo-3/porter/install/setup.bash

# Activate Python virtual environment
source /home/lemon/dev/roscamp-repo-3/TechnicalResearch/server/.venv/bin/activate

# Start the FastAPI server
cd /home/lemon/dev/roscamp-repo-3/TechnicalResearch/server
python run.py
