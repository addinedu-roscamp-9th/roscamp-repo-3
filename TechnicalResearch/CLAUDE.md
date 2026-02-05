# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a multi-robot coordination system with YOLO-based object detection. It consists of three main services that communicate over HTTP and UDP.

## Architecture

```
┌─────────────────┐     HTTP/8000      ┌─────────────────┐
│   JetCobot      │◄──────────────────►│   FastAPI       │
│   (Robot Client)│                    │   Server        │
└────────┬────────┘                    └─────────────────┘
         │
         │ UDP/9000 (base64 images)
         ▼
┌─────────────────┐
│   AI Service    │
│   (YOLO)        │
└─────────────────┘
```

- **Server** (`server/`): FastAPI REST API on port 8000. Routes: `/jetcobot/`, `/pinky/`, `/gui/`
- **AI Service** (`ai/`): Async UDP server on port 9000 running YOLO inference. Accepts base64-encoded JPEG images via JSON over UDP
- **JetCobot** (`jetcobot/`): Robot client controlling MyCobot280 arm. Sends images to AI, receives detection results

## Running the Services

Each service has its own virtual environment in `.venv/`:

```sh
# Server (FastAPI)
cd server && . .venv/bin/activate && python main.py

# AI Service (YOLO)
cd ai && . .venv/bin/activate && python main.py

# JetCobot Client
cd jetcobot/204f && . .venv/bin/activate && python main.py
```

## Installing Dependencies

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

## Database

MySQL database `home_ai`. Schema in `databse/home_ai.sql`. Tables: users, robots, robot_types, detections, items, commands, positions, postures, schedules, logs.

## Key Modules

**Robot Control** (`jetcobot/movement/`):
- `motion.py`: Pick-and-place control for MyCobot280 using camera-to-base coordinate transformation
- `vision.py`: Real-time ArUco marker detection, outputs to `target.json`
- Calibration data loaded from `calib_out/` (K.npy, dist.npy, R_cb.npy, t_cb.npy)

**AI Inference** (`ai/`):
- `inference/yolo_model.py`: YOLODetector wrapper around Ultralytics
- `server/udp_server.py`: Async UDP protocol handler
- Model weights: `model/best.pt`

## Communication Protocol

JetCobot → AI Server (UDP):
```json
{"image": "<base64-encoded-jpeg>"}
```

Response contains detection results from YOLO model.
