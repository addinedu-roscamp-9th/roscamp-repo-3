# Server

- [Project Structure](#project-structure)
- [Trouble Shooting](#trouble-shooting)

---

## Project Structure

```
project/
├── app/
│   ├── database/    ←  DB connection and mapper for query
│   ├── routers/     ←  API endpoints (handles requests)
│   ├── services/    ←  Business logic
│   ├── models/      ←  Data structures
│   └── main.py      ←  App entry point
└── run.py       ←  Run this to start the server
```

---

## Environment

Run the `venv_setup.sh` script

```sh
./venv_setup.sh
```

1. Creates `.venv` at current directory
2. Updates PIP
3. Install required packages

Check `.venv` created:

```sh
ls -a # or $ la
```

Start the venv:

```sh
source .venv/bin/activate
```

---

## Trouble Shooting

Pinky should be in same network and have same `ROS_DOMAIN_ID`

### No module named debugcrew_msgs

1. Build the `roscamp-repo-3/porter` package

2. Source `/opt/ros/jazzy/local_setup.bash` and `porter/install/local_setup.bash`

### Navigation action server not available

1. Bringup pinky

2. Load map
