# Porter

Transport pinky

## Table of Contents

- [Run](#run)
- [Packages](#packages)
- [Nodes](#nodes)
- [Msgs](#msgs)

---

## Run

```sh
cd ~/dev/roscamp-repo-3/porter
colcon build --packages-select debugcrew_msgs debugcrew
source install/setup.bash
ros2 launch debugcrew pinky.launch.py
```

---

## Packages

- debugcrew
- debugcrew_msgs

---

## Nodes

- vel_sub — receives `/porter_target`, drives Nav2, hands off to PID at < 0.30 m
- pid_node — fine PID positioning, publishes `/porter_status` on arrival

---

## Msgs

### PorterTarget.msg

```
float32 x
float32 y
float32 yaw
```

### PorterStatus.msg

```
string status
float32 x
float32 y
float32 yaw
```
