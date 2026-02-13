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
ros2 run debugcrew vel_sub
```

Publis topic

```sh
ros2 topic pub /porter_target debugcrew_msgs/msg/PorterTarget "{x: 5.0, y: 3.0, w: 0.0}"
```

---

## Packages

- debugcrew
- debugcrew_msgs

---

## Nodes

- vel_sub

---

## Msgs

- PorterTarget.msg

```
float32 x
float32 y
float32 w
```
