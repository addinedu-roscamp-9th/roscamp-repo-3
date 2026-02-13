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
ros2 run debugcrew pinky_node
```

Publis topic

```sh
ros2 topic pub /pink/target debugcrew_msgs/msg/Pinkysy "{x: 5.0, y: 3.0, w: 0.0}"
```

---

## Packages

- debugcrew
- debugcrew_msgs

---

## Nodes

- pinky_node

---

## Msgs

- PinkyTarget.msg

```
float32 x
float32 y
float32 w
```
