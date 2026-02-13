def cmd_pinky(position) -> bool:
    x = position.x
    y = position.y
    w = position.w
    print(x, y, w)

    return True


def wait_pinky(timeout: int) -> bool:
    print(timeout)

    return True
