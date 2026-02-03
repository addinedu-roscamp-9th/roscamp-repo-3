import base64

import cv2
import numpy as np


def decode_image(b64_string: str):
    img_bytes = base64.b64decode(b64_string)
    img_array = np.frombuffer(img_bytes, dtype=np.uint8)
    return cv2.imdecode(img_array, cv2.IMREAD_COLOR)


def encode_image(image) -> str:
    _, encoded = cv2.imencode(".jpg", image)
    return base64.b64encode(encoded).decode()
