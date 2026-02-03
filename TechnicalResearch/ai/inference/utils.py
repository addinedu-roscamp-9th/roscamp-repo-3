import base64

import cv2
import numpy as np


def decode_image(b64_string: str):
    """Turn a base64 string back into an OpenCV image."""
    img_bytes = base64.b64decode(b64_string)  # convert base64 text to raw bytes
    img_array = np.frombuffer(img_bytes, dtype=np.uint8)  # wrap bytes in a numpy array
    return cv2.imdecode(img_array, cv2.IMREAD_COLOR)  # decompress into a BGR image


def encode_image(image) -> str:
    """Pack an OpenCV image into a base64 string for easy transmission."""
    _, encoded = cv2.imencode(".jpg", image)  # compress the image as JPEG
    return base64.b64encode(encoded).decode()  # convert to base64 text
