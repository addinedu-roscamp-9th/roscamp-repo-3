from pathlib import Path

from ultralytics import YOLO


class YOLODetector:
    """Wrapper around YOLO to make detection easy and return clean results."""

    def __init__(self, model_path: Path):
        self.model = YOLO(model_path)  # load the pretrained YOLO model from disk
        print(f"Model loaded from {model_path}")

    def predict(self, image):
        """Run detection on an image and return a list of found objects."""
        results = self.model.predict(image, verbose=False)  # run inference quietly
        detections = []

        for r in results:
            for box in r.boxes:  # loop through each detected object
                detections.append(
                    {
                        "class": int(box.cls),  # numeric class ID
                        "name": self.model.names[
                            int(box.cls)
                        ],  # human-readable label like "car" or "person"
                        "confidence": float(box.conf),  # how sure the model is (0 to 1)
                        "bbox": box.xyxy[
                            0
                        ].tolist(),  # bounding box as [x1, y1, x2, y2]
                    }
                )

        return detections
