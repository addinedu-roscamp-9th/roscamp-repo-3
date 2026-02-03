from pathlib import Path

from ultralytics import YOLO


class YOLODetector:
    def __init__(self, model_path: Path):
        self.model = YOLO(model_path)
        print(f"Model loaded from {model_path}")

    def predict(self, image):
        results = self.model.predict(image, verbose=False)

        detections = []
        for r in results:
            for box in r.boxes:
                detections.append(
                    {
                        "class": int(box.cls),
                        "name": self.model.names[int(box.cls)],
                        "confidence": float(box.conf),
                        "bbox": box.xyxy[0].tolist(),
                    }
                )
        return detections
