from typing import List

from app.model.posture import Posture


class Controller:
    def __init__(self, postures: List[Posture], move):
        self.postures = postures
        self.move = move

    def execute(self) -> bool:
        for posture in self.postures:
            try:
                self.move.execute(posture)
            except Exception as e:
                print(f"Error executing posture: {e}")
                raise

        return True
