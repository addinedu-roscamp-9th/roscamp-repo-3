from pydantic import BaseModel


class Posture(BaseModel):
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    gap: int
