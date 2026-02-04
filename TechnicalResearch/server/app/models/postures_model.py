from pydantic import BaseModel


class Postures(BaseModel):
    pos_id: str
    pos_name: str
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    angle: int
    gap: int
