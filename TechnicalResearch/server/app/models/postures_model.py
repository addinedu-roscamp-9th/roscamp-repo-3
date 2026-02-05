from pydantic import BaseModel


class Postures(BaseModel):
    pos_id: str
    pos_name: str
    j1: int
    j2: int
    j3: int
    j4: int
    j5: int
    j6: int
    angle: int
    gap: int
