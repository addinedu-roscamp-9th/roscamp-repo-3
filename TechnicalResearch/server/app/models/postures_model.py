from pydantic import BaseModel, ConfigDict


class Postures(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    pos_id: str
    pos_name: str
    j1: int
    j2: int
    j3: int
    j4: int
    j5: int
    j6: int
    gap: int
