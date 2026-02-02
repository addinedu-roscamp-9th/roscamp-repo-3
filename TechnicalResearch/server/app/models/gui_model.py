from dataclasses import dataclass

from pydantic import BaseModel


@dataclass
class GuiData(BaseModel):
    connected: bool
