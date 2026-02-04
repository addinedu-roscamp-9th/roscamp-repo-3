"""
GUI 데이터 모델
"""

from pydantic import BaseModel
from typing import Optional, List


class GuiData(BaseModel):
    robot_id: int
    status: str
    command: str
    destination: Optional[str] = None
    item: Optional[str] = None
    from_room: Optional[str] = None
    schedule_name: Optional[str] = None
    tasks: Optional[List[str]] = None
