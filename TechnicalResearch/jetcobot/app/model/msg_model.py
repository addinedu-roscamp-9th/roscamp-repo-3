from pydantic import BaseModel


class ServerMsg(BaseModel):
    msg_type: str
    item: str
