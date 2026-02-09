from pydantic import BaseModel


class Item(BaseModel):
    item_id: str
    item_name: str
    amount: int
    frequency: int
