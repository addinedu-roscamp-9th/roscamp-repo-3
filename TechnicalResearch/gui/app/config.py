"""Load runtime configuration from config.json."""

import json
from pathlib import Path

_CONFIG_PATH = Path(__file__).parent.parent / "config.json"

with _CONFIG_PATH.open(encoding="utf-8") as _f:
    _config = json.load(_f)

SERVER_URL: str = _config["server_url"]
