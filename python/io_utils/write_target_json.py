from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Iterable, Optional

from python.common.config import BOARD, SHARED_DIR


def build_target_payload(
    valid: bool,
    source: str = "color",
    confidence: float = 0.0,
    pixel: Optional[Iterable[float]] = None,
    board: Optional[Iterable[float]] = None,
    world: Optional[Iterable[float]] = None,
    timestamp: Optional[float] = None,
) -> dict:
    board_list = list(board) if board is not None else None
    if world is None and board_list is not None:
        world = BOARD.board_to_world((board_list[0], board_list[1]))
    return {
        "valid": bool(valid),
        "source": source,
        "confidence": float(confidence),
        "pixel": list(pixel) if pixel is not None else None,
        "board": board_list,
        "world": list(world) if world is not None else None,
        "timestamp": float(timestamp if timestamp is not None else time.time()),
    }


def write_target_json(payload: dict, path: Path = SHARED_DIR / "target.json") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(".tmp")
    temp_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    os.replace(temp_path, path)
