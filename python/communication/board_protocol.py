from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class BoardConfig:
    x_list: tuple[float, float, float] = (-0.20, 0.0, 0.20)
    y_list: tuple[float, float, float] = (-0.13, 0.0, 0.13)
    center_world: tuple[float, float, float] = (0.45, 0.0, 0.05)

    def target(self, target_id: int) -> dict[str, object]:
        if target_id < 1 or target_id > 9:
            raise ValueError("target_id must be in [1, 9].")

        row = (target_id - 1) // 3 + 1
        col = (target_id - 1) % 3 + 1
        board_x = self.x_list[col - 1]
        board_y = self.y_list[row - 1]
        world = (
            self.center_world[0] + board_x,
            self.center_world[1] + board_y,
            self.center_world[2],
        )

        return {
            "target_id": target_id,
            "row": row,
            "col": col,
            "board": [board_x, board_y],
            "world": list(world),
            "x": world[0],
            "y": world[1],
            "z": world[2],
        }


def build_payload(
    seq: int,
    target: dict[str, object],
    source: str,
    extra: dict[str, Any] | None = None,
) -> bytes:
    payload = {
        "valid": True,
        "source": source,
        "seq": seq,
        "timestamp": time.time(),
        **target,
    }
    if extra:
        payload.update(extra)
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
