from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SHARED_DIR = ROOT_DIR / "shared"
DATA_DIR = ROOT_DIR / "data"
SCREENSHOT_DIR = DATA_DIR / "screenshots"
CALIBRATION_FILE = DATA_DIR / "calibration" / "board_corners.json"


@dataclass(frozen=True)
class BoardConfig:
    width: float = 0.45
    height: float = 0.30
    rows: int = 3
    cols: int = 3
    center_world: Tuple[float, float, float] = (0.45, 0.0, 0.05)
    target_radius: float = 0.028
    board_thickness: float = 0.015

    @property
    def x_list(self) -> np.ndarray:
        return np.linspace(-0.15, 0.15, self.cols)

    @property
    def y_list(self) -> np.ndarray:
        return np.linspace(-0.10, 0.10, self.rows)

    def hole_positions_board(self) -> List[Tuple[float, float]]:
        return [(float(x), float(y)) for y in self.y_list for x in self.x_list]

    def board_to_world(self, point_board: Tuple[float, float]) -> Tuple[float, float, float]:
        x, y = point_board
        cx, cy, cz = self.center_world
        return (cx + float(x), cy + float(y), cz)


@dataclass(frozen=True)
class CameraConfig:
    width: int = 640
    height: int = 480
    fov: float = 45.0
    near: float = 0.01
    far: float = 2.0
    position: Tuple[float, float, float] = (0.45, 0.0, 0.78)
    target: Tuple[float, float, float] = (0.45, 0.0, 0.05)
    up: Tuple[float, float, float] = (0.0, 1.0, 0.0)


BOARD = BoardConfig()
CAMERA = CameraConfig()


def ensure_project_dirs() -> None:
    for path in [SHARED_DIR, SCREENSHOT_DIR, CALIBRATION_FILE.parent]:
        path.mkdir(parents=True, exist_ok=True)

