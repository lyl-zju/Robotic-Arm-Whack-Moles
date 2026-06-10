from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, Tuple

import cv2
import numpy as np

from config import BOARD, CALIBRATION_FILE


def default_calibration() -> dict:
    return {
        "description": "Replace image_points with measured board corner pixels: left_top, right_top, right_bottom, left_bottom.",
        "image_points": [[128, 96], [512, 96], [512, 384], [128, 384]],
        "board_points": [
            [-BOARD.width / 2, BOARD.height / 2],
            [BOARD.width / 2, BOARD.height / 2],
            [BOARD.width / 2, -BOARD.height / 2],
            [-BOARD.width / 2, -BOARD.height / 2],
        ],
    }


def ensure_calibration_file(path: Path = CALIBRATION_FILE) -> None:
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(default_calibration(), indent=2), encoding="utf-8")


def load_calibration(path: Path = CALIBRATION_FILE) -> Tuple[np.ndarray, np.ndarray]:
    ensure_calibration_file(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    image_points = np.asarray(data["image_points"], dtype=np.float32)
    board_points = np.asarray(data["board_points"], dtype=np.float32)
    if image_points.shape != (4, 2) or board_points.shape != (4, 2):
        raise ValueError("Calibration file must contain four image points and four board points.")
    return image_points, board_points


def compute_homography(image_points: np.ndarray, board_points: np.ndarray) -> np.ndarray:
    homography, status = cv2.findHomography(image_points, board_points)
    if homography is None or status is None:
        raise RuntimeError("Failed to compute homography. Check board corner order.")
    return homography


def pixel_to_board(pixel: Iterable[float], homography: np.ndarray) -> Tuple[float, float]:
    pts = np.asarray([[list(pixel)]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(pts, homography)[0, 0]
    return float(mapped[0]), float(mapped[1])


def load_default_homography() -> np.ndarray:
    image_points, board_points = load_calibration()
    return compute_homography(image_points, board_points)

