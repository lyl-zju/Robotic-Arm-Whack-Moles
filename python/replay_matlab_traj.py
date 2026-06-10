from __future__ import annotations

import csv
from pathlib import Path

from config import SHARED_DIR


def load_q_traj(path: Path = SHARED_DIR / "q_traj.csv") -> list[dict[str, float]]:
    if not path.exists():
        raise FileNotFoundError(f"MATLAB trajectory file not found: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(handle)]


def replay_placeholder(path: Path = SHARED_DIR / "q_traj.csv") -> None:
    rows = load_q_traj(path)
    print(f"Loaded {len(rows)} trajectory samples from {path}")
    print("TODO: load a URDF whose joint order matches MATLAB, then step through q1...qn.")


if __name__ == "__main__":
    replay_placeholder()

