from __future__ import annotations

import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from python.common.config import SCREENSHOT_DIR
from python.simulation.pybullet_scene import create_scene


def capture_random_target(gui: bool = True) -> Path:
    output = SCREENSHOT_DIR / "camera_rgb.png"
    scene = create_scene(gui=gui)
    try:
        scene.capture_rgb(output)
    finally:
        scene.close()
    return output


def main() -> None:
    path = capture_random_target(gui=True)
    print(f"Saved screenshot to {path}")


if __name__ == "__main__":
    main()
