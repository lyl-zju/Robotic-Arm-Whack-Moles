from __future__ import annotations

from pathlib import Path

from config import SCREENSHOT_DIR
from pybullet_scene import create_scene


def capture_random_target(gui: bool = True) -> Path:
    output = SCREENSHOT_DIR / "camera_rgb.png"
    scene = create_scene(gui=gui)
    try:
        scene.capture_rgb(output)
    finally:
        scene.close()
    return output


if __name__ == "__main__":
    path = capture_random_target(gui=True)
    print(f"Saved screenshot to {path}")

