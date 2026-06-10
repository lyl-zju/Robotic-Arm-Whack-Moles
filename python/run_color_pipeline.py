from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2

from config import SCREENSHOT_DIR, ensure_project_dirs
from detect_color_target import annotate_detection, detect_red_target
from homography import load_default_homography, pixel_to_board
from pybullet_scene import create_scene
from write_target_json import build_target_payload, write_target_json


def run(gui: bool = True, image_path: Path | None = None) -> dict:
    ensure_project_dirs()
    started = time.time()

    if image_path is None:
        scene = create_scene(gui=gui)
        try:
            image_rgb = scene.capture_rgb(SCREENSHOT_DIR / "camera_rgb.png")
        finally:
            scene.close()
    else:
        image_bgr = cv2.imread(str(image_path))
        if image_bgr is None:
            raise FileNotFoundError(f"Cannot read image: {image_path}")
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    detection = detect_red_target(image_rgb, input_rgb=True)
    homography = load_default_homography()

    if detection.valid and detection.pixel is not None:
        board_xy = pixel_to_board(detection.pixel, homography)
        payload = build_target_payload(
            valid=True,
            source="color",
            confidence=detection.confidence,
            pixel=detection.pixel,
            board=board_xy,
            timestamp=time.time() - started,
        )
    else:
        payload = build_target_payload(valid=False, source="color", timestamp=time.time() - started)

    write_target_json(payload)

    annotated = annotate_detection(image_rgb, detection)
    cv2.imwrite(
        str(SCREENSHOT_DIR / "color_detection_annotated.png"),
        cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR),
    )
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run PyBullet capture + red color detection pipeline.")
    parser.add_argument("--gui", action="store_true", help="Show PyBullet GUI window.")
    parser.add_argument("--nogui", action="store_true", help="Run PyBullet in DIRECT mode.")
    parser.add_argument("--image", type=Path, help="Use an existing image instead of capturing from PyBullet.")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    gui = args.gui and not args.nogui
    result = run(gui=gui, image_path=args.image)
    print(result)

