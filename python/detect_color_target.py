from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class ColorDetection:
    valid: bool
    confidence: float
    pixel: Optional[Tuple[float, float]]
    bbox: Optional[Tuple[int, int, int, int]]
    area: float


def detect_red_target(image: np.ndarray, input_rgb: bool = True, min_area: float = 80.0) -> ColorDetection:
    if input_rgb:
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    else:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red_1 = np.array([0, 80, 80])
    upper_red_1 = np.array([12, 255, 255])
    lower_red_2 = np.array([168, 80, 80])
    upper_red_2 = np.array([180, 255, 255])

    mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask_1, mask_2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return ColorDetection(False, 0.0, None, None, 0.0)

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return ColorDetection(False, 0.0, None, None, area)

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return ColorDetection(False, 0.0, None, None, area)

    u = float(moments["m10"] / moments["m00"])
    v = float(moments["m01"] / moments["m00"])
    x, y, w, h = cv2.boundingRect(contour)
    image_area = float(image.shape[0] * image.shape[1])
    confidence = min(1.0, area / max(image_area * 0.004, 1.0))
    return ColorDetection(True, confidence, (u, v), (x, y, x + w, y + h), area)


def annotate_detection(image_rgb: np.ndarray, detection: ColorDetection) -> np.ndarray:
    annotated = image_rgb.copy()
    if detection.valid and detection.pixel is not None and detection.bbox is not None:
        x1, y1, x2, y2 = detection.bbox
        u, v = detection.pixel
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(annotated, (int(round(u)), int(round(v))), 5, (0, 255, 255), -1)
    return annotated

