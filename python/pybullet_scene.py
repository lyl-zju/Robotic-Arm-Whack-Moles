from __future__ import annotations

import random
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from config import BOARD, CAMERA, BoardConfig, CameraConfig, ensure_project_dirs

try:
    import pybullet as p
    import pybullet_data
except ImportError as exc:
    raise SystemExit("Missing pybullet. Install dependencies with: pip install -r requirements.txt") from exc


class WhacAMoleScene:
    def __init__(
        self,
        board: BoardConfig = BOARD,
        camera: CameraConfig = CAMERA,
        gui: bool = True,
    ) -> None:
        self.board = board
        self.camera = camera
        self.gui = gui
        self.client_id: Optional[int] = None
        self.target_body_id: Optional[int] = None
        self.target_board_xy: Optional[Tuple[float, float]] = None

    def connect(self) -> None:
        mode = p.GUI if self.gui else p.DIRECT
        self.client_id = p.connect(mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

    def build_board(self) -> None:
        cx, cy, cz = self.board.center_world
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.board.width / 2, self.board.height / 2, self.board.board_thickness / 2],
            rgbaColor=[0.16, 0.18, 0.20, 1.0],
        )
        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.board.width / 2, self.board.height / 2, self.board.board_thickness / 2],
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[cx, cy, cz - self.board.board_thickness / 2],
        )

        for x, y in self.board.hole_positions_board():
            hole_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=self.board.target_radius * 1.18,
                length=0.006,
                rgbaColor=[0.03, 0.03, 0.035, 1.0],
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=hole_visual,
                basePosition=[cx + x, cy + y, cz + 0.004],
            )

    def reset_target(self, board_xy: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        if self.target_body_id is not None:
            p.removeBody(self.target_body_id)
            self.target_body_id = None

        if board_xy is None:
            board_xy = random.choice(self.board.hole_positions_board())

        wx, wy, wz = self.board.board_to_world(board_xy)
        target_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=self.board.target_radius,
            length=0.025,
            rgbaColor=[1.0, 0.02, 0.02, 1.0],
        )
        self.target_body_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=target_visual,
            basePosition=[wx, wy, wz + 0.018],
        )
        self.target_board_xy = board_xy
        return board_xy

    def capture_rgb(self, save_path: Optional[Path] = None) -> np.ndarray:
        view = p.computeViewMatrix(
            cameraEyePosition=self.camera.position,
            cameraTargetPosition=self.camera.target,
            cameraUpVector=self.camera.up,
        )
        projection = p.computeProjectionMatrixFOV(
            fov=self.camera.fov,
            aspect=self.camera.width / self.camera.height,
            nearVal=self.camera.near,
            farVal=self.camera.far,
        )
        _, _, rgba, _, _ = p.getCameraImage(
            width=self.camera.width,
            height=self.camera.height,
            viewMatrix=view,
            projectionMatrix=projection,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
        image = np.reshape(np.array(rgba, dtype=np.uint8), (self.camera.height, self.camera.width, 4))[:, :, :3]
        if save_path is not None:
            save_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(save_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        return image

    def step(self, steps: int = 30) -> None:
        for _ in range(steps):
            p.stepSimulation()

    def close(self) -> None:
        if self.client_id is not None:
            p.disconnect(self.client_id)
            self.client_id = None


def create_scene(gui: bool = True, target: Optional[Tuple[float, float]] = None) -> WhacAMoleScene:
    ensure_project_dirs()
    scene = WhacAMoleScene(gui=gui)
    scene.connect()
    scene.build_board()
    scene.reset_target(target)
    scene.step()
    return scene

