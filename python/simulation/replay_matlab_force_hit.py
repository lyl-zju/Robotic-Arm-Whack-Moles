from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from python.common.config import BOARD, ROOT_DIR, SHARED_DIR, ensure_project_dirs


RESULTS_DATA_DIR = ROOT_DIR / "results" / "data"
DEFAULT_LOG_PATH = RESULTS_DATA_DIR / "pybullet_force_log.csv"
DEFAULT_SUMMARY_PATH = SHARED_DIR / "pybullet_force_result.json"
DEFAULT_LOCAL_UR5_URDF = ROOT_DIR / "data" / "urdf" / "ur5" / "ur5_robot.urdf"


@dataclass(frozen=True)
class TrajSample:
    t: float
    q: list[float]


@dataclass(frozen=True)
class TargetInfo:
    world: tuple[float, float, float]
    source: str


@dataclass
class ReplayStats:
    peak_force: float = 0.0
    success: bool = False
    success_time: Optional[float] = None
    max_contact_count: int = 0


def import_pybullet():
    try:
        import pybullet as p
    except ImportError as exc:
        raise SystemExit(
            "Missing pybullet. Install dependencies with: pip install -r requirements.txt"
        ) from exc

    try:
        import pybullet_data
    except ImportError:
        pybullet_data = None

    return p, pybullet_data


def load_q_traj(path: Path) -> list[TrajSample]:
    if not path.exists():
        raise FileNotFoundError(f"MATLAB trajectory file not found: {path}")

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "t" not in reader.fieldnames:
            raise ValueError(f"Trajectory CSV must contain a 't' column: {path}")

        q_keys = [name for name in reader.fieldnames if name.lower().startswith("q")]
        q_keys.sort(key=lambda name: int(name[1:]) if name[1:].isdigit() else 9999)
        if not q_keys:
            raise ValueError(f"Trajectory CSV must contain q1...qn columns: {path}")

        samples = [
            TrajSample(t=float(row["t"]), q=[float(row[key]) for key in q_keys])
            for row in reader
        ]

    if len(samples) < 2:
        raise ValueError("Trajectory must contain at least two samples.")
    return samples


def read_json_if_exists(path: Path) -> Optional[dict]:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def parse_world(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError("Target world coordinates must contain exactly 3 numbers.")
    return float(values[0]), float(values[1]), float(values[2])


def resolve_target(
    explicit_world: Optional[Sequence[float]],
    result_path: Path,
    target_json_path: Path,
) -> TargetInfo:
    if explicit_world is not None:
        return TargetInfo(parse_world(explicit_world), "cli")

    result = read_json_if_exists(result_path)
    if result and "target_world" in result:
        return TargetInfo(parse_world(result["target_world"]), "result.json")

    target = read_json_if_exists(target_json_path)
    if target and target.get("valid", False):
        if target.get("world") is not None:
            return TargetInfo(parse_world(target["world"]), "target.json:world")
        if target.get("board") is not None:
            bx, by = float(target["board"][0]), float(target["board"][1])
            return TargetInfo(BOARD.board_to_world((bx, by)), "target.json:board")

    raise FileNotFoundError(
        "Cannot resolve target point. Run MATLAB first so shared/result.json exists, "
        "or pass --target-world X Y Z."
    )


def resolve_threshold(cli_threshold: Optional[float], result_path: Path) -> float:
    if cli_threshold is not None:
        return float(cli_threshold)
    result = read_json_if_exists(result_path)
    if result and "force_threshold" in result:
        return float(result["force_threshold"])
    return 10.0


def sample_q(samples: list[TrajSample], t: float, cursor: int) -> tuple[list[float], int]:
    if t <= samples[0].t:
        return samples[0].q, 0
    if t >= samples[-1].t:
        return samples[-1].q, len(samples) - 2

    while cursor < len(samples) - 2 and samples[cursor + 1].t < t:
        cursor += 1
    while cursor > 0 and samples[cursor].t > t:
        cursor -= 1

    a = samples[cursor]
    b = samples[cursor + 1]
    span = max(b.t - a.t, 1e-9)
    ratio = (t - a.t) / span
    q = [qa + ratio * (qb - qa) for qa, qb in zip(a.q, b.q)]
    return q, cursor


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def vec_lerp(a: Sequence[float], b: Sequence[float], ratio: float) -> list[float]:
    return [lerp(float(x), float(y), ratio) for x, y in zip(a, b)]


def clamp_vec_norm(v: Sequence[float], max_norm: float) -> list[float]:
    norm = math.sqrt(sum(float(x) * float(x) for x in v))
    if norm <= max_norm or norm < 1e-12:
        return [float(x) for x in v]
    scale = max_norm / norm
    return [float(x) * scale for x in v]


def find_robot_urdf(pybullet_data, explicit_path: Optional[Path]) -> Optional[Path]:
    if explicit_path is not None:
        normalized = str(explicit_path).replace("\\", "/").lower()
        if normalized in {"path/to/ur5.urdf", "path/to/robot.urdf"}:
            print(
                f"Placeholder URDF path '{explicit_path}' was provided; "
                f"using local UR5 model instead: {DEFAULT_LOCAL_UR5_URDF}"
            )
            return DEFAULT_LOCAL_UR5_URDF if DEFAULT_LOCAL_UR5_URDF.exists() else None
        if not explicit_path.exists():
            raise FileNotFoundError(f"Robot URDF does not exist: {explicit_path}")
        return explicit_path

    if DEFAULT_LOCAL_UR5_URDF.exists():
        return DEFAULT_LOCAL_UR5_URDF

    if pybullet_data is None:
        return None

    data_root = Path(pybullet_data.getDataPath())
    preferred = [
        "ur5/ur5.urdf",
        "ur5/ur5_robot.urdf",
        "urdf/ur5.urdf",
        "ur5.urdf",
        "xarm/xarm6_robot.urdf",
        "kuka_iiwa/model.urdf",
        "franka_panda/panda.urdf",
    ]
    for rel_path in preferred:
        path = data_root / rel_path
        if path.exists():
            return path

    keywords = ("ur5", "universal", "xarm6")
    for path in data_root.rglob("*.urdf"):
        low = str(path).lower()
        if any(keyword in low for keyword in keywords):
            return path
    return None


def pybullet_load_path(path: Path) -> str:
    try:
        return os.path.relpath(path.resolve(), Path.cwd())
    except ValueError:
        return str(path)


def connect(p, gui: bool, physics_hz: int) -> int:
    mode = p.GUI if gui else p.DIRECT
    client_id = p.connect(mode)
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    dt = 1.0 / float(physics_hz)
    p.setTimeStep(dt)
    p.setPhysicsEngineParameter(
        fixedTimeStep=dt,
        numSolverIterations=180,
        contactBreakingThreshold=0.001,
    )
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.95,
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=[BOARD.center_world[0], BOARD.center_world[1], 0.06],
        )
    return client_id


def add_plane(p) -> None:
    try:
        p.loadURDF("plane.urdf")
    except Exception:
        shape = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape)


def set_contact_dynamics(p, body_id: int, link_id: int = -1) -> None:
    try:
        p.changeDynamics(
            body_id,
            link_id,
            lateralFriction=0.9,
            restitution=0.0,
            contactStiffness=25000.0,
            contactDamping=800.0,
        )
    except TypeError:
        p.changeDynamics(body_id, link_id, lateralFriction=0.9, restitution=0.0)


def build_board_and_target(
    p,
    target_world: tuple[float, float, float],
    mole_radius: float,
    mole_height: float,
    z_hit_offset: float,
) -> tuple[int, list[float], list[float]]:
    cx, cy, cz = BOARD.center_world
    z_hit = cz + z_hit_offset

    board_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[BOARD.width / 2, BOARD.height / 2, BOARD.board_thickness / 2],
        rgbaColor=[0.16, 0.18, 0.20, 1.0],
    )
    board_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[BOARD.width / 2, BOARD.height / 2, BOARD.board_thickness / 2],
    )
    board_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=board_collision,
        baseVisualShapeIndex=board_visual,
        basePosition=[cx, cy, cz - BOARD.board_thickness / 2],
    )
    set_contact_dynamics(p, board_id)

    hole_visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=BOARD.target_radius * 1.18,
        length=0.004,
        rgbaColor=[0.02, 0.02, 0.025, 1.0],
    )
    for bx, by in BOARD.hole_positions_board():
        wx, wy, _ = BOARD.board_to_world((bx, by))
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=hole_visual,
            basePosition=[wx, wy, cz + 0.004],
        )

    target_x, target_y, _ = target_world
    mole_center_up = [target_x, target_y, z_hit - mole_height / 2]
    mole_center_down = [target_x, target_y, cz - mole_height]
    mole_visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=mole_radius,
        length=mole_height,
        rgbaColor=[1.0, 0.03, 0.02, 1.0],
    )
    mole_collision = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=mole_radius,
        height=mole_height,
    )
    mole_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=mole_collision,
        baseVisualShapeIndex=mole_visual,
        basePosition=mole_center_up,
    )
    set_contact_dynamics(p, mole_id)

    marker_visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=mole_radius * 0.35,
        rgbaColor=[1.0, 1.0, 0.0, 1.0],
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=marker_visual,
        basePosition=[target_x, target_y, z_hit + 0.035],
    )

    return mole_id, mole_center_up, mole_center_down


def get_controllable_joints(p, robot_id: int, q_count: int) -> list[int]:
    movable_types = {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}
    joints = []
    for index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, index)
        if info[2] in movable_types:
            joints.append(index)
    if len(joints) < q_count:
        raise RuntimeError(
            f"Robot has {len(joints)} movable joints, but MATLAB trajectory has {q_count} joints."
        )
    return joints[:q_count]


def choose_ee_link(p, robot_id: int, joint_indices: list[int]) -> int:
    preferred_names = {"tool0", "ee_link", "wrist_3_link", "flange", "panda_hand"}
    for index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, index)
        link_name = info[12].decode("utf-8", errors="ignore")
        if link_name in preferred_names:
            return index
    return joint_indices[-1]


def create_hammer_body(p, pos: Sequence[float], radius: float, mass: float) -> int:
    visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=[0.95, 0.72, 0.12, 1.0],
    )
    collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    hammer_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=list(pos),
    )
    set_contact_dynamics(p, hammer_id)
    return hammer_id


def sum_contact_force(p, body_a: int, body_b: int) -> tuple[float, int]:
    contacts = p.getContactPoints(bodyA=body_a, bodyB=body_b)
    force = sum(max(0.0, float(contact[9])) for contact in contacts)
    return force, len(contacts)


def update_overlay(
    p,
    text_id: int,
    t: float,
    force: float,
    stats: ReplayStats,
    threshold: float,
    position: Sequence[float],
) -> int:
    status = "SUCCESS" if stats.success else "WAITING"
    color = [0.1, 0.9, 0.2] if stats.success else [1.0, 0.85, 0.1]
    text = (
        f"t={t:5.3f}s  F={force:7.2f}N  "
        f"peak={stats.peak_force:7.2f}N  threshold={threshold:.2f}N  {status}"
    )
    return p.addUserDebugText(
        text,
        list(position),
        textColorRGB=color,
        textSize=1.15,
        lifeTime=0.0,
        replaceItemUniqueId=text_id,
    )


def update_stats(
    stats: ReplayStats,
    t: float,
    force: float,
    contact_count: int,
    threshold: float,
) -> None:
    stats.peak_force = max(stats.peak_force, force)
    stats.max_contact_count = max(stats.max_contact_count, contact_count)
    if not stats.success and stats.peak_force >= threshold:
        stats.success = True
        stats.success_time = t


def retract_mole_if_needed(
    p,
    mole_id: int,
    t: float,
    stats: ReplayStats,
    mole_center_up: Sequence[float],
    mole_center_down: Sequence[float],
    retract_time: float,
) -> None:
    if not stats.success or stats.success_time is None:
        return
    ratio = smoothstep((t - stats.success_time) / max(retract_time, 1e-6))
    pos = vec_lerp(mole_center_up, mole_center_down, ratio)
    p.resetBasePositionAndOrientation(mole_id, pos, [0, 0, 0, 1])


def write_log(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "t",
        "force",
        "peak_force",
        "contact_count",
        "success",
        "hammer_x",
        "hammer_y",
        "hammer_z",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(
    path: Path,
    stats: ReplayStats,
    threshold: float,
    target: TargetInfo,
    mode: str,
    log_path: Path,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "success": bool(stats.success),
        "peak_force": float(stats.peak_force),
        "force_threshold": float(threshold),
        "success_time": stats.success_time,
        "max_contact_count": int(stats.max_contact_count),
        "target_source": target.source,
        "target_world": list(target.world),
        "mode": mode,
        "log": str(log_path),
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def replay_with_robot(
    p,
    pybullet_data,
    samples: list[TrajSample],
    args: argparse.Namespace,
    mole_id: int,
    mole_center_up: Sequence[float],
    mole_center_down: Sequence[float],
    threshold: float,
) -> tuple[ReplayStats, list[dict[str, float]]]:
    urdf_path = find_robot_urdf(pybullet_data, args.robot_urdf)
    if urdf_path is None:
        raise RuntimeError("No suitable robot URDF found in pybullet_data.")

    if pybullet_data is not None:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    robot_start_pos = [args.robot_base[0], args.robot_base[1], args.robot_base[2]]
    robot_id = p.loadURDF(
        pybullet_load_path(urdf_path),
        robot_start_pos,
        p.getQuaternionFromEuler(args.robot_rpy),
        useFixedBase=True,
    )
    joint_indices = get_controllable_joints(p, robot_id, len(samples[0].q))
    ee_link = choose_ee_link(p, robot_id, joint_indices)

    for joint_index, q in zip(joint_indices, samples[0].q):
        p.resetJointState(robot_id, joint_index, q)
    p.stepSimulation()

    ee_state = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True)
    hammer_id = create_hammer_body(p, ee_state[0], args.hammer_radius, args.hammer_mass)
    p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_link,
        childBodyUniqueId=hammer_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=args.hammer_offset,
        childFramePosition=[0, 0, 0],
    )

    print(f"Loaded robot URDF: {urdf_path}")
    print(f"Controlled joints: {joint_indices}")
    print(f"End-effector link index: {ee_link}")

    return run_replay_loop(
        p=p,
        samples=samples,
        args=args,
        threshold=threshold,
        mole_id=mole_id,
        mole_center_up=mole_center_up,
        mole_center_down=mole_center_down,
        contact_body_id=hammer_id,
        command_callback=lambda t, q: p.setJointMotorControlArray(
            robot_id,
            joint_indices,
            p.POSITION_CONTROL,
            targetPositions=q,
            forces=[args.motor_force] * len(joint_indices),
            positionGains=[args.position_gain] * len(joint_indices),
            velocityGains=[args.velocity_gain] * len(joint_indices),
        ),
        hammer_position_callback=lambda: p.getBasePositionAndOrientation(hammer_id)[0],
    )


def hammer_tip_path(
    t: float,
    total_time: float,
    target_world: tuple[float, float, float],
    z_hit: float,
    press_depth: float,
) -> list[float]:
    x, y, _ = target_world
    hover_z = z_hit + 0.10
    start_z = z_hit + 0.20
    contact_z = z_hit
    press_z = z_hit - press_depth

    u = max(0.0, min(1.0, t / max(total_time, 1e-6)))
    if u < 0.25:
        r = smoothstep(u / 0.25)
        z = lerp(start_z, hover_z, r)
    elif u < 0.50:
        r = smoothstep((u - 0.25) / 0.25)
        z = lerp(hover_z, contact_z, r)
    elif u < 0.75:
        r = smoothstep((u - 0.50) / 0.25)
        z = lerp(contact_z, press_z, r)
    else:
        r = smoothstep((u - 0.75) / 0.25)
        z = lerp(press_z, hover_z, r)
    return [x, y, z]


def replay_with_hammer_proxy(
    p,
    samples: list[TrajSample],
    args: argparse.Namespace,
    target_world: tuple[float, float, float],
    mole_id: int,
    mole_center_up: Sequence[float],
    mole_center_down: Sequence[float],
    threshold: float,
) -> tuple[ReplayStats, list[dict[str, float]]]:
    z_hit = BOARD.center_world[2] + args.z_hit_offset
    tip_start = hammer_tip_path(0.0, samples[-1].t, target_world, z_hit, args.press_depth)
    center_start = [tip_start[0], tip_start[1], tip_start[2] + args.hammer_radius]
    hammer_id = create_hammer_body(p, center_start, args.hammer_radius, args.hammer_mass)

    print("Running hammer-proxy replay.")
    print("This mode uses q_traj.csv for timing and drives a simple dynamic hammer at the MATLAB target point.")

    def command(t: float, _q: list[float]) -> None:
        tip = hammer_tip_path(t, samples[-1].t, target_world, z_hit, args.press_depth)
        desired = [tip[0], tip[1], tip[2] + args.hammer_radius]
        pos, _ = p.getBasePositionAndOrientation(hammer_id)
        vel = [(desired[i] - pos[i]) * args.hammer_servo_gain for i in range(3)]
        vel = clamp_vec_norm(vel, args.max_hammer_speed)
        p.resetBaseVelocity(hammer_id, linearVelocity=vel, angularVelocity=[0, 0, 0])

    return run_replay_loop(
        p=p,
        samples=samples,
        args=args,
        threshold=threshold,
        mole_id=mole_id,
        mole_center_up=mole_center_up,
        mole_center_down=mole_center_down,
        contact_body_id=hammer_id,
        command_callback=command,
        hammer_position_callback=lambda: p.getBasePositionAndOrientation(hammer_id)[0],
    )


def run_replay_loop(
    p,
    samples: list[TrajSample],
    args: argparse.Namespace,
    threshold: float,
    mole_id: int,
    mole_center_up: Sequence[float],
    mole_center_down: Sequence[float],
    contact_body_id: int,
    command_callback,
    hammer_position_callback,
) -> tuple[ReplayStats, list[dict[str, float]]]:
    dt = 1.0 / float(args.physics_hz)
    total_time = samples[-1].t + args.extra_time
    cursor = 0
    stats = ReplayStats()
    log_rows: list[dict[str, float]] = []
    text_id = -1
    next_overlay_t = -1.0

    steps = int(math.ceil(total_time / dt))
    for step_index in range(steps + 1):
        t = step_index * dt
        q, cursor = sample_q(samples, min(t, samples[-1].t), cursor)
        command_callback(t, q)

        p.stepSimulation()

        force, contact_count = sum_contact_force(p, contact_body_id, mole_id)
        update_stats(stats, t, force, contact_count, threshold)
        retract_mole_if_needed(
            p,
            mole_id,
            t,
            stats,
            mole_center_up,
            mole_center_down,
            args.retract_time,
        )

        hammer_pos = hammer_position_callback()
        if args.gui and t >= next_overlay_t:
            text_id = update_overlay(
                p,
                text_id,
                t,
                force,
                stats,
                threshold,
                [BOARD.center_world[0] - 0.28, BOARD.center_world[1] - 0.28, 0.34],
            )
            next_overlay_t = t + 1.0 / 30.0

        if step_index % max(1, int(args.physics_hz / args.log_hz)) == 0:
            log_rows.append(
                {
                    "t": t,
                    "force": force,
                    "peak_force": stats.peak_force,
                    "contact_count": contact_count,
                    "success": 1.0 if stats.success else 0.0,
                    "hammer_x": float(hammer_pos[0]),
                    "hammer_y": float(hammer_pos[1]),
                    "hammer_z": float(hammer_pos[2]),
                }
            )

        if args.gui and args.real_time:
            time.sleep(dt / max(args.time_scale, 1e-6))

    return stats, log_rows


def parse_vec3(values: Optional[Iterable[str]]) -> Optional[list[float]]:
    if values is None:
        return None
    parsed = [float(v) for v in values]
    if len(parsed) != 3:
        raise argparse.ArgumentTypeError("Expected exactly three values.")
    return parsed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Replay MATLAB q_traj.csv in PyBullet, display live contact force, "
            "and decide whether the mole is knocked down."
        )
    )
    parser.add_argument("--traj", type=Path, default=SHARED_DIR / "q_traj.csv")
    parser.add_argument("--result", type=Path, default=SHARED_DIR / "result.json")
    parser.add_argument("--target-json", type=Path, default=SHARED_DIR / "target.json")
    parser.add_argument("--target-world", nargs=3, type=float)
    parser.add_argument("--threshold", type=float)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG_PATH)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY_PATH)
    parser.add_argument("--mode", choices=["auto", "robot", "hammer"], default="auto")
    parser.add_argument("--gui", dest="gui", action="store_true", default=True)
    parser.add_argument("--nogui", dest="gui", action="store_false")
    parser.add_argument("--real-time", dest="real_time", action="store_true", default=True)
    parser.add_argument("--fast", dest="real_time", action="store_false")
    parser.add_argument("--time-scale", type=float, default=1.0)
    parser.add_argument("--physics-hz", type=int, default=1000)
    parser.add_argument("--log-hz", type=int, default=200)
    parser.add_argument("--extra-time", type=float, default=0.6)

    parser.add_argument("--robot-urdf", type=Path)
    parser.add_argument("--robot-base", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--robot-rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--motor-force", type=float, default=700.0)
    parser.add_argument("--position-gain", type=float, default=0.35)
    parser.add_argument("--velocity-gain", type=float, default=1.0)

    parser.add_argument("--hammer-radius", type=float, default=0.022)
    parser.add_argument("--hammer-mass", type=float, default=0.35)
    parser.add_argument("--hammer-offset", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--hammer-servo-gain", type=float, default=35.0)
    parser.add_argument("--max-hammer-speed", type=float, default=1.5)

    parser.add_argument("--mole-radius", type=float, default=BOARD.target_radius)
    parser.add_argument("--mole-height", type=float, default=0.035)
    parser.add_argument("--z-hit-offset", type=float, default=0.01)
    parser.add_argument("--press-depth", type=float, default=0.012)
    parser.add_argument("--retract-time", type=float, default=0.25)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    ensure_project_dirs()
    RESULTS_DATA_DIR.mkdir(parents=True, exist_ok=True)
    args = build_arg_parser().parse_args(argv)

    samples = load_q_traj(args.traj)
    target = resolve_target(args.target_world, args.result, args.target_json)
    threshold = resolve_threshold(args.threshold, args.result)

    p, pybullet_data = import_pybullet()
    client_id = connect(p, args.gui, args.physics_hz)
    try:
        if pybullet_data is not None:
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
        add_plane(p)
        mole_id, mole_center_up, mole_center_down = build_board_and_target(
            p,
            target.world,
            args.mole_radius,
            args.mole_height,
            args.z_hit_offset,
        )

        print(f"Trajectory samples: {len(samples)} from {args.traj}")
        print(f"Target source: {target.source}, world={target.world}")
        print(f"Force threshold: {threshold:.3f} N")

        mode = args.mode
        if mode == "auto":
            mode = "robot" if find_robot_urdf(pybullet_data, args.robot_urdf) is not None else "hammer"
        actual_mode = mode

        if mode == "robot":
            try:
                stats, log_rows = replay_with_robot(
                    p,
                    pybullet_data,
                    samples,
                    args,
                    mole_id,
                    mole_center_up,
                    mole_center_down,
                    threshold,
                )
            except Exception as exc:
                if args.mode == "robot":
                    raise
                print(f"Robot replay unavailable ({exc}). Falling back to hammer proxy.")
                actual_mode = "hammer"
                stats, log_rows = replay_with_hammer_proxy(
                    p,
                    samples,
                    args,
                    target.world,
                    mole_id,
                    mole_center_up,
                    mole_center_down,
                    threshold,
                )
        else:
            stats, log_rows = replay_with_hammer_proxy(
                p,
                samples,
                args,
                target.world,
                mole_id,
                mole_center_up,
                mole_center_down,
                threshold,
            )

        write_log(args.log, log_rows)
        write_summary(args.summary, stats, threshold, target, actual_mode, args.log)
        print("")
        print("=== PyBullet force replay result ===")
        print(f"peak_force: {stats.peak_force:.3f} N")
        print(f"threshold:  {threshold:.3f} N")
        print(f"success:    {stats.success}")
        print(f"success_t:  {stats.success_time if stats.success_time is not None else 'n/a'}")
        print(f"log:        {args.log}")
        print(f"summary:    {args.summary}")

        if args.gui:
            print("Close the PyBullet window to finish, or press Ctrl+C in this terminal.")
            while p.isConnected(client_id):
                time.sleep(0.1)
    finally:
        if p.isConnected(client_id):
            p.disconnect(client_id)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
