from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable

from .board_protocol import BoardConfig, build_payload


def default_detector_script() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root.parent / "test_virtual_camera" / "red_grid_detector.py"


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the vision grid detector and forward changed target_id values "
            "to MATLAB over UDP."
        )
    )
    parser.add_argument("--host", default="127.0.0.1", help="MATLAB receiver host.")
    parser.add_argument("--port", type=int, default=5005, help="MATLAB UDP local port.")
    parser.add_argument(
        "--detector-script",
        type=Path,
        default=default_detector_script(),
        help="Path to red_grid_detector.py.",
    )
    parser.add_argument(
        "--detector-python",
        default=sys.executable,
        help="Python executable used to run the detector.",
    )
    parser.add_argument(
        "--source",
        default="vision_grid_detector",
        help="Source string stored in UDP payloads.",
    )
    parser.add_argument(
        "--resend-same-after-s",
        type=float,
        default=0.0,
        help=(
            "Resend an unchanged target after this many seconds. "
            "Use 0 to send only target_id changes."
        ),
    )
    parser.add_argument(
        "--stable-frames",
        type=int,
        default=5,
        help=(
            "Number of consecutive detector JSON outputs with the same valid "
            "target_id required before sending to MATLAB."
        ),
    )
    parser.add_argument(
        "--echo-detector-log",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Print non-JSON detector output with a detector> prefix.",
    )

    args, detector_args = parser.parse_known_args(argv)
    if detector_args and detector_args[0] == "--":
        detector_args = detector_args[1:]
    args.detector_args = detector_args
    return args


def parse_detector_payload(line: str) -> dict[str, Any] | None:
    line = line.strip()
    if not line.startswith("{") or not line.endswith("}"):
        return None
    try:
        payload = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(payload, dict):
        return None
    return payload


def valid_target_id(payload: dict[str, Any]) -> int | None:
    if not payload.get("valid", False):
        return None
    try:
        target_id = int(payload["target_id"])
    except (KeyError, TypeError, ValueError):
        return None
    if 1 <= target_id <= 9:
        return target_id
    return None


def should_send_target(
    target_id: int,
    last_sent_id: int | None,
    last_sent_time: float,
    resend_same_after_s: float,
) -> bool:
    if target_id != last_sent_id:
        return True
    if resend_same_after_s <= 0:
        return False
    return time.monotonic() - last_sent_time >= resend_same_after_s


def has_detector_option(detector_args: list[str], option: str) -> bool:
    prefix = f"{option}="
    return any(arg == option or arg.startswith(prefix) for arg in detector_args)


def detector_command(args: argparse.Namespace) -> list[str]:
    script = args.detector_script.resolve()
    command = [args.detector_python, "-u", str(script)]
    if not has_detector_option(args.detector_args, "--print-every"):
        command.extend(["--print-every", "0"])
    command.extend(args.detector_args)
    return command


def stream_vision_targets(args: argparse.Namespace) -> int:
    detector_script = args.detector_script.resolve()
    if not detector_script.exists():
        raise FileNotFoundError(f"Detector script not found: {detector_script}")
    if args.resend_same_after_s < 0:
        raise ValueError("--resend-same-after-s must be non-negative.")
    if args.stable_frames < 1:
        raise ValueError("--stable-frames must be at least 1.")

    command = detector_command(args)
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env.setdefault("PYTHONIOENCODING", "utf-8")

    board = BoardConfig()
    address = (args.host, args.port)
    seq = 0
    last_sent_id: int | None = None
    last_sent_time = 0.0
    last_invalid_method = ""
    stable_candidate_id: int | None = None
    stable_count = 0

    print(f"Starting detector: {' '.join(command)}")
    print(
        f"Forwarding target_id values to {args.host}:{args.port} after "
        f"{args.stable_frames} stable frames."
    )
    print("Press Ctrl+C in this terminal to stop both detector and sender.")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        process = subprocess.Popen(
            command,
            cwd=str(detector_script.parent),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        try:
            assert process.stdout is not None
            for line in process.stdout:
                payload = parse_detector_payload(line)
                if payload is None:
                    if args.echo_detector_log:
                        print(f"detector> {line.rstrip()}", flush=True)
                    continue

                target_id = valid_target_id(payload)
                if target_id is None:
                    stable_candidate_id = None
                    stable_count = 0
                    method = str(payload.get("method", "invalid"))
                    if method != last_invalid_method:
                        print(f"vision invalid: method={method}", flush=True)
                        last_invalid_method = method
                    continue

                if target_id == stable_candidate_id:
                    stable_count += 1
                else:
                    stable_candidate_id = target_id
                    stable_count = 1
                    print(
                        f"vision candidate id={target_id} stable={stable_count}/{args.stable_frames}",
                        flush=True,
                    )

                if stable_count < args.stable_frames:
                    continue

                if not should_send_target(
                    target_id,
                    last_sent_id,
                    last_sent_time,
                    args.resend_same_after_s,
                ):
                    continue

                target = board.target(target_id)
                extra = {
                    "vision_method": payload.get("method"),
                    "vision_confidence": payload.get("confidence"),
                    "vision_center": payload.get("center"),
                    "vision_area": payload.get("area"),
                    "vision_radius": payload.get("radius"),
                    "vision_timestamp": payload.get("timestamp"),
                    "vision_stable_frames": stable_count,
                }
                udp_socket.sendto(build_payload(seq, target, args.source, extra), address)
                last_sent_id = target_id
                last_sent_time = time.monotonic()
                last_invalid_method = ""

                print(
                    "sent seq={seq:06d} id={target_id} row={row} col={col} "
                    "stable={stable_count} method={method} confidence={confidence}".format(
                        seq=seq,
                        target_id=target_id,
                        row=target["row"],
                        col=target["col"],
                        stable_count=stable_count,
                        method=payload.get("method"),
                        confidence=payload.get("confidence"),
                    ),
                    flush=True,
                )
                seq += 1
        except KeyboardInterrupt:
            print("\nStopping vision bridge...")
        finally:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)

    return process.returncode or 0


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    return stream_vision_targets(args)


if __name__ == "__main__":
    raise SystemExit(main())
