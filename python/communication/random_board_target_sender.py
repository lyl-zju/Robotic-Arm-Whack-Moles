from __future__ import annotations

import argparse
import random
import socket
import time
from typing import Iterable

from .board_protocol import BoardConfig, build_payload


def choose_target_id(rng: random.Random, previous_id: int | None, allow_repeat: bool) -> int:
    target_id = rng.randint(1, 9)
    if allow_repeat or previous_id is None:
        return target_id

    while target_id == previous_id:
        target_id = rng.randint(1, 9)
    return target_id


def stream_random_targets(
    host: str,
    port: int,
    period_s: float,
    count: int,
    seed: int | None,
    allow_repeat: bool,
    start_delay_s: float,
) -> None:
    if period_s <= 0:
        raise ValueError("period_s must be positive.")
    if count < 0:
        raise ValueError("count must be non-negative.")
    if start_delay_s < 0:
        raise ValueError("start_delay_s must be non-negative.")

    rng = random.Random(seed)
    board = BoardConfig()
    address = (host, port)
    seq = 0
    previous_id: int | None = None

    if start_delay_s > 0:
        print(f"Waiting {start_delay_s:.1f} s before sending the first target...")
        time.sleep(start_delay_s)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        print(f"Streaming random board targets to {host}:{port} every {period_s:.2f} s.")
        print("Press Ctrl+C to stop.")

        while count == 0 or seq < count:
            target_id = choose_target_id(rng, previous_id, allow_repeat)
            previous_id = target_id
            target = board.target(target_id)
            udp_socket.sendto(build_payload(seq, target, "random_board_demo"), address)
            print(
                "seq={seq:06d} id={target_id} row={row} col={col} "
                "board=({board_x:+.2f},{board_y:+.2f})".format(
                    seq=seq,
                    target_id=target_id,
                    row=target["row"],
                    col=target["col"],
                    board_x=target["board"][0],
                    board_y=target["board"][1],
                ),
                flush=True,
            )

            seq += 1
            if count == 0 or seq < count:
                time.sleep(period_s)


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return parsed


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send random 3x3 board targets to MATLAB over UDP."
    )
    parser.add_argument("--host", default="127.0.0.1", help="MATLAB receiver host.")
    parser.add_argument("--port", type=int, default=5005, help="MATLAB UDP local port.")
    parser.add_argument(
        "--period-s",
        type=float,
        default=2.5,
        help="Seconds between target messages. Keep this longer than one hit cycle.",
    )
    parser.add_argument(
        "--count",
        type=positive_int,
        default=0,
        help="Number of targets to send. Use 0 for infinite streaming.",
    )
    parser.add_argument("--seed", type=int, default=None, help="Random seed.")
    parser.add_argument(
        "--allow-repeat",
        action="store_true",
        help="Allow the same target id to be sent twice in a row.",
    )
    parser.add_argument(
        "--start-delay-s",
        type=float,
        default=1.0,
        help="Delay before the first UDP packet.",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)
    try:
        stream_random_targets(
            host=args.host,
            port=args.port,
            period_s=args.period_s,
            count=args.count,
            seed=args.seed,
            allow_repeat=args.allow_repeat,
            start_delay_s=args.start_delay_s,
        )
    except KeyboardInterrupt:
        print("\nSender stopped.")


if __name__ == "__main__":
    main()
