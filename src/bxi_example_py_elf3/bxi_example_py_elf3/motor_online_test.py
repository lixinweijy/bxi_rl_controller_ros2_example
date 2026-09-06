"""Probe ELF3 motors directly through Linux SocketCAN."""

from __future__ import annotations

import argparse
import select
import socket
import struct
import sys
import time
from collections.abc import Sequence


BUS_JOINTS = (
    (
        "l_hip_y_joint",
        "l_hip_x_joint",
        "l_hip_z_joint",
        "l_knee_y_joint",
        "l_ankle_a_joint",
        "l_ankle_b_joint",
        "waist_a_joint",
        "waist_b_joint",
    ),
    (
        "r_hip_y_joint",
        "r_hip_x_joint",
        "r_hip_z_joint",
        "r_knee_y_joint",
        "r_ankle_a_joint",
        "r_ankle_b_joint",
        "waist_z_joint",
    ),
    (
        "l_shoulder_y_joint",
        "l_shoulder_x_joint",
        "l_shoulder_z_joint",
        "l_elbow_y_joint",
        "l_wrist_x_joint",
        "l_wrist_y_joint",
        "l_wrist_z_joint",
        "head_z_joint",
    ),
    (
        "r_shoulder_y_joint",
        "r_shoulder_x_joint",
        "r_shoulder_z_joint",
        "r_elbow_y_joint",
        "r_wrist_x_joint",
        "r_wrist_y_joint",
        "r_wrist_z_joint",
        "head_y_joint",
    ),
)
BUS_OFFSETS = (0, 8, 15, 23)

_EXIT_MOTOR_MODE = b"\xff" * 7 + b"\xfd"
_CANFD_FRAME = struct.Struct("=IBBBB64s")
_CAN_FRAME = struct.Struct("=IB3x8s")
_CANFD_BRS = 0x01
_CANFD_FDF = 0x04
_CAN_RTR_FLAG = 0x40000000
_CAN_ERR_FLAG = 0x20000000


def _probe_frame(bus: int) -> bytes:
    payload = bytearray(64)
    for motor_id in range(1, len(BUS_JOINTS[bus]) + 1):
        offset = (motor_id - 1) * len(_EXIT_MOTOR_MODE)
        payload[offset : offset + len(_EXIT_MOTOR_MODE)] = _EXIT_MOTOR_MODE
    return _CANFD_FRAME.pack(
        0x7FF,
        len(payload),
        _CANFD_BRS | _CANFD_FDF,
        0,
        0,
        bytes(payload),
    )


def _response_motor_id(frame: bytes) -> int | None:
    if len(frame) not in (_CAN_FRAME.size, _CANFD_FRAME.size):
        return None
    can_id = struct.unpack_from("=I", frame)[0]
    if can_id & (_CAN_RTR_FLAG | _CAN_ERR_FLAG) or frame[4] < 8:
        return None
    motor_id = can_id & 0x0F
    return motor_id if 1 <= motor_id <= 8 else None


def _open_bus(bus: int) -> socket.socket:
    can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        can_socket.setsockopt(
            socket.SOL_CAN_RAW,
            socket.CAN_RAW_FD_FRAMES,
            1,
        )
        can_socket.bind((f"can{bus}",))
    except Exception:
        can_socket.close()
        raise
    return can_socket


def _probe_bus(bus: int, timeout_ms: int) -> set[int]:
    with _open_bus(bus) as can_socket:
        can_socket.send(_probe_frame(bus))
        online: set[int] = set()
        deadline = time.monotonic() + timeout_ms / 1000.0
        while len(online) < len(BUS_JOINTS[bus]):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select((can_socket,), (), (), remaining)
            if not readable:
                break
            motor_id = _response_motor_id(can_socket.recv(_CANFD_FRAME.size))
            if motor_id is not None and motor_id <= len(BUS_JOINTS[bus]):
                online.add(motor_id)
        return online


def _print_bus(bus: int, online: set[int] | None) -> None:
    print(f"can{bus}")
    for motor_id, joint_name in enumerate(BUS_JOINTS[bus], start=1):
        status = "" if online is None else (
            "ONLINE " if motor_id in online else "OFFLINE"
        )
        print(
            f"  {status} id={motor_id} "
            f"bit={BUS_OFFSETS[bus] + motor_id - 1} {joint_name}"
        )


def _timeout_ms(value: str) -> int:
    timeout = int(value)
    if not 1 <= timeout <= 10_000:
        raise argparse.ArgumentTypeError("timeout_ms must be between 1 and 10000")
    return timeout


def _arguments(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", nargs="?", default="all", choices=(
        "all", "can0", "can1", "can2", "can3"
    ))
    parser.add_argument("timeout_ms", nargs="?", default=100, type=_timeout_ms)
    parser.add_argument("--list", action="store_true", dest="list_only")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _arguments(argv)
    buses = range(4) if args.target == "all" else (int(args.target[-1]),)
    if args.list_only:
        for bus in buses:
            _print_bus(bus, None)
        return 0

    all_online = True
    for bus in buses:
        try:
            online = _probe_bus(bus, args.timeout_ms)
        except OSError as error:
            print(f"can{bus} probe failed: {error}", file=sys.stderr)
            all_online = False
            continue
        _print_bus(bus, online)
        all_online = all_online and len(online) == len(BUS_JOINTS[bus])
    return 0 if all_online else 1


if __name__ == "__main__":
    raise SystemExit(main())
