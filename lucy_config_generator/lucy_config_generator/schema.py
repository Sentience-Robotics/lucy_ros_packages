# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""YAML hardware mapping validation."""

from __future__ import annotations

import re
from typing import Any

REQUIRED_ROOT = (
    "version",
    "robot_name",
    "firmware",
    "controller_manager",
    "boards",
    "actuators",
    "sensors",
)
REQUIRED_ACTUATOR = (
    "id",
    "urdf_joint",
    "board",
    "virtual_pin",
    "physical_pin",
    "servo_type",
    "offset_deg",
    "direction",
    "scale",
    "servo_min_deg",
    "servo_max_deg",
    "servo_default_deg",
    "enabled",
)
REQUIRED_SENSOR = (
    "id",
    "type",
    "associated_actuator",
    "board",
    "virtual_pin",
    "physical_pin",
    "min_value",
    "max_value",
    "enabled",
)
REQUIRED_BOARD = (
    "serial_id",
    "board_class",
    "firmware_target",
    "compile_definition",
    "topic_actuators",
    "topic_sensors",
    "controller",
)

# Firmware C template: single internal PWM stack vs internal + I2C (PCA) stack.
BOARD_CLASS_INTERNAL_ONLY = "internal_servo_only"
BOARD_CLASS_INTERNAL_I2C_PWM = "internal_servo_i2c_pwm"
BOARD_CLASSES = frozenset({BOARD_CLASS_INTERNAL_ONLY, BOARD_CLASS_INTERNAL_I2C_PWM})

_BOARD_ID_RE = re.compile(r"^rp2040_[a-z][a-z0-9_]*$")
_TOPIC_RE = re.compile(r"^[a-z][a-z0-9_/]*$")
_SERIAL_ID_RE = re.compile(r"^[A-Za-z0-9]*$")


def ros2_hardware_suffix(board_id: str) -> str:
    """Return the snake_case segment after the ``rp2040_`` board id prefix."""
    if not _BOARD_ID_RE.fullmatch(board_id):
        raise ValueError(
            f"board id {board_id!r} must match {_BOARD_ID_RE.pattern} "
            "(rp2040_ prefix + snake_case suffix)"
        )
    return board_id[7:]  # len("rp2040_") == 7


def derive_ros2_hardware_name(board_id: str) -> str:
    """
    Return ``<ros2_control name="…">`` for *board_id*.

    Pattern: ``LucyHardware`` + PascalCase of the segment after ``rp2040_``.
    Example: ``rp2040_left_arm`` → ``LucyHardwareLeftArm``.
    """
    suffix = ros2_hardware_suffix(board_id)
    parts = [p for p in suffix.split("_") if p]
    if not parts:
        raise ValueError(f"board id {board_id!r}: empty suffix after rp2040_")
    inner = "".join(p[:1].upper() + p[1:].lower() if p else "" for p in parts)
    return "LucyHardware" + inner


def derive_ros2_node_name(board_id: str) -> str:
    """
    Return the ``node_name`` hardware parameter for *board_id*.

    Pattern: ``lucy_hardware_interface_`` + snake_case suffix after ``rp2040_``.
    """
    return "lucy_hardware_interface_" + ros2_hardware_suffix(board_id)


def validate_hardware_yaml(data: dict[str, Any]) -> None:
    """Raise ValueError if the mapping is structurally invalid."""
    for key in REQUIRED_ROOT:
        if key not in data:
            raise ValueError(f"missing root key: {key}")
    if data["version"] != 1:
        raise ValueError("version must be 1")
    boards: dict[str, Any] = data["boards"]
    if not isinstance(boards, dict) or not boards:
        raise ValueError("boards must be a non-empty map")

    for bid, bdef in boards.items():
        for k in REQUIRED_BOARD:
            if k not in bdef:
                raise ValueError(f"board {bid}: missing {k}")
        ctrl = bdef["controller"]
        if "name" not in ctrl or "type" not in ctrl:
            raise ValueError(f"board {bid}: controller needs name and type")

        bc = bdef["board_class"]
        if bc not in BOARD_CLASSES:
            raise ValueError(
                f"board {bid}: board_class must be one of {sorted(BOARD_CLASSES)}, got {bc!r}"
            )

        sid = bdef["serial_id"]
        if not isinstance(sid, str):
            raise ValueError(f"board {bid}: serial_id must be a string")
        if sid and not _SERIAL_ID_RE.fullmatch(sid):
            raise ValueError(
                f"board {bid}: serial_id must be empty or alphanumeric "
                "(USB serial / picotool --ser)"
            )

        for topic_key in ("topic_actuators", "topic_sensors"):
            t = bdef[topic_key]
            if not isinstance(t, str) or not t:
                raise ValueError(f"board {bid}: {topic_key} must be a non-empty string")
            if not _TOPIC_RE.fullmatch(t):
                raise ValueError(
                    f"board {bid}: {topic_key} must match {_TOPIC_RE.pattern} (no leading slash)"
                )

        try:
            derive_ros2_hardware_name(bid)
            derive_ros2_node_name(bid)
        except ValueError as e:
            raise ValueError(f"board {bid}: {e}") from e
        if bdef["topic_actuators"] != bdef["topic_actuators"].strip():
            raise ValueError(f"board {bid}: topic_actuators must not have surrounding whitespace")

    actuators: list[dict[str, Any]] = data["actuators"]
    if not isinstance(actuators, list):
        raise ValueError("actuators must be a list")
    by_board: dict[str, list[dict[str, Any]]] = {}
    for a in actuators:
        for k in REQUIRED_ACTUATOR:
            if k not in a:
                raise ValueError(f"actuator {a.get('id')}: missing {k}")
        b = a["board"]
        if b not in boards:
            raise ValueError(f"actuator {a['id']}: unknown board {b}")
        pin = int(a["physical_pin"])
        if pin < 1 or pin > 18:
            raise ValueError(
                f"actuator {a['id']}: physical_pin {pin} out of range 1..18 "
                "(emitted as INTERNAL_SERVO_<physical_pin> in firmware C)"
            )
        st = str(a["servo_type"]).strip('"')
        if st not in ("180", "270", "300"):
            raise ValueError(f"actuator {a['id']}: invalid servo_type {st!r}")
        by_board.setdefault(b, []).append(a)

    for board_id, lst in by_board.items():
        vpins = sorted(int(x["virtual_pin"]) for x in lst)
        if len(vpins) != len(set(vpins)):
            raise ValueError(f"board {board_id}: duplicate virtual_pin")
        if vpins != list(range(len(vpins))):
            raise ValueError(
                f"board {board_id}: virtual_pin must be contiguous from 0..N-1, got {vpins}"
            )
        for a in lst:
            lo = a["servo_min_deg"]
            hi = a["servo_max_deg"]
            d = a["servo_default_deg"]
            if lo is not None and hi is not None and d is not None:
                if float(d) < float(lo) or float(d) > float(hi):
                    raise ValueError(
                        f"actuator {a['id']}: servo_default_deg out of [{lo}, {hi}]"
                    )

    sensors: list[dict[str, Any]] = data["sensors"]
    if not isinstance(sensors, list):
        raise ValueError("sensors must be a list")
    actuator_ids = {a["id"] for a in actuators}
    sensors_by_board: dict[str, list[dict[str, Any]]] = {}
    for s in sensors:
        for k in REQUIRED_SENSOR:
            if k not in s:
                raise ValueError(f"sensor {s.get('id')}: missing {k}")
        if s["board"] not in boards:
            raise ValueError(f"sensor {s['id']}: unknown board {s['board']}")
        if s["associated_actuator"] not in actuator_ids:
            raise ValueError(
                f"sensor {s['id']}: associated_actuator {s['associated_actuator']} not found"
            )
        sensors_by_board.setdefault(s["board"], []).append(s)

    for board_id, lst in sensors_by_board.items():
        vpins = sorted(int(x["virtual_pin"]) for x in lst)
        if lst and (len(vpins) != len(set(vpins))):
            raise ValueError(f"board {board_id}: duplicate sensor virtual_pin")
        if lst and vpins != list(range(len(vpins))):
            raise ValueError(
                f"board {board_id}: sensor virtual_pin must be contiguous 0..N-1, got {vpins}"
            )
