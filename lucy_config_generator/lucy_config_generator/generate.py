# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Jinja2 rendering for firmware, ros2_control, and controllers."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import jinja2
import yaml

from lucy_config_generator.schema import (
    BOARD_CLASS_INTERNAL_I2C_PWM,
    BOARD_CLASS_INTERNAL_ONLY,
    derive_ros2_hardware_name,
    derive_ros2_node_name,
    validate_hardware_yaml,
)


def _templates_dir() -> Path:
    return Path(__file__).resolve().parent / "templates"


def _jinja_env() -> jinja2.Environment:
    return jinja2.Environment(
        loader=jinja2.FileSystemLoader(str(_templates_dir())),
        autoescape=False,
        trim_blocks=True,
        lstrip_blocks=True,
    )


def load_hardware_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping")
    return data


def _xacro_argv(urdf_xacro: Path, base_path: Path, controller_config: Path) -> list[str]:
    """Prefer ``ros2 run xacro xacro`` so broken distro ``xacro`` console scripts still work."""
    tail = [
        str(urdf_xacro),
        f"base_path:={base_path}",
        f"controller_config:={controller_config}",
        "use_gazebo_sim:=false",
    ]
    if shutil.which("ros2") is not None:
        return ["ros2", "run", "xacro", "xacro", *tail]
    if shutil.which("xacro") is not None:
        return ["xacro", *tail]
    raise RuntimeError(
        "Need 'ros2' (for `ros2 run xacro xacro`) "
        "or `xacro` on PATH to process URDF"
    )


def urdf_joint_names(
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
) -> set[str]:
    """Joint names from processed URDF (same args as thais_urdf xacro smoke)."""
    if not urdf_xacro.is_file():
        raise FileNotFoundError(urdf_xacro)
    if not base_path.is_dir():
        raise FileNotFoundError(base_path)
    if not controller_config.is_file():
        raise FileNotFoundError(controller_config)
    cmd = _xacro_argv(urdf_xacro, base_path, controller_config)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120, check=False)
    if r.returncode != 0:
        raise RuntimeError(f"xacro failed: {r.stderr}")
    root = ET.fromstring(r.stdout)
    names: set[str] = set()
    for j in root.findall("joint"):
        n = j.attrib.get("name")
        if n:
            names.add(n)
    return names


def _board_ids_in_yaml_order(data: dict[str, Any]) -> list[str]:
    """Preserve ``boards:`` key order from YAML (PyYAML + Python 3.7+ dict order)."""
    boards = data["boards"]
    if not isinstance(boards, dict):
        raise ValueError("boards must be a mapping")
    return list(boards.keys())


def _actuators_for_board(
    data: dict[str, Any], board_id: str, *, enabled_only: bool
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for a in data["actuators"]:
        if a["board"] != board_id:
            continue
        if enabled_only and not a.get("enabled", True):
            continue
        out.append(a)
    out.sort(key=lambda x: int(x["virtual_pin"]))
    return out


def _sensors_for_board(data: dict[str, Any], board_id: str) -> list[dict[str, Any]]:
    out = [s for s in data["sensors"] if s["board"] == board_id]
    out.sort(key=lambda x: int(x["virtual_pin"]))
    return out


def _sensors_for_board_firmware(data: dict[str, Any], board_id: str) -> list[dict[str, Any]]:
    """Pressure rows only if their associated actuator is enabled (matches firmware C scope)."""
    enabled_ids = {a["id"] for a in data["actuators"] if a.get("enabled", True)}
    out = [
        s
        for s in data["sensors"]
        if s["board"] == board_id and s["associated_actuator"] in enabled_ids
    ]
    out.sort(key=lambda x: int(x["virtual_pin"]))
    return out


def _ros2_control_blocks(
    data: dict[str, Any], board_ids: list[str]
) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    for bid in board_ids:
        bdef = data["boards"][bid]
        joints = _actuators_for_board(data, bid, enabled_only=False)
        blocks.append(
            {
                "board_id": bid,
                "hardware_name": derive_ros2_hardware_name(bid),
                "publisher_topic": bdef["topic_actuators"],
                "node_name": derive_ros2_node_name(bid),
                "joints": joints,
            }
        )
    return blocks


def _extra_joints(data: dict[str, Any], urdf_joints: set[str]) -> list[str]:
    """
    Joints published at default via broadcaster, not listed on Lucy hardware blocks.

    Every actuator (enabled or not) is exported under ``ros2_control`` and trajectory
    controllers; only **non-actuator** URDF joints (passive / unmapped) need
    ``extra_joints`` so ``joint_state_broadcaster`` can publish them for TF.
    """
    actuated = {a["urdf_joint"] for a in data["actuators"]}
    extra = sorted(urdf_joints - actuated)
    return extra


def _firmware_template_for_board_class(board_class: str) -> str:
    if board_class == BOARD_CLASS_INTERNAL_I2C_PWM:
        return "config_internal_i2c_board.c.j2"
    if board_class == BOARD_CLASS_INTERNAL_ONLY:
        return "config_internal_only_board.c.j2"
    raise ValueError(f"unknown board_class for firmware template: {board_class!r}")


def render_firmware_c(
    data: dict[str, Any],
    board_id: str,
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    actuators = _actuators_for_board(data, board_id, enabled_only=True)
    sensors = _sensors_for_board_firmware(data, board_id)
    board_class = data["boards"][board_id]["board_class"]
    tpl_name = _firmware_template_for_board_class(board_class)
    tpl = env.get_template(tpl_name)
    return tpl.render(
        board_id=board_id,
        actuators=actuators,
        sensors=sensors,
    )


def render_ros2_control_xacro(
    data: dict[str, Any],
    board_ids: list[str],
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    tpl = env.get_template("ros2_control.xacro.j2")
    return tpl.render(blocks=_ros2_control_blocks(data, board_ids))


def render_controllers_yaml(
    data: dict[str, Any],
    board_ids: list[str],
    extra_joints: list[str],
    env: jinja2.Environment | None = None,
) -> str:
    env = env or _jinja_env()
    tpl = env.get_template("controllers.yaml.j2")
    controllers: list[dict[str, Any]] = []
    update_rate = int(data["controller_manager"]["update_rate"])
    for bid in board_ids:
        ctrl = data["boards"][bid]["controller"]
        joints = [a["urdf_joint"] for a in _actuators_for_board(data, bid, enabled_only=False)]
        controllers.append(
            {
                "name": ctrl["name"],
                "type": ctrl["type"],
                "joints": joints,
            }
        )
    return tpl.render(
        update_rate=update_rate,
        controllers=controllers,
        extra_joints=extra_joints,
    )


def _resolve_board_ids(
    data: dict[str, Any], boards_filter: set[str] | None
) -> list[str]:
    board_ids = _board_ids_in_yaml_order(data)
    if boards_filter is not None:
        board_ids = [b for b in board_ids if b in boards_filter]
        if not board_ids:
            raise ValueError("no boards left after --boards filter")
    return board_ids


def generate(
    *,
    input_yaml: Path,
    urdf_xacro: Path,
    base_path: Path,
    controller_config: Path,
    output_dir: Path,
    targets: set[str],
    boards_filter: set[str] | None,
    simulation_only: bool = False,
) -> None:
    """Validate YAML and write selected artifacts into output_dir."""
    data = load_hardware_yaml(input_yaml)
    validate_hardware_yaml(data)

    if simulation_only:
        ros2_control_boards = _board_ids_in_yaml_order(data)
    else:
        ros2_control_boards = _resolve_board_ids(data, boards_filter)

    output_dir.mkdir(parents=True, exist_ok=True)

    extra: list[str] = []
    if targets & {"controllers", "all"}:
        extra = _extra_joints(data, urdf_joint_names(urdf_xacro, base_path, controller_config))

    env = _jinja_env()

    if targets & {"firmware", "all"} and not simulation_only:
        fw_boards = _resolve_board_ids(data, boards_filter)
        for bid in fw_boards:
            text = render_firmware_c(data, bid, env)
            out = output_dir / f"config_{bid}.c"
            out.write_text(text, encoding="utf-8")

    if targets & {"ros2_control", "all"}:
        xacro_out = output_dir / "inmoov_ros2_control.xacro"
        xacro_out.write_text(
            render_ros2_control_xacro(data, ros2_control_boards, env),
            encoding="utf-8",
        )

    if targets & {"controllers", "all"}:
        yaml_out = output_dir / "controllers.yaml"
        yaml_out.write_text(
            render_controllers_yaml(data, ros2_control_boards, extra, env),
            encoding="utf-8",
        )


def generate_from_xacro_string_for_tests(
    data: dict[str, Any],
    urdf_xml: str,
    targets: set[str],
    boards_filter: set[str] | None = None,
    simulation_only: bool = False,
) -> dict[str, str]:
    """In-process generation for unit tests (no xacro binary). Returns name->content."""
    validate_hardware_yaml(data)
    if simulation_only:
        board_ids = _board_ids_in_yaml_order(data)
        firmware_boards = board_ids
    else:
        board_ids = _resolve_board_ids(data, boards_filter)
        firmware_boards = board_ids
    root = ET.fromstring(urdf_xml)
    urdf_joints = {j.attrib["name"] for j in root.findall("joint") if "name" in j.attrib}
    extra = _extra_joints(data, urdf_joints)
    env = _jinja_env()
    out: dict[str, str] = {}
    if targets & {"firmware", "all"} and not simulation_only:
        for bid in firmware_boards:
            out[f"config_{bid}.c"] = render_firmware_c(data, bid, env)
    if targets & {"ros2_control", "all"}:
        out["inmoov_ros2_control.xacro"] = render_ros2_control_xacro(data, board_ids, env)
    if targets & {"controllers", "all"}:
        out["controllers.yaml"] = render_controllers_yaml(data, board_ids, extra, env)
    return out
