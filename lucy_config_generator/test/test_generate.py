# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Golden tests for lucy_config_generator (#96)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from lucy_config_generator.generate import generate_from_xacro_string_for_tests
from lucy_config_generator.schema import (
    derive_ros2_hardware_name,
    derive_ros2_node_name,
    validate_hardware_yaml,
)

_FIXTURES = Path(__file__).resolve().parent / "fixtures"


def _fixture_urdf_xml() -> str:
    """Return mock robot XML from test_robot.urdf.xacro for joint-name extraction."""
    return (_FIXTURES / "test_robot.urdf.xacro").read_text(encoding="utf-8")


def _load_mapping() -> dict:
    with (_FIXTURES / "test_mapping.yaml").open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_schema_accepts_fixture():
    validate_hardware_yaml(_load_mapping())


def test_ros2_names_derived_from_board_id():
    assert derive_ros2_hardware_name("rp2040_left_arm") == "LucyHardwareLeftArm"
    assert derive_ros2_node_name("rp2040_left_arm") == "lucy_hardware_interface_left_arm"
    assert derive_ros2_hardware_name("rp2040_torso_head") == "LucyHardwareTorsoHead"
    assert derive_ros2_node_name("rp2040_torso_head") == "lucy_hardware_interface_torso_head"


def test_schema_rejects_bad_version():
    data = _load_mapping()
    data["version"] = 2
    with pytest.raises(ValueError, match="version"):
        validate_hardware_yaml(data)


def test_golden_firmware_left_arm():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {"firmware"}, None)[
        "config_rp2040_left_arm.c"
    ]
    expected = (_FIXTURES / "golden_config_rp2040_left_arm.c").read_text(encoding="utf-8")
    assert got == expected


def test_golden_firmware_right_arm():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {"firmware"}, None)[
        "config_rp2040_right_arm.c"
    ]
    expected = (_FIXTURES / "golden_config_rp2040_right_arm.c").read_text(encoding="utf-8")
    assert got == expected


def test_golden_firmware_torso():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {"firmware"}, None)[
        "config_rp2040_torso_head.c"
    ]
    expected = (_FIXTURES / "golden_config_rp2040_torso_head.c").read_text(encoding="utf-8")
    assert got == expected


def test_golden_ros2_control():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {"ros2_control"}, None)[
        "inmoov_ros2_control.xacro"
    ]
    expected = (_FIXTURES / "golden_inmoov_ros2_control.xacro").read_text(encoding="utf-8")
    assert got == expected


def test_golden_controllers_extra_joints():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {"controllers"}, None)[
        "controllers.yaml"
    ]
    expected = (_FIXTURES / "golden_controllers.yaml").read_text(encoding="utf-8")
    assert got == expected


def test_boards_filter_emits_subset():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(
        data,
        _fixture_urdf_xml(),
        {"firmware"},
        {"rp2040_left_arm"},
    )
    assert set(got.keys()) == {"config_rp2040_left_arm.c"}
