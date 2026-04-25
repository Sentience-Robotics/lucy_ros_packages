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


def test_schema_rejects_empty_compile_definition():
    data = _load_mapping()
    data["boards"]["rp2040_left_arm"]["compile_definition"] = ""
    with pytest.raises(ValueError, match="compile_definition"):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_controller_name():
    data = _load_mapping()
    data["boards"]["rp2040_left_arm"]["controller"]["name"] = ""
    with pytest.raises(ValueError, match=r"controller\.name"):
        validate_hardware_yaml(data)


def test_schema_rejects_invalid_internal_servo_slots():
    data = _load_mapping()
    data["boards"]["rp2040_left_arm"]["internal_servo_slots"] = 0
    with pytest.raises(ValueError, match="internal_servo_slots must be >= 1"):
        validate_hardware_yaml(data)


def test_schema_rejects_null_virtual_pin():
    data = _load_mapping()
    data["actuators"][0]["virtual_pin"] = None
    with pytest.raises(ValueError, match="virtual_pin must be an integer"):
        validate_hardware_yaml(data)


def test_schema_rejects_servo_min_greater_than_max():
    data = _load_mapping()
    data["actuators"][0]["servo_min_deg"] = 10
    data["actuators"][0]["servo_max_deg"] = 5
    data["actuators"][0]["servo_default_deg"] = 7
    with pytest.raises(ValueError, match="servo_min_deg .* must be <= servo_max_deg"):
        validate_hardware_yaml(data)


def test_schema_rejects_physical_pin_above_board_slot_limit():
    data = _load_mapping()
    data["boards"]["rp2040_left_arm"]["internal_servo_slots"] = 8
    data["actuators"][0]["physical_pin"] = 9
    with pytest.raises(ValueError, match=r"physical_pin 9 out of range 1..8"):
        validate_hardware_yaml(data)


def test_schema_rejects_non_bool_enabled_actuator():
    data = _load_mapping()
    data["actuators"][0]["enabled"] = "fal"
    with pytest.raises(ValueError, match="enabled must be a boolean"):
        validate_hardware_yaml(data)


def test_schema_rejects_non_bool_enabled_sensor():
    data = _load_mapping()
    data["sensors"][0]["enabled"] = "fal"
    with pytest.raises(ValueError, match="enabled must be a boolean"):
        validate_hardware_yaml(data)


def test_schema_rejects_non_numeric_offset_deg():
    data = _load_mapping()
    data["actuators"][0]["offset_deg"] = None
    with pytest.raises(ValueError, match="offset_deg must be numeric"):
        validate_hardware_yaml(data)


def test_schema_rejects_non_unit_direction():
    data = _load_mapping()
    data["actuators"][0]["direction"] = 0
    with pytest.raises(ValueError, match="direction must be -1 or 1"):
        validate_hardware_yaml(data)


def test_schema_rejects_zero_scale():
    data = _load_mapping()
    data["actuators"][0]["scale"] = 0
    with pytest.raises(ValueError, match="scale must be non-zero"):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_sensor_id():
    data = _load_mapping()
    data["sensors"][0]["id"] = ""
    with pytest.raises(ValueError, match=r"sensor .*: id must be a non-empty string"):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_sensor_type():
    data = _load_mapping()
    data["sensors"][0]["type"] = ""
    with pytest.raises(ValueError, match=r"sensor .*: type must be a non-empty string"):
        validate_hardware_yaml(data)


def test_schema_rejects_sensor_min_value_greater_than_max_value():
    data = _load_mapping()
    data["sensors"][0]["min_value"] = 2000
    data["sensors"][0]["max_value"] = 0
    with pytest.raises(ValueError, match=r"sensor .*: min_value .* must be <= max_value"):
        validate_hardware_yaml(data)


def test_schema_reports_missing_virtual_pin_when_gap_exists():
    data = _load_mapping()
    data["actuators"][2]["virtual_pin"] = 3
    with pytest.raises(ValueError, match=r"board .*: missing virtual_pin indices \[[0-9, ]+\]"):
        validate_hardware_yaml(data)


def test_schema_suppresses_sensor_contiguity_when_sensor_has_item_error():
    data = _load_mapping()
    data["sensors"][0]["associated_actuator"] = "does_not_exist"
    with pytest.raises(ValueError) as exc:
        validate_hardware_yaml(data)
    msg = str(exc.value)
    assert "associated_actuator does_not_exist not found" in msg
    assert "sensor virtual_pin must be contiguous" not in msg
    assert "missing sensor virtual_pin indices" not in msg


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
