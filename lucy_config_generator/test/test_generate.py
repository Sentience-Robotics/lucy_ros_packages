# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Golden tests for lucy_config_generator (#96)."""

from __future__ import annotations

from pathlib import Path

from lucy_config_generator.generate import generate_from_xacro_string_for_tests
from lucy_config_generator.schema import derive_ros2_hardware_name
from lucy_config_generator.schema import derive_ros2_node_name
from lucy_config_generator.schema import GENERATED_FILES_DEFAULTS
from lucy_config_generator.schema import resolve_generated_files
from lucy_config_generator.schema import validate_hardware_yaml

import pytest
import yaml

_FIXTURES = Path(__file__).resolve().parent / 'fixtures'


def _fixture_urdf_xml() -> str:
    """Return mock robot XML from test_robot.urdf.xacro for joint-name extraction."""
    return (_FIXTURES / 'test_robot.urdf.xacro').read_text(encoding='utf-8')


def _load_mapping() -> dict:
    with (_FIXTURES / 'test_mapping.yaml').open(encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_schema_accepts_fixture():
    validate_hardware_yaml(_load_mapping())


def test_ros2_names_derived_from_board_id():
    assert derive_ros2_hardware_name('rp2040_left_arm') == 'LucyHardwareLeftArm'
    assert derive_ros2_node_name('rp2040_left_arm') == 'lucy_hardware_interface_left_arm'
    assert derive_ros2_hardware_name('rp2040_torso_head') == 'LucyHardwareTorsoHead'
    assert derive_ros2_node_name('rp2040_torso_head') == 'lucy_hardware_interface_torso_head'


def test_schema_rejects_bad_version():
    data = _load_mapping()
    data['version'] = 2
    with pytest.raises(ValueError, match='version'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_compile_definition():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['compile_definition'] = ''
    with pytest.raises(ValueError, match='compile_definition'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_controller_name():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['controller']['name'] = ''
    with pytest.raises(ValueError, match=r'controller\.name'):
        validate_hardware_yaml(data)


def test_schema_rejects_invalid_internal_servo_slots():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['internal_servo_slots'] = 0
    with pytest.raises(ValueError, match='internal_servo_slots must be >= 1'):
        validate_hardware_yaml(data)


def test_schema_rejects_null_virtual_pin():
    data = _load_mapping()
    data['actuators'][0]['virtual_pin'] = None
    with pytest.raises(ValueError, match='virtual_pin must be an integer'):
        validate_hardware_yaml(data)


def test_schema_rejects_servo_min_greater_than_max():
    data = _load_mapping()
    data['actuators'][0]['servo_min_deg'] = 10
    data['actuators'][0]['servo_max_deg'] = 5
    data['actuators'][0]['servo_default_deg'] = 7
    with pytest.raises(ValueError, match='servo_min_deg .* must be <= servo_max_deg'):
        validate_hardware_yaml(data)


def test_schema_rejects_physical_pin_above_board_slot_limit():
    data = _load_mapping()
    data['boards']['rp2040_left_arm']['internal_servo_slots'] = 8
    data['actuators'][0]['physical_pin'] = 9
    with pytest.raises(ValueError, match=r'physical_pin 9 out of range 1..8'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_bool_enabled_actuator():
    data = _load_mapping()
    data['actuators'][0]['enabled'] = 'fal'
    with pytest.raises(ValueError, match='enabled must be a boolean'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_bool_enabled_sensor():
    data = _load_mapping()
    data['sensors'][0]['enabled'] = 'fal'
    with pytest.raises(ValueError, match='enabled must be a boolean'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_numeric_offset_deg():
    data = _load_mapping()
    data['actuators'][0]['offset_deg'] = None
    with pytest.raises(ValueError, match='offset_deg must be numeric'):
        validate_hardware_yaml(data)


def test_schema_rejects_non_unit_direction():
    data = _load_mapping()
    data['actuators'][0]['direction'] = 0
    with pytest.raises(ValueError, match='direction must be -1 or 1'):
        validate_hardware_yaml(data)


def test_schema_rejects_zero_scale():
    data = _load_mapping()
    data['actuators'][0]['scale'] = 0
    with pytest.raises(ValueError, match='scale must be non-zero'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_sensor_id():
    data = _load_mapping()
    data['sensors'][0]['id'] = ''
    with pytest.raises(ValueError, match=r'sensor .*: id must be a non-empty string'):
        validate_hardware_yaml(data)


def test_schema_rejects_empty_sensor_type():
    data = _load_mapping()
    data['sensors'][0]['type'] = ''
    with pytest.raises(ValueError, match=r'sensor .*: type must be a non-empty string'):
        validate_hardware_yaml(data)


def test_schema_rejects_sensor_min_value_greater_than_max_value():
    data = _load_mapping()
    data['sensors'][0]['min_value'] = 2000
    data['sensors'][0]['max_value'] = 0
    with pytest.raises(ValueError, match=r'sensor .*: min_value .* must be <= max_value'):
        validate_hardware_yaml(data)


def test_schema_reports_missing_virtual_pin_when_gap_exists():
    data = _load_mapping()
    data['actuators'][2]['virtual_pin'] = 3
    with pytest.raises(ValueError, match=r'board .*: missing virtual_pin indices \[[0-9, ]+\]'):
        validate_hardware_yaml(data)


def test_schema_suppresses_sensor_contiguity_when_sensor_has_item_error():
    data = _load_mapping()
    data['sensors'][0]['associated_actuator'] = 'does_not_exist'
    with pytest.raises(ValueError) as exc:
        validate_hardware_yaml(data)
    msg = str(exc.value)
    assert 'associated_actuator does_not_exist not found' in msg
    assert 'sensor virtual_pin must be contiguous' not in msg
    assert 'missing sensor virtual_pin indices' not in msg


def test_golden_firmware_left_arm():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'firmware'}, None)[
        'config_rp2040_left_arm.c'
    ]
    expected = (_FIXTURES / 'golden_config_rp2040_left_arm.c').read_text(encoding='utf-8')
    assert got == expected


def test_golden_firmware_right_arm():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'firmware'}, None)[
        'config_rp2040_right_arm.c'
    ]
    expected = (_FIXTURES / 'golden_config_rp2040_right_arm.c').read_text(encoding='utf-8')
    assert got == expected


def test_golden_firmware_torso():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'firmware'}, None)[
        'config_rp2040_torso_head.c'
    ]
    expected = (_FIXTURES / 'golden_config_rp2040_torso_head.c').read_text(encoding='utf-8')
    assert got == expected


def test_golden_ros2_control():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'ros2_control'}, None)[
        GENERATED_FILES_DEFAULTS['ros2_control_xacro']
    ]
    expected = (_FIXTURES / 'golden_inmoov_ros2_control.xacro').read_text(encoding='utf-8')
    assert got == expected


def test_ros2_control_emits_urdf_limits_on_command_interface():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'ros2_control'}, None)[
        GENERATED_FILES_DEFAULTS['ros2_control_xacro']
    ]
    assert 'name="left_shoulder_y_link_joint"' in got
    assert '<param name="min">0.0</param>' in got
    assert '<param name="max">1.0</param>' in got
    # Every actuated revolute in the fixture has limits 0..1 rad.
    assert got.count('<param name="min">') == 5
    assert got.count('<param name="max">') == 5


def test_golden_controllers_extra_joints():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(data, _fixture_urdf_xml(), {'controllers'}, None)[
        GENERATED_FILES_DEFAULTS['controllers_yaml']
    ]
    expected = (_FIXTURES / 'golden_controllers.yaml').read_text(encoding='utf-8')
    assert got == expected


def test_boards_filter_emits_subset():
    data = _load_mapping()
    got = generate_from_xacro_string_for_tests(
        data,
        _fixture_urdf_xml(),
        {'firmware'},
        {'rp2040_left_arm'},
    )
    assert set(got.keys()) == {'config_rp2040_left_arm.c'}


def test_generated_files_defaults_when_section_absent():
    assert resolve_generated_files(_load_mapping()) == GENERATED_FILES_DEFAULTS


def test_generated_files_override_basenames():
    data = _load_mapping()
    data['generated_files'] = {
        'ros2_control_xacro': 'thais_ros2_control.xacro',
        'controllers_yaml': 'thais_controllers.yaml',
    }
    assert resolve_generated_files(data) == {
        'ros2_control_xacro': 'thais_ros2_control.xacro',
        'controllers_yaml': 'thais_controllers.yaml',
    }


def test_generated_files_rejects_path_separator():
    data = _load_mapping()
    data['generated_files'] = {'controllers_yaml': 'config/controllers.yaml'}
    with pytest.raises(ValueError, match='bare filename'):
        validate_hardware_yaml(data)


def test_generated_files_basenames_drive_output_keys():
    data = _load_mapping()
    data['generated_files'] = {
        'ros2_control_xacro': 'thais_ros2_control.xacro',
        'controllers_yaml': 'thais_controllers.yaml',
    }
    got = generate_from_xacro_string_for_tests(
        data, _fixture_urdf_xml(), {'ros2_control', 'controllers'}, None
    )
    assert 'thais_ros2_control.xacro' in got
    assert 'thais_controllers.yaml' in got
    assert GENERATED_FILES_DEFAULTS['ros2_control_xacro'] not in got
