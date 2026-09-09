# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Unit tests for Rust firmware YAML generation."""

from __future__ import annotations

from lucy_config_generator.generate import generate_from_xacro_string_for_tests
from lucy_config_generator.generate import render_firmware_yaml


MINIMAL = {
    'version': 1,
    'robot_name': 'test',
    'firmware': {'source_dir': 'lucy_embedded_firmware', 'build_dir': 'build/firmware'},
    'controller_manager': {'update_rate': 100},
    'boards': {
        'rp2040_left_arm': {
            'serial_id': 'ABC',
            'board_class': 'internal_servo_only',
            'internal_servo_slots': 8,
            'firmware_target': 'lucy_left_arm',
            'compile_definition': 'USE_LEFT',
            'topic_actuators': 'actuators/left_arm',
            'topic_sensors': 'sensors/left_arm',
            'controller': {
                'name': 'left_arm_controller',
                'type': 'joint_trajectory_controller/JointTrajectoryController',
            },
        },
        'rp2040_torso_head': {
            'serial_id': '',
            'board_class': 'internal_servo_i2c_pwm',
            'internal_servo_slots': 8,
            'firmware_target': 'lucy_torso',
            'compile_definition': 'USE_TORSO',
            'topic_actuators': 'actuators/torso',
            'topic_sensors': 'sensors/torso',
            'controller': {
                'name': 'torso_controller',
                'type': 'joint_trajectory_controller/JointTrajectoryController',
            },
        },
    },
    'actuators': [
        {
            'id': 'left_elbow',
            'urdf_joint': 'left_elbow_joint',
            'board': 'rp2040_left_arm',
            'virtual_pin': 0,
            'physical_pin': 10,
            'servo_type': '270',
            'offset_deg': 0,
            'direction': 1,
            'scale': 1,
            'servo_min_deg': 0,
            'servo_max_deg': 270,
            'servo_default_deg': 135,
            'enabled': True,
        },
        {
            'id': 'head_jaw',
            'urdf_joint': 'jaw_joint',
            'board': 'rp2040_torso_head',
            'virtual_pin': 1,
            'physical_pin': 12,
            'servo_type': '180',
            'offset_deg': 0,
            'direction': -1,
            'scale': 1,
            'servo_min_deg': 0,
            'servo_max_deg': 180,
            'servo_default_deg': 90,
            'enabled': True,
        },
        {
            'id': 'disabled_joint',
            'urdf_joint': 'disabled_joint',
            'board': 'rp2040_left_arm',
            'virtual_pin': 2,
            'physical_pin': 11,
            'servo_type': '300',
            'offset_deg': 0,
            'direction': 1,
            'scale': 0.25,
            'servo_min_deg': 0,
            'servo_max_deg': 300,
            'servo_default_deg': 150,
            'enabled': False,
        },
    ],
    'sensors': [],
}


def test_render_firmware_yaml_servo_types_and_enabled_filter():
    text = render_firmware_yaml(MINIMAL, 'rp2040_left_arm')
    assert 'board_id: rp2040_left_arm' in text
    assert 'left_elbow' in text
    assert 'max_angle: 270' in text
    assert 'disabled_joint' not in text
    assert 'PwmServoConfig' in text
    assert 'PwmServoModbusAdapter' in text


def test_generate_emits_yaml_not_c():
    urdf = """<?xml version="1.0"?>
<robot name="t">
  <joint name="left_elbow_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="jaw_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="disabled_joint" type="revolute">
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    out = generate_from_xacro_string_for_tests(
        MINIMAL, urdf, targets={'firmware'}, boards_filter={'rp2040_left_arm'}
    )
    assert 'config_rp2040_left_arm.yaml' in out
    assert 'config_rp2040_left_arm.c' not in out
