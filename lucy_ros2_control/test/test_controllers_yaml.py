# Copyright 2025 Sentience Robotics Team
# SPDX-License-Identifier: GPL-3.0
"""Validate lucy_controllers.yaml loads and contains expected ros2_control configuration."""
from pathlib import Path

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def _controllers_path() -> Path:
    if get_package_share_directory is not None:
        try:
            share = Path(get_package_share_directory('lucy_ros2_control'))
            p = share / 'config' / 'lucy_controllers.yaml'
            if p.is_file():
                return p
        except Exception:
            pass
    # Source checkout: lucy_ros2_control/config/...
    return Path(__file__).resolve().parents[1] / 'config' / 'lucy_controllers.yaml'


def test_controllers_yaml_exists():
    path = _controllers_path()
    assert path.is_file(), f'missing {path}'


def test_controllers_yaml_structure():
    path = _controllers_path()
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict), 'root must be a mapping'
    assert 'controller_manager' in data
    cm = data['controller_manager']['ros__parameters']
    assert 'update_rate' in cm
    for name in ('joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller'):
        assert name in cm, f'controller_manager must declare {name}'
    assert 'joint_state_broadcaster' in data
    extra = data['joint_state_broadcaster']['ros__parameters'].get('extra_joints', [])
    assert isinstance(extra, list)
    for name in ('left_arm_controller', 'right_arm_controller'):
        assert name in data
        joints = data[name]['ros__parameters']['joints']
        assert isinstance(joints, list) and len(joints) > 0
