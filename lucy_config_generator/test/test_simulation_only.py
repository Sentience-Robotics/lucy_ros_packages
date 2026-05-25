# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Simulation-only generation: single ros2_control block and lucy_sim_controller."""

from __future__ import annotations

from pathlib import Path

import yaml

from lucy_config_generator.generate import generate_from_xacro_string_for_tests

_FIXTURES = Path(__file__).resolve().parent / "fixtures"


def _load_mapping() -> dict:
    with (_FIXTURES / "test_mapping.yaml").open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _fixture_urdf_xml() -> str:
    return (_FIXTURES / "test_robot.urdf.xacro").read_text(encoding="utf-8")


def test_simulation_only_single_ros2_control_and_jtc():
    data = _load_mapping()
    out = generate_from_xacro_string_for_tests(
        data,
        _fixture_urdf_xml(),
        targets={"ros2_control", "controllers"},
        simulation_only=True,
    )
    assert not any(k.startswith("config_") for k in out)

    xacro = out["inmoov_ros2_control.xacro"]
    assert xacro.count('<ros2_control name="') == 1
    assert 'name="LucyHardwareSim"' in xacro
    assert "mock_components/GenericSystem" in xacro

    actuator_joints = sorted(a["urdf_joint"] for a in data["actuators"])
    for joint in actuator_joints:
        assert f'<joint name="{joint}"' in xacro

    ctrl_yaml = yaml.safe_load(out["controllers.yaml"])
    cm = ctrl_yaml["controller_manager"]["ros__parameters"]
    assert "lucy_sim_controller" in cm
    assert cm["lucy_sim_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert "left_arm_controller" not in cm

    sim_joints = ctrl_yaml["lucy_sim_controller"]["ros__parameters"]["joints"]
    assert sorted(sim_joints) == actuator_joints
