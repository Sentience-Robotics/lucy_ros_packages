# Copyright 2025 Sentience Robotics Team
#
# SPDX-License-Identifier: GPL-3.0-only

"""Manage RSP + ros2_control_node + spawners; restart on /lucy_control/restart."""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from lucy_control_supervisor.controllers_spawn import controllers_to_spawn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@dataclass
class _ManagedProc:
    name: str
    popen: subprocess.Popen
    kind: str  # rsp | cm | spawner


@dataclass
class _StackConfig:
    urdf_path: Path
    base_path: Path
    controllers_yaml: Path
    use_gazebo_sim: bool
    use_mock_hardware: bool
    gazebo_only: bool


class ControlSupervisorNode(Node):
    RESTART_SERVICE = "/lucy_control/restart"
    GAZEBO_RUNNING_TOPIC = "/lucy/gazebo_running"
    TERMINATE_TIMEOUT_S = 10.0
    SPAWN_SETTLE_S = 2.0

    def __init__(self) -> None:
        super().__init__("lucy_control_supervisor")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_path", "")
        self.declare_parameter("controllers_yaml", "")
        self.declare_parameter("use_gazebo_sim", False)
        self.declare_parameter("use_mock_hardware", False)
        self.declare_parameter("gazebo_only", False)
        self.declare_parameter("autostart", True)

        self._children: List[_ManagedProc] = []
        self._restart_lock = False
        self._srv = self.create_service(Trigger, self.RESTART_SERVICE, self._on_restart)
        # Latched publisher so any late subscriber (e.g. the LCP after rosbridge reconnects)
        # immediately sees whether Gazebo is part of the current launch.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._gazebo_running_pub = self.create_publisher(
            Bool, self.GAZEBO_RUNNING_TOPIC, latched_qos
        )
        self._publish_gazebo_running()
        if self.get_parameter("autostart").value:
            ok, msg = self._restart_stack()
            if not ok:
                self.get_logger().error(msg)

    def _publish_gazebo_running(self) -> None:
        msg = Bool()
        msg.data = bool(self.get_parameter("use_gazebo_sim").value)
        self._gazebo_running_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._terminate_children()
        return super().destroy_node()

    def _cfg(self) -> _StackConfig:
        return _StackConfig(
            urdf_path=Path(self.get_parameter("urdf_path").value).resolve(),
            base_path=Path(self.get_parameter("base_path").value).resolve(),
            controllers_yaml=Path(self.get_parameter("controllers_yaml").value).resolve(),
            use_gazebo_sim=bool(self.get_parameter("use_gazebo_sim").value),
            use_mock_hardware=bool(self.get_parameter("use_mock_hardware").value),
            gazebo_only=bool(self.get_parameter("gazebo_only").value),
        )

    def _xacro_cmd(self, cfg: _StackConfig) -> list[str]:
        tail = [
            str(cfg.urdf_path),
            f"base_path:={cfg.base_path}",
            f"controller_config:={cfg.controllers_yaml}",
            f"use_gazebo_sim:={'true' if cfg.use_gazebo_sim else 'false'}",
            f"use_mock_hardware:={'true' if cfg.use_mock_hardware else 'false'}",
        ]
        if shutil.which("ros2"):
            return ["ros2", "run", "xacro", "xacro", *tail]
        if shutil.which("xacro"):
            return ["xacro", *tail]
        raise RuntimeError("xacro not found on PATH")

    def _expand_robot_description(self, cfg: _StackConfig) -> str:
        cmd = self._xacro_cmd(cfg)
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120, check=False)
        if r.returncode != 0:
            raise RuntimeError(f"xacro failed: {r.stderr or r.stdout}")
        return r.stdout

    def _terminate_children(self) -> None:
        for child in reversed(self._children):
            if child.popen.poll() is None:
                try:
                    child.popen.send_signal(signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + self.TERMINATE_TIMEOUT_S
        for child in self._children:
            while child.popen.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
        for child in self._children:
            if child.popen.poll() is None:
                try:
                    child.popen.kill()
                except ProcessLookupError:
                    pass
        self._children.clear()

    def _start_rsp(self, cfg: _StackConfig, urdf_xml: str) -> None:
        payload = {
            "robot_state_publisher": {
                "ros__parameters": {
                    "robot_description": urdf_xml,
                    "use_sim_time": bool(cfg.use_gazebo_sim),
                }
            }
        }
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as f:
            yaml.safe_dump(payload, f, sort_keys=False)
            params_file = f.name
        cmd = [
            "ros2",
            "run",
            "robot_state_publisher",
            "robot_state_publisher",
            "--ros-args",
            "--params-file",
            params_file,
        ]
        popen = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
        )
        self._children.append(_ManagedProc("robot_state_publisher", popen, "rsp"))
        time.sleep(self.SPAWN_SETTLE_S)

    def _start_cm(self, cfg: _StackConfig) -> None:
        cmd = [
            "ros2",
            "run",
            "controller_manager",
            "ros2_control_node",
            "--ros-args",
            "--params-file",
            str(cfg.controllers_yaml),
            "-r",
            "~/robot_description:=/robot_description",
        ]
        popen = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
        )
        self._children.append(_ManagedProc("ros2_control_node", popen, "cm"))
        time.sleep(self.SPAWN_SETTLE_S)

    def _start_spawners(self, cfg: _StackConfig) -> Optional[str]:
        names = controllers_to_spawn(cfg.controllers_yaml)
        if not names:
            return "no controllers found in controllers.yaml"
        for name in names:
            cmd = [
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                name,
                "--switch-timeout",
                "10",
            ]
            if cfg.use_gazebo_sim:
                cmd.extend(["--ros-args", "-p", "use_sim_time:=true"])
            popen = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
            )
            self._children.append(_ManagedProc(f"spawner_{name}", popen, "spawner"))
            time.sleep(1.0)
        return None

    def _restart_stack(self) -> tuple[bool, str]:
        if self._restart_lock:
            return False, "restart already in progress"
        self._restart_lock = True
        try:
            cfg = self._cfg()
            for p in (cfg.urdf_path, cfg.base_path, cfg.controllers_yaml):
                if not p.exists():
                    return False, f"missing path: {p}"

            self._terminate_children()
            urdf_xml = self._expand_robot_description(cfg)

            if not cfg.gazebo_only:
                self._start_rsp(cfg, urdf_xml)
                if not cfg.use_gazebo_sim:
                    self._start_cm(cfg)

            err = self._start_spawners(cfg)
            if err:
                return False, err

            note = ""
            if cfg.use_gazebo_sim:
                note = (
                    " Gazebo: spawners restarted; if URDF hardware topology changed, "
                    "restart Gazebo (gz_ros2_control loads URDF at spawn)."
                )
            return True, f"control stack restarted.{note}"
        except Exception as e:
            return False, str(e)
        finally:
            self._restart_lock = False

    def _on_restart(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, msg = self._restart_stack()
        response.success = ok
        response.message = msg
        if not ok:
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)
        return response


def main() -> None:
    rclpy.init()
    node = ControlSupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
