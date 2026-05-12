#!/usr/bin/env python3
# Copyright 2025 Sentience Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
ROS2 node that provides /config/* services for hardware YAML lifecycle management.

Services:
  /config/get      — read active.yaml (empty name) or configs/<name>.yaml
  /config/save     — validate and write configs/<name>.yaml
  /config/list     — list all saved configs; report active config name
  /config/activate — copy configs/<name>.yaml → active.yaml; backup old active
  /config/delete   — remove configs/<name>.yaml (not active, not "default")
"""

from __future__ import annotations

import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from lucy_msgs.srv import ActivateConfig, DeleteConfig, GetConfig, ListConfigs, SaveConfig

# ---------------------------------------------------------------------------
# Validation (mirrors test_hardware_yaml.py logic)
# ---------------------------------------------------------------------------

REQUIRED_ROOT = ("version", "robot_name", "firmware", "controller_manager", "boards", "actuators", "sensors")
REQUIRED_ACTUATOR = (
    "id", "urdf_joint", "board", "virtual_pin", "physical_pin", "servo_type",
    "offset_deg", "direction", "scale", "servo_min_deg", "servo_max_deg",
    "servo_default_deg", "enabled",
)
REQUIRED_SENSOR = (
    "id", "type", "associated_actuator", "board", "virtual_pin", "physical_pin",
    "min_value", "max_value", "enabled",
)
REQUIRED_BOARD = (
    "serial_id", "board_class", "internal_servo_slots", "firmware_target",
    "compile_definition", "topic_actuators", "topic_sensors", "controller",
)
BOARD_CLASSES = frozenset({"internal_servo_only", "internal_servo_i2c_pwm"})
_BOARD_ID_RE = re.compile(r"^rp2040_[a-z][a-z0-9_]*$")
_TOPIC_RE = re.compile(r"^[a-z][a-z0-9_/]*$")
_SERIAL_ID_RE = re.compile(r"^[A-Za-z0-9]*$")


def _validate_hardware_yaml(data: dict[str, Any]) -> list[str]:
    """Return a list of validation error strings; empty list means valid."""
    errors: list[str] = []

    for key in REQUIRED_ROOT:
        if key not in data:
            errors.append(f"missing root key: {key}")
    if errors:
        return errors

    if data["version"] != 1:
        errors.append("version must be 1")

    boards: dict[str, Any] = data.get("boards", {})
    if not isinstance(boards, dict) or not boards:
        errors.append("boards must be a non-empty map")
        return errors

    for bid, bdef in boards.items():
        for k in REQUIRED_BOARD:
            if k not in bdef:
                errors.append(f"board {bid}: missing {k}")
        ctrl = bdef.get("controller", {})
        if "name" not in ctrl or "type" not in ctrl:
            errors.append(f"board {bid}: controller needs name and type")
        if not _BOARD_ID_RE.fullmatch(bid):
            errors.append(f"board id {bid!r} must match {_BOARD_ID_RE.pattern}")
        bc = bdef.get("board_class", "")
        if bc not in BOARD_CLASSES:
            errors.append(f"board {bid}: board_class must be one of {sorted(BOARD_CLASSES)}, got {bc!r}")
        sid = bdef.get("serial_id", "")
        if not isinstance(sid, str):
            errors.append(f"board {bid}: serial_id must be a string")
        elif sid and not _SERIAL_ID_RE.fullmatch(sid):
            errors.append(f"board {bid}: serial_id must be empty or alphanumeric")
        for topic_key in ("topic_actuators", "topic_sensors"):
            t = bdef.get(topic_key, "")
            if not isinstance(t, str) or not t:
                errors.append(f"board {bid}: {topic_key} must be a non-empty string")
            elif not _TOPIC_RE.fullmatch(t):
                errors.append(f"board {bid}: {topic_key} must match {_TOPIC_RE.pattern}")

    actuators: list[dict[str, Any]] = data.get("actuators", [])
    if not isinstance(actuators, list):
        errors.append("actuators must be a list")
        return errors

    by_board: dict[str, list[dict[str, Any]]] = {}
    for a in actuators:
        for k in REQUIRED_ACTUATOR:
            if k not in a:
                errors.append(f"actuator {a.get('id')}: missing {k}")
        b = a.get("board", "")
        if b not in boards:
            errors.append(f"actuator {a.get('id')}: unknown board {b!r}")
        else:
            by_board.setdefault(b, []).append(a)

    for board_id, lst in by_board.items():
        vpins = sorted(int(x["virtual_pin"]) for x in lst)
        if len(vpins) != len(set(vpins)):
            errors.append(f"board {board_id}: duplicate virtual_pin")
        elif vpins != list(range(len(vpins))):
            errors.append(f"board {board_id}: virtual_pin must be contiguous from 0..N-1, got {vpins}")
        for a in lst:
            lo = a.get("servo_min_deg")
            hi = a.get("servo_max_deg")
            d = a.get("servo_default_deg")
            if lo is not None and hi is not None and d is not None:
                if float(d) < float(lo) or float(d) > float(hi):
                    errors.append(f"actuator {a['id']}: servo_default_deg {d} out of [{lo}, {hi}]")

    sensors: list[dict[str, Any]] = data.get("sensors", [])
    if not isinstance(sensors, list):
        errors.append("sensors must be a list")
        return errors
    actuator_ids = {a.get("id") for a in actuators}
    for s in sensors:
        for k in REQUIRED_SENSOR:
            if k not in s:
                errors.append(f"sensor {s.get('id')}: missing {k}")
        if s.get("board") not in boards:
            errors.append(f"sensor {s.get('id')}: unknown board {s.get('board')!r}")
        if s.get("associated_actuator") not in actuator_ids:
            errors.append(
                f"sensor {s.get('id')}: associated_actuator {s.get('associated_actuator')!r} not found"
            )

    return errors


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class HardwareConfigServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("lucy_config_services")

        share = get_package_share_directory("thais_urdf")
        self._hw_dir = Path(share) / "config" / "hardware"
        self._configs_dir = self._hw_dir / "configs"
        self._backups_dir = self._hw_dir / "backups"
        self._active_yaml = self._hw_dir / "active.yaml"
        self._active_meta = self._hw_dir / "active_meta.yaml"

        self._configs_dir.mkdir(parents=True, exist_ok=True)
        self._backups_dir.mkdir(parents=True, exist_ok=True)

        self.create_service(GetConfig, "/config/get", self._handle_get)
        self.create_service(SaveConfig, "/config/save", self._handle_save)
        self.create_service(ListConfigs, "/config/list", self._handle_list)
        self.create_service(ActivateConfig, "/config/activate", self._handle_activate)
        self.create_service(DeleteConfig, "/config/delete", self._handle_delete)

        self.get_logger().info(f"lucy_config_services ready — managing {self._hw_dir}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _read_active_name(self) -> str:
        if self._active_meta.is_file():
            try:
                meta = yaml.safe_load(self._active_meta.read_text(encoding="utf-8")) or {}
                return str(meta.get("name", ""))
            except Exception:
                pass
        return ""

    def _write_active_meta(self, name: str) -> None:
        meta = {
            "name": name,
            "activated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        }
        self._active_meta.write_text(yaml.dump(meta, default_flow_style=False), encoding="utf-8")

    def _config_path(self, name: str) -> Path:
        return self._configs_dir / f"{name}.yaml"

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _handle_get(
        self, request: GetConfig.Request, response: GetConfig.Response
    ) -> GetConfig.Response:
        name = (request.config_name or "").strip()
        try:
            if not name:
                if not self._active_yaml.is_file():
                    response.success = False
                    response.message = "active.yaml not found"
                    return response
                config_yaml = self._active_yaml.read_text(encoding="utf-8")
                config_name = self._read_active_name()
            else:
                path = self._config_path(name)
                if not path.is_file():
                    response.success = False
                    response.message = f"Config {name!r} not found"
                    return response
                config_yaml = path.read_text(encoding="utf-8")
                config_name = name

            response.success = True
            response.message = ""
            response.config_name = config_name
            response.config_yaml = config_yaml
        except Exception as e:
            self.get_logger().error(f"get error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _handle_save(
        self, request: SaveConfig.Request, response: SaveConfig.Response
    ) -> SaveConfig.Response:
        name = (request.config_name or "").strip()
        if not name:
            response.success = False
            response.message = "config_name must not be empty"
            return response
        try:
            data = yaml.safe_load(request.config_yaml)
            if not isinstance(data, dict):
                response.success = False
                response.message = "config_yaml must be a YAML mapping"
                return response
            errors = _validate_hardware_yaml(data)
            if errors:
                response.success = False
                response.message = "Validation failed"
                response.validation_errors = errors
                return response

            self._config_path(name).write_text(request.config_yaml, encoding="utf-8")

            if request.activate:
                self._do_activate(name)

            response.success = True
            response.message = f"Saved config {name!r}" + (" and activated" if request.activate else "")
        except Exception as e:
            self.get_logger().error(f"save error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _handle_list(
        self, request: ListConfigs.Request, response: ListConfigs.Response
    ) -> ListConfigs.Response:
        try:
            names = sorted(p.stem for p in self._configs_dir.glob("*.yaml"))
            response.success = True
            response.message = ""
            response.active_config = self._read_active_name()
            response.config_names = names
        except Exception as e:
            self.get_logger().error(f"list error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _handle_activate(
        self, request: ActivateConfig.Request, response: ActivateConfig.Response
    ) -> ActivateConfig.Response:
        name = (request.config_name or "").strip()
        if not name:
            response.success = False
            response.message = "config_name must not be empty"
            return response
        try:
            backup_name = self._do_activate(name)
            response.success = True
            response.message = f"Activated config {name!r}"
            response.backup_name = backup_name
        except Exception as e:
            self.get_logger().error(f"activate error: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _do_activate(self, name: str) -> str:
        src = self._config_path(name)
        if not src.is_file():
            raise FileNotFoundError(f"Config {name!r} not found in configs/")

        backup_name = ""
        if self._active_yaml.is_file():
            ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            prev_name = self._read_active_name() or "unknown"
            backup_name = f"{prev_name}_{ts}"
            shutil.copy2(self._active_yaml, self._backups_dir / f"{backup_name}.yaml")

        shutil.copy2(src, self._active_yaml)
        self._write_active_meta(name)
        self.get_logger().info(f"Activated config {name!r} (backup: {backup_name or 'none'})")
        return backup_name

    def _handle_delete(
        self, request: DeleteConfig.Request, response: DeleteConfig.Response
    ) -> DeleteConfig.Response:
        name = (request.config_name or "").strip()
        if not name:
            response.success = False
            response.message = "config_name must not be empty"
            return response
        if name == "default":
            response.success = False
            response.message = "Cannot delete the 'default' config"
            return response
        if name == self._read_active_name():
            response.success = False
            response.message = f"Config {name!r} is currently active; activate another first"
            return response
        try:
            path = self._config_path(name)
            if not path.is_file():
                response.success = False
                response.message = f"Config {name!r} not found"
                return response
            path.unlink()
            response.success = True
            response.message = f"Deleted config {name!r}"
        except Exception as e:
            self.get_logger().error(f"delete error: {e}")
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HardwareConfigServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
