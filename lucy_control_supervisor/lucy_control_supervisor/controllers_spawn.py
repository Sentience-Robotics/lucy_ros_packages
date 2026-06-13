"""Shared helper: controller names declared in controllers.yaml."""

from __future__ import annotations

from pathlib import Path

import yaml


def controllers_to_spawn(controllers_yaml_path: Path) -> list[str]:
    """Return controller names under controller_manager.ros__parameters (except update_rate)."""
    data = yaml.safe_load(controllers_yaml_path.read_text(encoding="utf-8")) or {}
    cm_params = data.get("controller_manager", {}).get("ros__parameters", {})
    if not isinstance(cm_params, dict):
        return []
    names = [name for name in cm_params.keys() if name != "update_rate"]
    jsb = "joint_state_broadcaster"
    if jsb in names:
        names.remove(jsb)
        return [jsb, *names]
    return names
