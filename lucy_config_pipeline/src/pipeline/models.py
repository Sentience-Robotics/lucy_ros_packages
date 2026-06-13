from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class PipelinePaths:
    config_dir: Path
    urdf_xacro: Path
    base_path: Path
    controller_config: Path
    robot_root: Path
    workspace_src: Path


@dataclass(frozen=True)
class FirmwarePaths:
    source_dir: Path
    build_dir: Path
