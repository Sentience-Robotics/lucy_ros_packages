from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import tempfile
import threading

from lucy_config_generator.generate import generate
from lucy_msgs.action import ConfigurePipeline
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from .config_store import ConfigStore
from .error_format import format_error_lines
from .validation import urdf_crosscheck, validate_schema


@dataclass(frozen=True)
class PipelinePaths:
    config_dir: Path
    urdf_xacro: Path
    base_path: Path
    controller_config: Path
    robot_root: Path
    workspace_src: Path


def resolve_mapping_input(store: ConfigStore, mapping_file: str) -> tuple[str, str]:
    """
    Resolve action goal mapping selector into ``(config_name, yaml_text)``.

    ``mapping_file`` may be:
    - empty => active config
    - absolute/relative ``*.yaml`` existing file path
    - named config in config store
    """
    if mapping_file:
        mapping_path = Path(mapping_file)
        if mapping_file.endswith(".yaml") and mapping_path.is_file():
            return mapping_path.stem, mapping_path.read_text(encoding="utf-8")
        return mapping_file, store.read_named_yaml(mapping_file)
    return store.get_active_name(), store.read_active_yaml()


def select_boards_to_process(data: dict, requested: list[str]) -> set[str] | None:
    """Return selected board ids, or None for all."""
    if not requested:
        return None
    boards = {b for b in requested if b}
    known = set(data.get("boards", {}).keys())
    unknown = sorted(boards - known)
    if unknown:
        raise ValueError(f"unknown boards requested: {', '.join(unknown)}")
    return boards


class PipelineActionServer(Node):
    def __init__(self, *, paths: PipelinePaths, config_store: ConfigStore):
        super().__init__("lucy_config_pipeline")
        self._paths = paths
        self._store = config_store
        self._busy_lock = threading.Lock()
        self._busy = False
        self._action_server = ActionServer(
            self,
            ConfigurePipeline,
            "configure_pipeline",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()

    def _goal_callback(self, _goal_request: ConfigurePipeline.Goal) -> GoalResponse:
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _feedback(
        self,
        goal_handle,
        *,
        phase: str,
        progress: float,
        detail: str,
        board: str = "",
    ) -> None:
        fb = ConfigurePipeline.Feedback()
        fb.phase = phase
        fb.progress = max(0.0, min(1.0, float(progress)))
        fb.detail = json.dumps(
            {
                "phase": phase,
                "board": board,
                "progress": fb.progress,
                "message": detail,
            }
        )
        fb.board = board
        goal_handle.publish_feedback(fb)

    def _execute(self, goal_handle):
        result = ConfigurePipeline.Result()
        result.success = False
        result.config_name = self._store.get_active_name()
        result.boards_flashed = []
        result.errors = []

        try:
            goal = goal_handle.request
            result.config_name, config_yaml = resolve_mapping_input(self._store, goal.mapping_file)

            self._feedback(
                goal_handle,
                phase="validate",
                progress=0.1,
                detail="schema validation",
            )
            data = validate_schema(config_yaml)

            for path in (
                self._paths.urdf_xacro,
                self._paths.base_path,
                self._paths.controller_config,
            ):
                if not path.exists():
                    raise FileNotFoundError(path)

            self._feedback(
                goal_handle,
                phase="validate",
                progress=0.6,
                detail="urdf cross-check",
            )
            report = urdf_crosscheck(
                data,
                self._paths.urdf_xacro,
                self._paths.base_path,
                self._paths.controller_config,
            )
            if report.errors:
                result.errors.extend(format_error_lines(report.errors))
                result.message = "validation failed"
                goal_handle.abort()
                return result

            with tempfile.TemporaryDirectory(prefix="lucy_config_pipeline_") as tmp:
                out_dir = Path(tmp)
                boards = select_boards_to_process(data, list(goal.boards_to_flash))
                self._feedback(
                    goal_handle,
                    phase="generate",
                    progress=0.2,
                    detail="rendering artifacts",
                )
                (out_dir / "input.yaml").write_text(config_yaml, encoding="utf-8")
                generate(
                    input_yaml=out_dir / "input.yaml",
                    urdf_xacro=self._paths.urdf_xacro,
                    base_path=self._paths.base_path,
                    controller_config=self._paths.controller_config,
                    output_dir=out_dir,
                    targets={"firmware", "ros2_control", "controllers"},
                    boards_filter=boards,
                )

                if not goal.dry_run:
                    self._feedback(
                        goal_handle,
                        phase="generate",
                        progress=0.7,
                        detail="installing outputs",
                    )
                    self._install_generated_outputs(out_dir, data)

            self._feedback(
                goal_handle,
                phase="build",
                progress=0.0,
                detail="build phase is a stub",
            )
            if not goal.dry_run and not goal.build_only:
                self._feedback(
                    goal_handle,
                    phase="flash",
                    progress=0.0,
                    detail="flash phase is a stub",
                )
                if boards:
                    result.boards_flashed = sorted(boards)
                else:
                    result.boards_flashed = sorted(data.get("boards", {}).keys())

            result.success = True
            result.message = "pipeline completed (build/flash phases are stubs)"
            goal_handle.succeed()
            return result
        except Exception as e:  # pragma: no cover - top-level action guard
            lines = [line for line in str(e).splitlines() if line.strip()]
            raw = lines if lines else [str(e)]
            result.errors.extend(format_error_lines(raw))
            result.message = f"pipeline failed: {e}"
            goal_handle.abort()
            return result
        finally:
            with self._busy_lock:
                self._busy = False

    def _install_generated_outputs(self, out_dir: Path, data: dict) -> None:
        xacro_dst = (
            self._paths.robot_root
            / "description"
            / "ros2_control"
            / "inmoov_ros2_control.xacro"
        )
        ctrl_dst = self._paths.robot_root / "config" / "controllers.yaml"
        xacro_dst.parent.mkdir(parents=True, exist_ok=True)
        ctrl_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(out_dir / "inmoov_ros2_control.xacro", xacro_dst)
        shutil.copy2(out_dir / "controllers.yaml", ctrl_dst)

        firmware_src_dir = str(data.get("firmware", {}).get("source_dir", "")).strip()
        if not firmware_src_dir:
            return
        fw_cfg_dir = (self._paths.workspace_src / firmware_src_dir / "config").resolve()
        fw_cfg_dir.mkdir(parents=True, exist_ok=True)
        for cfile in out_dir.glob("config_*.c"):
            shutil.copy2(cfile, fw_cfg_dir / cfile.name)
