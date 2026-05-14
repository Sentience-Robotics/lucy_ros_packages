from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import tempfile
import threading

from lucy_config_generator.generate import generate
from lucy_msgs.action import ConfigurePipeline
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from ..config_store import ConfigStore
from ..error_format import format_error_lines
from ..validation import urdf_crosscheck, validate_schema
from .build import run_build_phase
from .flash import (
    flash_picotool_timeout_seconds,
    flash_uptime_wait_seconds,
    flash_usb_wait_seconds,
    run_flash_phase,
)
from .models import PipelinePaths
from .selection import board_build_plan, resolve_mapping_input, select_boards_to_process


class PipelineActionServer(Node):
    _BUILD_TIMEOUT_SECONDS = int(os.environ.get("LUCY_PIPELINE_BUILD_TIMEOUT_SEC", "300"))

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
        q = round(max(0.0, min(1.0, float(progress))), 2)
        fb.progress = q
        fb.detail = json.dumps(
            {
                "phase": phase,
                "board": board,
                "progress": q,
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

            self._feedback(goal_handle, phase="validate", progress=0.1, detail="schema validation")
            data = validate_schema(config_yaml)

            for path in (
                self._paths.urdf_xacro,
                self._paths.base_path,
                self._paths.controller_config,
            ):
                if not path.exists():
                    raise FileNotFoundError(path)

            self._feedback(goal_handle, phase="validate", progress=0.6, detail="urdf cross-check")
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

            build_failed_boards: list[str] = []
            if not goal.dry_run:
                build_failed_boards = run_build_phase(
                    data=data,
                    selected_boards=boards,
                    workspace_src=self._paths.workspace_src,
                    timeout_seconds=self._BUILD_TIMEOUT_SECONDS,
                    feedback=lambda **kwargs: self._feedback(goal_handle, **kwargs),
                    log_error=lambda msg: self.get_logger().error(msg),
                )

            plan_boards = [b for b, _ in board_build_plan(data, boards)]
            plan_set = set(plan_boards)
            built_ok = {b for b in plan_boards if b not in set(build_failed_boards)}

            if not goal.dry_run and goal.build_only and build_failed_boards:
                result.errors.extend(
                    format_error_lines(
                        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
                    )
                )
                result.message = "build failed"
                goal_handle.abort()
                return result

            if not goal.dry_run and plan_set and not built_ok:
                result.errors.extend(
                    format_error_lines(
                        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
                    )
                )
                result.message = "build failed"
                goal_handle.abort()
                return result

            if build_failed_boards:
                result.errors.extend(
                    format_error_lines(
                        [f"build failed for boards: {', '.join(sorted(build_failed_boards))}"]
                    )
                )

            flash_failed: list[str] = []
            if not goal.dry_run and not goal.build_only:
                flash_failed, flashed_ok = run_flash_phase(
                    data=data,
                    selected_boards=boards,
                    boards_built_ok=built_ok,
                    workspace_src=self._paths.workspace_src,
                    picotool_timeout_seconds=flash_picotool_timeout_seconds(),
                    usb_wait_seconds=flash_usb_wait_seconds(),
                    uptime_wait_seconds=flash_uptime_wait_seconds(),
                    node=self,
                    feedback=lambda **kwargs: self._feedback(goal_handle, **kwargs),
                    log_error=lambda msg: self.get_logger().error(msg),
                )
                result.boards_flashed = flashed_ok
                if flash_failed:
                    result.errors.extend(
                        format_error_lines(
                            [f"flash failed for boards: {', '.join(sorted(flash_failed))}"]
                        )
                    )
                    result.message = "flash failed"
                    goal_handle.abort()
                    return result
                if flashed_ok:
                    self._store.record_flashed_preset(result.config_name)

            result.success = True
            if build_failed_boards:
                result.message = "pipeline completed with partial build failure"
            else:
                result.message = "pipeline completed"
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
