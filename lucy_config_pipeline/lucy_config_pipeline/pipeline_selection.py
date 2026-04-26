"""Backward-compatible import wrapper."""

from .pipeline.selection import (
    board_build_plan,
    resolve_firmware_paths,
    resolve_mapping_input,
    select_boards_to_process,
)

__all__ = [
    "resolve_mapping_input",
    "select_boards_to_process",
    "resolve_firmware_paths",
    "board_build_plan",
]
