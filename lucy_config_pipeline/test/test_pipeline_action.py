from pathlib import Path

import pytest

from lucy_config_pipeline.config_store import ConfigStore
from lucy_config_pipeline.pipeline_action_server import (
    resolve_mapping_input,
    select_boards_to_process,
)


def test_resolve_mapping_input_active_and_named(tmp_path: Path):
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()
    store.write_named_yaml("default", "version: 1\nrobot_name: t\n")
    store.write_named_yaml("alt", "version: 1\nrobot_name: alt\n")
    store.activate("default")

    name, txt = resolve_mapping_input(store, "")
    assert name == "default"
    assert "robot_name: t" in txt

    name, txt = resolve_mapping_input(store, "alt")
    assert name == "alt"
    assert "robot_name: alt" in txt


def test_resolve_mapping_input_from_yaml_path(tmp_path: Path):
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()

    p = tmp_path / "custom.yaml"
    p.write_text("version: 1\nrobot_name: custom\n", encoding="utf-8")

    name, txt = resolve_mapping_input(store, str(p))
    assert name == "custom"
    assert "robot_name: custom" in txt


def test_select_boards_to_process():
    data = {"boards": {"rp2040_left_arm": {}, "rp2040_right_arm": {}}}

    assert select_boards_to_process(data, []) is None
    assert select_boards_to_process(data, ["rp2040_left_arm"]) == {"rp2040_left_arm"}

    with pytest.raises(ValueError, match="unknown boards"):
        select_boards_to_process(data, ["rp2040_foo"])
