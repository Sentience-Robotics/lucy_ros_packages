from pathlib import Path

from lucy_config_pipeline.config_store import ConfigStore


def test_save_list_activate_delete(tmp_path: Path):
    store = ConfigStore(tmp_path / "hardware")
    store.ensure_layout()

    a = "version: 1\nrobot_name: t\n"
    b = "version: 1\nrobot_name: t2\n"
    store.write_named_yaml("default", a)
    store.write_named_yaml("alt", b)

    assert store.list_configs() == ["alt", "default"]

    backup = store.activate("default")
    assert backup == ""
    assert store.get_active_name() == "default"

    backup = store.activate("alt")
    assert backup != ""
    assert (store.backups_dir / f"{backup}.yaml").is_file()

    try:
        store.delete("default")
        assert False, "default deletion should fail"
    except ValueError:
        pass
