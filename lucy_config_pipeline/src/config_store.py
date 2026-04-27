from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import shutil
import yaml


@dataclass(frozen=True)
class ConfigStore:
    config_dir: Path

    def __post_init__(self) -> None:
        object.__setattr__(self, "config_dir", self.config_dir.resolve())

    @property
    def active_yaml(self) -> Path:
        return self.config_dir / "active.yaml"

    @property
    def active_meta_yaml(self) -> Path:
        return self.config_dir / "active_meta.yaml"

    @property
    def named_dir(self) -> Path:
        return self.config_dir / "configs"

    @property
    def backups_dir(self) -> Path:
        return self.config_dir / "backups"

    def ensure_layout(self) -> None:
        self.config_dir.mkdir(parents=True, exist_ok=True)
        self.named_dir.mkdir(parents=True, exist_ok=True)
        self.backups_dir.mkdir(parents=True, exist_ok=True)

    def list_configs(self) -> list[str]:
        self.ensure_layout()
        return sorted(p.stem for p in self.named_dir.glob("*.yaml"))

    def get_active_name(self) -> str:
        if not self.active_meta_yaml.is_file():
            return "default"
        data = yaml.safe_load(self.active_meta_yaml.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            name = str(data.get("name", "")).strip()
            if name:
                return name
        return "default"

    def _named_path(self, config_name: str) -> Path:
        if not config_name or "/" in config_name or ".." in config_name:
            raise ValueError("config_name must be a simple non-empty name")
        return self.named_dir / f"{config_name}.yaml"

    def read_named_yaml(self, config_name: str) -> str:
        path = self._named_path(config_name)
        if not path.is_file():
            raise FileNotFoundError(path)
        return path.read_text(encoding="utf-8")

    def read_active_yaml(self) -> str:
        if not self.active_yaml.is_file():
            raise FileNotFoundError(self.active_yaml)
        return self.active_yaml.read_text(encoding="utf-8")

    def write_named_yaml(self, config_name: str, config_yaml: str) -> Path:
        self.ensure_layout()
        path = self._named_path(config_name)
        path.write_text(config_yaml, encoding="utf-8")
        return path

    def activate(self, config_name: str) -> str:
        self.ensure_layout()
        src = self._named_path(config_name)
        if not src.is_file():
            raise FileNotFoundError(src)

        backup_name = ""
        if self.active_yaml.is_file():
            old = self.get_active_name()
            ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            backup_name = f"{ts}_{old or 'active'}"
            shutil.copy2(self.active_yaml, self.backups_dir / f"{backup_name}.yaml")

        shutil.copy2(src, self.active_yaml)
        self.active_meta_yaml.write_text(
            f'name: "{config_name}"\nactivated_at: "{datetime.utcnow().isoformat()}Z"\n',
            encoding="utf-8",
        )
        return backup_name

    def delete(self, config_name: str) -> None:
        active = self.get_active_name()
        if config_name in {"default", active}:
            raise ValueError("cannot delete default or active config")
        path = self._named_path(config_name)
        if not path.is_file():
            raise FileNotFoundError(path)
        path.unlink()
