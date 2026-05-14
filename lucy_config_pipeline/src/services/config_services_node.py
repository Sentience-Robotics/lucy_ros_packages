from __future__ import annotations

from pathlib import Path

from lucy_msgs.srv import ActivateConfig, DeleteConfig, GetConfig, ListConfigs, SaveConfig
import rclpy
from rclpy.node import Node

from ..config_store import ConfigStore
from ..error_format import format_error_lines
from ..validation import urdf_crosscheck, validate_schema


class ConfigServicesNode(Node):
    def __init__(
        self,
        *,
        robot_package: str,
        config_store: ConfigStore,
        urdf_xacro: Path,
        base_path: Path,
        controller_config: Path,
    ):
        super().__init__("lucy_config_services")
        self._robot_package = robot_package
        self._store = config_store
        self._urdf_xacro = urdf_xacro
        self._base_path = base_path
        self._controller_config = controller_config

        self.create_service(ListConfigs, "config/list", self._on_list_configs)
        self.create_service(GetConfig, "config/get", self._on_get_config)
        self.create_service(SaveConfig, "config/save", self._on_save_config)
        self.create_service(ActivateConfig, "config/activate", self._on_activate_config)
        self.create_service(DeleteConfig, "config/delete", self._on_delete_config)

    def _on_list_configs(
        self, _req: ListConfigs.Request, res: ListConfigs.Response
    ) -> ListConfigs.Response:
        try:
            res.config_names = self._store.list_configs()
            res.active_config = self._store.get_active_name()
            res.success = True
            res.message = "ok"
        except Exception as e:  # pragma: no cover - defensive wrapper
            res.success = False
            res.message = str(e)
        return res

    def _on_get_config(
        self, req: GetConfig.Request, res: GetConfig.Response
    ) -> GetConfig.Response:
        try:
            if req.config_name:
                res.config_name = req.config_name
                res.config_yaml = self._store.read_named_yaml(req.config_name)
            else:
                res.config_name = self._store.get_active_name()
                res.config_yaml = self._store.read_active_yaml()
            res.robot_package = self._robot_package
            res.flashed_config_name = self._store.get_flashed_name()
            res.flashed_at = self._store.get_flashed_at()
            res.success = True
            res.message = "ok"
        except FileNotFoundError:
            res.success = False
            res.message = "Configuration not found"
            res.robot_package = self._robot_package
            res.config_name = ""
            res.config_yaml = ""
            res.flashed_config_name = ""
            res.flashed_at = ""
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.robot_package = self._robot_package
            res.config_name = ""
            res.config_yaml = ""
            res.flashed_config_name = ""
            res.flashed_at = ""
        return res

    def _on_save_config(
        self, req: SaveConfig.Request, res: SaveConfig.Response
    ) -> SaveConfig.Response:
        try:
            data = validate_schema(req.config_yaml)
            report = urdf_crosscheck(
                data,
                self._urdf_xacro,
                self._base_path,
                self._controller_config,
            )
            res.urdf_warnings = report.warnings
            if report.errors:
                res.validation_errors = report.errors
                res.success = False
                res.message = "validation failed"
                return res

            self._store.write_named_yaml(req.config_name, req.config_yaml)
            if req.activate:
                self._store.activate(req.config_name)
            res.success = True
            res.message = "saved"
        except ValueError as e:
            res.success = False
            res.message = "validation failed"
            lines = [line for line in str(e).splitlines() if line.strip()]
            res.validation_errors = format_error_lines(lines)
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _on_activate_config(
        self, req: ActivateConfig.Request, res: ActivateConfig.Response
    ) -> ActivateConfig.Response:
        try:
            res.backup_name = self._store.activate(req.config_name)
            res.success = True
            res.message = "activated"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _on_delete_config(
        self, req: DeleteConfig.Request, res: DeleteConfig.Response
    ) -> DeleteConfig.Response:
        try:
            self._store.delete(req.config_name)
            res.success = True
            res.message = "deleted"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = ConfigServicesNode(
        robot_package="thais_urdf",
        config_store=ConfigStore(Path(".")),
        urdf_xacro=Path("inmoov.urdf.xacro"),
        base_path=Path("."),
        controller_config=Path("controllers.yaml"),
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
