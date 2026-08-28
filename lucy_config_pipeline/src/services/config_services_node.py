from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node

from lucy_msgs.srv import ActivateConfig
from lucy_msgs.srv import DeleteConfig
from lucy_msgs.srv import GetConfig
from lucy_msgs.srv import GetMesh
from lucy_msgs.srv import ListConfigs
from lucy_msgs.srv import SaveConfig

from ..config_store import ConfigStore
from ..error_format import format_error_lines
from ..validation import urdf_crosscheck
from ..validation import validate_schema


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
        super().__init__('lucy_config_services')
        self._robot_package = robot_package
        self._store = config_store
        self._urdf_xacro = urdf_xacro
        self._base_path = base_path
        self._controller_config = controller_config

        self.create_service(ListConfigs, 'config/list', self._on_list_configs)
        self.create_service(GetConfig, 'config/get', self._on_get_config)
        self.create_service(SaveConfig, 'config/save', self._on_save_config)
        self.create_service(ActivateConfig, 'config/activate', self._on_activate_config)
        self.create_service(DeleteConfig, 'config/delete', self._on_delete_config)
        self.create_service(GetMesh, 'mesh/get', self._on_get_mesh)

    def _on_list_configs(
        self, _req: ListConfigs.Request, res: ListConfigs.Response
    ) -> ListConfigs.Response:
        try:
            res.config_names = self._store.list_configs()
            res.active_config = self._store.get_active_name()
            res.success = True
            res.message = 'ok'
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
            res.message = 'ok'
        except FileNotFoundError:
            res.success = False
            res.message = 'Configuration not found'
            res.robot_package = self._robot_package
            res.config_name = ''
            res.config_yaml = ''
            res.flashed_config_name = ''
            res.flashed_at = ''
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.robot_package = self._robot_package
            res.config_name = ''
            res.config_yaml = ''
            res.flashed_config_name = ''
            res.flashed_at = ''
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
                res.message = 'validation failed'
                return res

            self._store.write_named_yaml(req.config_name, req.config_yaml)
            if req.activate:
                self._store.activate(req.config_name)
            res.success = True
            res.message = 'saved'
        except ValueError as e:
            res.success = False
            res.message = 'validation failed'
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
            res.message = 'activated'
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
            res.message = 'deleted'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _on_get_mesh(self, req: GetMesh.Request, res: GetMesh.Response) -> GetMesh.Response:
        # Serves the mesh files referenced by the URDF (file:// / package:// /
        # absolute) so the web viewer can fetch them over ROS instead of the
        # filesystem. Reads are confined to the robot's description tree.
        try:
            candidate = self._mesh_path_from_ref(req.path)
            base = self._base_path.resolve()
            if base != candidate and base not in candidate.parents:
                res.success = False
                res.message = f'path outside robot description: {candidate}'
                return res
            if not candidate.is_file():
                res.success = False
                res.message = f'mesh not found: {candidate}'
                return res
            res.data = candidate.read_text()
            res.success = True
            res.message = 'ok'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _mesh_path_from_ref(self, ref: str) -> Path:
        """Resolve a URDF mesh reference to an absolute filesystem path."""
        if ref.startswith('file://'):
            return Path(ref[len('file://'):]).resolve()
        if ref.startswith('package://'):
            # package://<pkg>/<rel> — robot packages are rooted at base_path's parent.
            _, _, rel = ref[len('package://'):].partition('/')
            return (self._base_path.parent / rel).resolve()
        return Path(ref).resolve()


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = ConfigServicesNode(
        robot_package='thais_urdf',
        config_store=ConfigStore(Path('.')),
        urdf_xacro=Path('inmoov.urdf.xacro'),
        base_path=Path('.'),
        controller_config=Path('controllers.yaml'),
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
