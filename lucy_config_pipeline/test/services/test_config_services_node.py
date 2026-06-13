from pathlib import Path

import pytest
import rclpy

from src.config_store import ConfigStore
from src.services.config_services_node import ConfigServicesNode


@pytest.fixture
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestConfigServicesNode:
    def test_get_client_count_service(self, rclpy_init_shutdown, tmp_path: Path):
        """Test /get_client_count service returns the current client count."""
        from lucy_msgs.srv import GetInt

        node = ConfigServicesNode(
            robot_package="test_pkg",
            config_store=ConfigStore(tmp_path / "hardware"),
            urdf_xacro=Path("/dev/null"),
            base_path=tmp_path,
            controller_config=Path("/dev/null"),
        )
        node._client_count = 5

        response = node._on_get_client_count(GetInt.Request(), GetInt.Response())

        assert response.value == 5
        node.destroy_node()
