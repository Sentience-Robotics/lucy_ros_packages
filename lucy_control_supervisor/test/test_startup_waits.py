"""Startup waits on the ROS graph rather than on fixed sleeps."""

import time
from pathlib import Path

import pytest

SRC = (Path(__file__).resolve().parents[1]
       / "lucy_control_supervisor" / "supervisor_node.py").read_text()


def test_no_fixed_sleep_between_spawners():
    # Spawners contend for one lock file, so overlapping them converts
    # concurrency into 20s lock waits.
    assert "time.sleep(1.0)" not in SRC


def test_controller_manager_is_waited_for_before_spawners():
    assert "/controller_manager/list_controllers" in SRC
    assert SRC.index("_service_available") < SRC.index("_start_spawners")


def test_robot_state_publisher_is_waited_for_by_topic():
    assert "count_publishers('/robot_description')" in SRC


def test_urdf_params_file_is_unlinked():
    # It holds a full URDF; one leaks per start otherwise.
    assert "os.unlink(params_file)" in SRC


def test_waits_are_bounded():
    assert "READY_TIMEOUT_S" in SRC and "SPAWNER_TIMEOUT_S" in SRC


def test_discovery_settle_is_kept():
    """The absence of a foreign controller_manager cannot be waited for."""
    assert "DISCOVERY_SETTLE_S" in SRC


class _FakeGraph:
    """Minimal stand-in for the Node graph API used by the waits."""

    def __init__(self, appear_after=0.0):
        self._deadline = time.time() + appear_after

    def get_service_names_and_types(self):
        if time.time() < self._deadline:
            return [("/other/service", ["x"])]
        return [("/controller_manager/list_controllers", ["x"])]


def _wait_for(predicate, timeout_s, poll_s=0.01):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return predicate()


def test_wait_returns_as_soon_as_the_condition_holds():
    graph = _FakeGraph(appear_after=0.0)
    started = time.time()
    ok = _wait_for(
        lambda: any(n == "/controller_manager/list_controllers"
                    for n, _ in graph.get_service_names_and_types()),
        timeout_s=5.0,
    )
    assert ok and time.time() - started < 0.5


def test_wait_gives_up_at_the_deadline():
    graph = _FakeGraph(appear_after=99.0)
    started = time.time()
    assert _wait_for(
        lambda: any(n == "/controller_manager/list_controllers"
                    for n, _ in graph.get_service_names_and_types()),
        timeout_s=0.3,
    ) is False
    assert time.time() - started < 2.0
