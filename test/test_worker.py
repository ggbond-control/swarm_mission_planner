import time
from types import SimpleNamespace

from swarm_mission_planner.node import PlannerNode
from swarm_mission_planner.worker import PlanningWorker
from swarm_protocol.contracts import canonical, normalize_mission
from test_planner import grid


def finish(worker):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        result = worker.poll()
        if result is not None:
            return result
        time.sleep(0.01)
    raise AssertionError("planner worker did not finish")


def test_timeout_reaps_process_and_next_request_can_run(tmp_path):
    grid(tmp_path)
    mission = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A"],
            "map_package": str(tmp_path),
            "tasks": [],
        },
        "worker-test",
    )
    robots = {
        "A": {
            "pose": {"frame_id": "map_000", "pos": [2, 2, 0]},
            "speed": 1,
            "capabilities": ["device"],
        }
    }
    worker = PlanningWorker(timeout=0.001)
    try:
        assert worker.submit("expired", mission, robots, radius=0)
        assert not worker.submit("overlap", mission, robots)
        process = worker.process
        result = finish(worker)
        assert not result["success"] and "timeout" in result["error"]
        assert result["request_id"] == "expired" and not worker.busy
        assert process._closed  # child handle reaped before accepting a replacement
        worker.timeout = 10
        assert worker.submit("replacement", mission, robots, radius=0)
        result = finish(worker)
        assert result["success"] and result["request_id"] == "replacement"
    finally:
        worker.close()


def test_worker_failure_is_structured_and_does_not_poison_next_job(tmp_path):
    worker = PlanningWorker(timeout=10)
    try:
        worker.submit("bad-map", {"map_package": str(tmp_path)}, {})
        result = finish(worker)
        assert not result["success"] and result["request_id"] == "bad-map"
        assert "no Nav2 maps" in result["error"] and not worker.busy
    finally:
        worker.close()


def test_duplicate_request_keeps_worker_but_new_request_supersedes_it(tmp_path):
    grid(tmp_path)
    mission = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A"],
            "map_package": str(tmp_path),
            "tasks": [],
        },
        "supersede",
    )
    robots = {
        "A": {
            "pose": {"frame_id": "map_000", "pos": [2, 2, 0]},
            "speed": 1,
            "capabilities": ["device"],
        }
    }
    worker = PlanningWorker(10)
    node = SimpleNamespace(
        worker=worker,
        get_parameter=lambda key: SimpleNamespace(
            value=0.1 if key == "footprint_radius" else 0.05
        ),
    )

    def send(request_id):
        PlannerNode.request(
            node,
            SimpleNamespace(
                data=canonical(
                    {"request_id": request_id, "mission": mission, "robots": robots}
                )
            ),
        )

    try:
        send("old")
        original = worker.process
        send("old")
        assert worker.process is original
        send("new")
        assert original._closed and worker.request_id == "new"
        assert finish(worker)["success"]
    finally:
        worker.close()
