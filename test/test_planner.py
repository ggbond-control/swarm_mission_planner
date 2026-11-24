import math

import pytest
import yaml
from PIL import Image
from swarm_mission_planner.planner import map_fingerprint
from swarm_mission_planner.planner import plan
from swarm_protocol.contracts import normalize_mission


def grid(root, name="map_000", origin=None):
    root.mkdir(exist_ok=True, parents=True)
    Image.new("L", (40, 40), 255).save(root / (name + ".png"))
    (root / (name + ".yaml")).write_text(
        yaml.safe_dump(
            {
                "image": name + ".png",
                "resolution": 1.0,
                "origin": origin or [0, 0, 0],
                "free_thresh": 0.196,
                "occupied_thresh": 0.65,
            }
        )
    )
    return root / (name + ".yaml")


def test_native_preflight_identity_and_rotation(tmp_path):
    grid(tmp_path, origin=[40, 20, math.pi / 2])
    robots = {"A": {"pose": {"frame_id": "map_000", "pos": [38, 22, 0]},
                    "speed": 1, "capabilities": ["device"]}}
    task = {"id": "t", "location": {"frame_id": "map_000", "pos": [30, 30, 0]}, "tasks": []}
    mission = normalize_mission({
        "round_epoch": 1, "robot_ids": ["A"], "map_package": str(tmp_path), "tasks": [task]
    }, "rotation")
    before = map_fingerprint(str(tmp_path), 0, 0)
    assert plan(mission, robots, radius=0, free_override=0)["map_digest"] == before
    image = Image.open(tmp_path / "map_000.png")
    image.putpixel((5, 10), 0)
    image.save(tmp_path / "map_000.png")
    assert map_fingerprint(str(tmp_path), 0, 0) != before


def portals(root):
    (root / "map_relations.csv").write_text("from_map,to_map,dx,dy\nROOT,a,0,0\na,b,0,0\n")
    (root / "transition_points.csv").write_text(
        "transition_id,from_map,to_map,world_x,world_y,world_z,world_yaw_rad,bidirectional,type\n"
        "t,a,b,5.5,5.5,0,0,true,flat\n")


def test_multirobot_cross_map_requires_root_collision_support(tmp_path):
    grid(tmp_path, "a")
    grid(tmp_path, "b")
    portals(tmp_path)
    task = {"id": "t", "location": {"frame_id": "a", "pos": [5, 5, 0]}, "tasks": []}
    m = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A", "B"],
            "map_package": str(tmp_path),
            "tasks": [task],
        },
        "maps",
    )
    robots = {
        r: {
            "pose": {"frame_id": name, "pos": [2, 2, 0]},
            "speed": 1,
            "capabilities": ["device"],
        }
        for r, name in [("A", "a"), ("B", "b")]
    }
    with pytest.raises(ValueError, match="ROOT"):
        plan(m, robots)


def test_aggregate_invalid_start_diagnostics(tmp_path):
    grid(tmp_path)
    m = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A", "B"],
            "map_package": str(tmp_path),
            "tasks": [],
        },
        "m",
    )
    robots = {
        r: {
            "pose": {"frame_id": "map_000", "pos": [-20.0, -30.0, 0]},
            "speed": 1.0,
            "capabilities": ["device"],
        }
        for r in ["A", "B"]
    }
    with pytest.raises(ValueError) as error:
        plan(m, robots, radius=0)
    assert all(x in str(error.value) for x in ["A", "B", "超出地图范围", "-20.000"])


def test_capability_matching_and_input_operations_preserved(tmp_path):
    grid(tmp_path)
    task = {
        "id": "gas",
        "location": {"frame_id": "map_000", "pos": [5, 5, 0]},
        "tasks": [
            {"method": "gas", "requirements": ["gas_sensor"], "params": {"foo": "bar"}}
        ],
    }
    m = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A"],
            "map_package": str(tmp_path),
            "tasks": [task],
        },
        "m",
    )
    robot = {
        "A": {
            "pose": {"frame_id": "map_000", "pos": [2, 2, 0]},
            "speed": 1.0,
            "capabilities": ["device"],
        }
    }
    result = plan(m, robot, radius=0)
    assert "gas" in result["unassignable"]
    robot["A"]["capabilities"].append("gas_sensor")
    assert plan(m, robot, radius=0)["routes"][0]["task_ids"] == ["gas"]
    assert m["tasks"][0]["tasks"][0]["params"] == {"foo": "bar"}


def test_route_preserves_portal_at_first_and_last_cell(tmp_path):
    grid(tmp_path, "a")
    grid(tmp_path, "b")
    portals(tmp_path)
    task = {"id": "t", "location": {"frame_id": "b", "pos": [5.5, 5.5, 0]}, "tasks": []}
    m = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A"],
            "map_package": str(tmp_path),
            "tasks": [task],
        },
        "m",
    )
    robots = {
        "A": {
            "pose": {"frame_id": "a", "pos": [5.5, 5.5, 0]},
            "speed": 1.0,
            "capabilities": ["device"],
        }
    }
    route = plan(m, robots, radius=0)["routes"][0]
    assert [p["location"]["frame_id"] for p in route["checkpoints"]] == ["a", "b", "b"]
