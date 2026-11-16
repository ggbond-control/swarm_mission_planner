import math

import pytest
import yaml
from PIL import Image
from swarm_mission_planner.maps import Grid, MapBundle
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


def test_rotated_origin_round_trip_and_obstacles(tmp_path):
    g = Grid(grid(tmp_path, origin=[10, 20, math.pi / 2]))
    assert g.cell(*g.point((5, 6))[:2]) == (5, 6)
    assert g.reason((-1, 2)) == "超出地图范围"


def test_zero_free_threshold_keeps_exactly_white_pixels_traversable(tmp_path):
    g = Grid(grid(tmp_path), free_override=0)
    assert not g.reason((10, 10))


def test_search_budget_exhaustion_is_not_reported_as_unreachable(tmp_path, monkeypatch):
    from swarm_mission_planner.maps import SearchBudgetExceeded

    grid(tmp_path)
    task = {
        "id": "t",
        "location": {"frame_id": "map_000", "pos": [5, 5, 0]},
        "tasks": [],
    }
    m = normalize_mission(
        {
            "round_epoch": 1,
            "robot_ids": ["A"],
            "map_package": str(tmp_path),
            "tasks": [task],
        },
        "budget",
    )
    robots = {
        "A": {
            "pose": {"frame_id": "map_000", "pos": [2, 2, 0]},
            "speed": 1,
            "capabilities": ["device"],
        }
    }

    def exhausted(*args, **kwargs):
        raise SearchBudgetExceeded("grid search budget exhausted")

    monkeypatch.setattr(MapBundle, "path", exhausted)
    with pytest.raises(SearchBudgetExceeded, match="budget"):
        plan(m, robots)


def test_multirobot_cross_map_requires_root_collision_support(tmp_path):
    grid(tmp_path, "a")
    grid(tmp_path, "b")
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


def test_gray_unknown_and_inflation(tmp_path):
    path = grid(tmp_path)
    image = Image.open(tmp_path / "map_000.png")
    image.putpixel((10, 29), 100)
    image.putpixel((12, 29), 0)
    image.save(tmp_path / "map_000.png")
    g = Grid(path, radius=1.1)
    assert g.reason((10, 10))
    assert g.reason((11, 10))
    assert not g.reason((20, 20))


def test_no_corner_cutting(tmp_path):
    grid(tmp_path)
    image = Image.new("L", (40, 40), 0)
    image.putpixel((10, 29), 255)
    image.putpixel((11, 28), 255)
    image.save(tmp_path / "map_000.png")
    maps = MapBundle(tmp_path, radius=0)
    with pytest.raises(ValueError, match="no traversable"):
        maps.path(("map_000", 10, 10), ("map_000", 11, 11))


def test_multimap_portal_and_map_digest(tmp_path):
    grid(tmp_path, "a")
    grid(tmp_path, "b")
    (tmp_path / "map_relations.csv").write_text(
        "from_map,to_map,dx,dy\nROOT,a,0,0\na,b,0,0\n"
    )
    (tmp_path / "transition_points.csv").write_text(
        "transition_id,from_map,to_map,world_x,world_y,bidirectional\nt,a,b,5,5,true\n"
    )
    maps = MapBundle(tmp_path, radius=0)
    path, cost = maps.path(("a", 3, 3), ("b", 8, 8))
    assert any(a[0] != b[0] for a, b in zip(path, path[1:]))
    assert cost > 0
    before = maps.fingerprint
    image = Image.open(tmp_path / "b.png")
    image.putpixel((30, 30), 0)
    image.save(tmp_path / "b.png")
    assert MapBundle(tmp_path, radius=0).fingerprint != before


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
    (tmp_path / "map_relations.csv").write_text(
        "from_map,to_map,dx,dy\nROOT,a,0,0\na,b,0,0\n"
    )
    (tmp_path / "transition_points.csv").write_text(
        "transition_id,from_map,to_map,world_x,world_y,bidirectional\nt,a,b,5,5,true\n"
    )
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
