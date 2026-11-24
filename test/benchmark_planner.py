"""Reproducible synthetic workload. Run explicitly; never contacts ROS/devices."""

import argparse
import json
import platform
import math
import subprocess
import time
from pathlib import Path

import yaml
from PIL import Image
from swarm_mission_planner.worker import PlanningWorker
from swarm_protocol.contracts import normalize_mission


def scenario(root, size, count, tasks):
    root.mkdir(parents=True, exist_ok=True)
    Image.new("L", (size, size), 255).save(root / "map_000.pgm")
    (root / "map_000.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map_000.pgm",
                "resolution": 0.1,
                "origin": [0, 0, 0],
                "free_thresh": 0.196,
                "occupied_thresh": 0.65,
            }
        )
    )
    ids = [f"BENCH-{i}" for i in range(count)]
    extent = size * 0.1
    robots = {
        robot: {
            "pose": {
                "frame_id": "map_000",
                "pos": [1, extent * (i + 1) / (count + 1), 0],
            },
            "speed": 1,
            "capabilities": ["device", f"lane.{i}"],
        }
        for i, robot in enumerate(ids)
    }
    points = []
    for number in range(tasks):
        lane, step = number % count, number // count
        points.append(
            {
                "id": f"p{number}",
                "location": {
                    "frame_id": "map_000",
                    "pos": [
                        extent
                        * (0.2 + 0.6 * (step + 1) / (1 + (tasks - 1) // count + 1)),
                        extent * (lane + 1) / (count + 1),
                        0,
                    ],
                },
                "tasks": [
                    {
                        "method": "wait",
                        "requirements": [f"lane.{lane}"],
                        "params": {"duration": 0},
                    }
                ],
            }
        )
    return normalize_mission(
        {"round_epoch": 1, "robot_ids": ids, "map_package": str(root), "tasks": points},
        "benchmark",
    ), robots



def legacy_config(mission, robots, radius):
    def position(pose):
        return {"map_id": pose["frame_id"], "local_xy": pose["pos"][:2]}

    return {
        "map": {"directory": mission["map_package"], "persistent_cache": False},
        "robots": [
            {"id": robot, "start": position(entry["pose"]),
             "nominal_speed_mps": entry["speed"], "capabilities": entry["capabilities"],
             "clearance_radius_m": radius, "footprint_radius_m": radius}
            for robot, entry in sorted(robots.items())
        ],
        "tasks": [
            {"id": task["id"], "location": position(task["location"]),
             "requirements": task["requirements"], "category": task.get("task_type", "inspection"),
             "service_seconds": math.ceil(task["service_seconds"]),
             "high_priority": task.get("high_priority", False)}
            for task in mission["tasks"]
        ],
    }


def legacy_result(binary, directory, mission, robots, timeout):
    directory.mkdir(parents=True, exist_ok=True)
    config = directory / "input.yaml"
    config.write_text(yaml.safe_dump(legacy_config(mission, robots, 0.1)))
    started = time.perf_counter()
    process = subprocess.run([str(binary), str(config), str(directory)], timeout=timeout,
                             capture_output=True, text=True, check=False)
    elapsed = time.perf_counter() - started
    if process.returncode:
        return {"success": False, "error": process.stderr.strip(), "cli_wall_seconds": elapsed}
    data = json.loads((directory / "plan.json").read_text())
    routes = {route["robot"]: [task for stop in route["stops"] for task in stop["tasks"]]
              for route in data["routes"]}
    metrics = {}
    for line in process.stdout.splitlines():
        name, _, value = line.strip().partition(": ")
        if name in {"timing_load_and_parse_seconds", "timing_planning_seconds", "peak_rss_mb",
                    "maximum_load_seconds", "total_load_seconds"}:
            metrics[name] = float(value)
    return {"success": True, "cli_wall_seconds": elapsed, "routes": routes, "metrics": metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sizes", nargs="+", type=int, default=[128, 256, 512])
    parser.add_argument("--robots", type=int, default=5)
    parser.add_argument("--tasks", type=int, default=20)
    parser.add_argument("--repeat", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=45)
    parser.add_argument("--legacy-cli", type=lambda value: Path(value).resolve())
    args = parser.parse_args()
    if min(args.sizes) < 32 or min(args.robots, args.tasks, args.repeat) < 1:
        parser.error("sizes >= 32 and positive counts required")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for size in args.sizes:
        mission, robots = scenario(
            output.parent / "fixtures" / str(size), size, args.robots, args.tasks
        )
        for attempt in range(args.repeat):
            worker = PlanningWorker(args.timeout)
            started = time.perf_counter()
            try:
                worker.submit(f"{size}-{attempt}", mission, robots, radius=0.1)
                result = None
                while result is None:
                    result = worker.poll()
                    time.sleep(0.01)
            finally:
                worker.close()
            row = {
                "size": size,
                "attempt": attempt + 1,
                "wall_seconds": time.perf_counter() - started,
                "success": result["success"],
                "metrics": result.get("metrics", {}),
                "error": result.get("error", ""),
            }
            if result["success"]:
                plan = result["plan"]
                row["routes"] = {route["robot_id"]: route["task_ids"] for route in plan["routes"]}
                row["statistics"] = plan["statistics"]
            if args.legacy_cli:
                try:
                    row["legacy"] = legacy_result(args.legacy_cli, output.parent / "legacy" / str(size) /
                                                  str(attempt + 1), mission, robots, args.timeout)
                except subprocess.TimeoutExpired:
                    row["legacy"] = {"success": False, "error": "legacy CLI exceeded timeout"}
                row["same_task_order"] = ("routes" in row and row["legacy"]["success"]
                                          and row["routes"] == row["legacy"]["routes"])
            rows.append(row)
            print(json.dumps(row), flush=True)
    output.write_text(
        json.dumps(
            {
                "machine": platform.machine(),
                "platform": platform.platform(),
                "python": platform.python_version(),
                "scenario": "open map, separated capability lanes; cold child per request",
                "robots": args.robots,
                "tasks": args.tasks,
                "timeout_seconds": args.timeout,
                "rows": rows,
                "scope": "synthetic local baseline, not board or real-map acceptance",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
