"""Reproducible synthetic workload. Run explicitly; never contacts ROS/devices."""

import argparse
import json
import platform
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sizes", nargs="+", type=int, default=[128, 256, 512])
    parser.add_argument("--robots", type=int, default=5)
    parser.add_argument("--tasks", type=int, default=20)
    parser.add_argument("--repeat", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=15)
    args = parser.parse_args()
    if min(args.sizes) < 32 or min(args.robots, args.tasks, args.repeat) < 1:
        parser.error("sizes >= 32 and positive counts required")
    output = args.output.resolve()
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
            rows.append(
                {
                    "size": size,
                    "attempt": attempt + 1,
                    "wall_seconds": time.perf_counter() - started,
                    "success": result["success"],
                    "metrics": result.get("metrics", {}),
                    "error": result.get("error", ""),
                }
            )
            print(json.dumps(rows[-1]), flush=True)
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
