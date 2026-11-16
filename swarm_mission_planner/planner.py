"""Capability-aware greedy insertion with conservative completion dependencies.

No legacy planner imports. Route conflicts are serialized using actual sealed
route results, not estimated wall-clock traversal times. Cyclic dependencies
fail explicitly rather than exporting a plan that can deadlock.
"""

import copy
import math

from swarm_protocol.contracts import digest, finite

from .maps import MapBundle, SearchBudgetExceeded


def plan(mission, robots, *, radius=0.3, free_override=None):
    bundle = MapBundle(mission["map_package"], radius, free_override)
    routes, positions, loads, paths, failures = {}, {}, {}, {}, []
    for robot in sorted(robots):
        info = robots[robot]
        try:
            speed = finite(info["speed"], f"{robot}.speed")
            if speed <= 0:
                raise ValueError(f"{robot}: speed must be positive")
            positions[robot] = bundle.vertex(info["pose"], "规划起点无效：" + robot)
            routes[robot] = {
                "robot_id": robot,
                "checkpoints": [],
                "task_ids": [],
                "dependencies": [],
            }
            loads[robot], paths[robot] = 0.0, [positions[robot]]
        except ValueError as error:
            failures.append(str(error))
    if failures:
        raise ValueError("; ".join(failures))
    original_starts = copy.deepcopy(positions)
    unassignable = {}
    task_order = sorted(
        enumerate(mission["tasks"]),
        key=lambda pair: (not pair[1].get("high_priority", False), pair[0]),
    )
    for _, task in task_order:
        goal = bundle.vertex(task["location"], "任务点无效：" + task["id"])
        candidates = []
        eligible = []
        for robot, info in sorted(robots.items()):
            if not set(task["requirements"]).issubset(info["capabilities"]):
                continue
            eligible.append(robot)
            try:
                path, distance = bundle.path(positions[robot], goal)
                cost = distance / info["speed"] + task["service_seconds"]
                candidates.append((loads[robot] + cost, robot, path))
            except SearchBudgetExceeded:
                raise
            except ValueError:
                continue
        if not candidates:
            if not eligible:
                unassignable[task["id"]] = {
                    "requirements": task["requirements"],
                    "reason": "所需检测设备当前不可用",
                }
                continue
            raise ValueError(f"任务点 {task['id']} 无可达路径；有能力设备={eligible}")
        load, robot, path = min(candidates)
        route = routes[robot]
        # Keep heading changes and portals. Do not invent task IDs for turn points.
        for i, b in enumerate(path):
            portal = (i > 0 and path[i - 1][0] != b[0]) or (
                i + 1 < len(path) and b[0] != path[i + 1][0]
            )
            turn = 0 < i < len(path) - 1 and (
                b[1] - path[i - 1][1],
                b[2] - path[i - 1][2],
            ) != (path[i + 1][1] - b[1], path[i + 1][2] - b[2])
            if portal or turn:
                route["checkpoints"].append(
                    {
                        "type": "turn",
                        "location": {
                            "frame_id": b[0],
                            "pos": bundle.maps[b[0]].point(b[1:]),
                            "orientation": [0.0, 0.0, 0.0, 1.0],
                        },
                        "task_ids": [],
                    }
                )
        route["checkpoints"].append(
            {
                "type": "task",
                "location": copy.deepcopy(task["location"]),
                "task_ids": [task["id"]],
            }
        )
        route["task_ids"].append(task["id"])
        paths[robot].extend(path[1:])
        positions[robot], loads[robot] = goal, load
    active = sorted(r for r in routes if routes[r]["task_ids"])
    if (
        active
        and len(robots) > 1
        and len({v[0] for path in paths.values() for v in path}) > 1
    ):
        raise ValueError(
            "当前规划器未实现多设备跨地图的ROOT空间冲突校验，拒绝输出执行计划"
        )
    # Expand swept cells by two footprints when comparing different robots.
    occupied = {}
    for robot, path in paths.items():
        cells = set()
        for name, x, y in path:
            margin = max(1, math.ceil(2 * radius / bundle.maps[name].resolution))
            cells.update(
                (name, x + dx, y + dy)
                for dx in range(-margin, margin + 1)
                for dy in range(-margin, margin + 1)
                if dx * dx + dy * dy <= margin * margin
            )
        occupied[robot] = cells
    edges = set()
    for i, a in enumerate(active):
        for b in active[i + 1 :]:
            if not occupied[a].intersection(paths[b]):
                continue
            a_before_b = (
                original_starts[b] not in occupied[a]
                and positions[a] not in occupied[b]
            )
            b_before_a = (
                original_starts[a] not in occupied[b]
                and positions[b] not in occupied[a]
            )
            if a_before_b:
                edges.add((a, b))
            elif b_before_a:
                edges.add((b, a))
            else:
                raise ValueError(
                    f"路线安全顺序无法确定：{a} 与 {b} 的起终点占用存在冲突"
                )
    # An idle robot is a stationary obstacle, not an invisible agent.
    for a in active:
        for b in set(routes) - set(active):
            if original_starts[b] in occupied[a]:
                raise ValueError(f"路线 {a} 与无执行路线设备 {b} 的停车位置冲突")
    pending = set(active)
    while pending:
        ready = {
            r for r in pending if not any(b == r and a in pending for a, b in edges)
        }
        if not ready:
            raise ValueError("route dependency cycle; no safe serialized schedule")
        pending -= ready
    for a, b in sorted(edges):
        routes[b]["dependencies"].append(a)
    result = {
        "routes": list(routes.values()),
        "unassignable": unassignable,
        "map_digest": bundle.fingerprint,
        "scheduling": "sealed-result-dependencies",
    }
    result["plan_digest"] = digest(result)
    return result
