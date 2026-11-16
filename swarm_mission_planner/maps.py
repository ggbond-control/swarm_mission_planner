"""Nav2 maps, yaw-aware coordinates, footprint inflation and explicit map portals."""

from __future__ import annotations

import csv
import hashlib
import heapq
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from swarm_protocol.contracts import digest, finite


class SearchBudgetExceeded(ValueError):
    """Resource exhaustion must not be reported as physical unreachability."""


class Grid:
    def __init__(self, path, radius=0.0, free_override=None):
        self.path = Path(path)
        meta = yaml.safe_load(self.path.read_text())
        self.name = self.path.stem
        self.resolution = finite(meta["resolution"], "resolution")
        if self.resolution <= 0:
            raise ValueError("resolution must be positive")
        self.origin = [finite(x, "origin") for x in meta["origin"]]
        if len(self.origin) != 3:
            raise ValueError("map origin must have three values")
        image_path = self.path.parent / meta["image"]
        image = Image.open(image_path).convert("RGBA")
        rgba = np.asarray(image)
        pixels = rgba[:, :, :3].mean(axis=2) / 255.0
        occupancy = pixels if meta.get("negate", 0) else 1.0 - pixels
        free = finite(
            meta.get("free_thresh", 0.196) if free_override is None else free_override,
            "free_thresh",
        )
        occupied = finite(meta.get("occupied_thresh", 0.65), "occupied_thresh")
        if not 0 <= free <= occupied <= 1:
            raise ValueError("invalid map occupancy thresholds")
        mode = meta.get("mode", "trinary")
        if mode not in {"trinary", "scale", "raw"}:
            raise ValueError("unsupported map mode")
        if mode == "raw":
            # Raw Nav2 occupancy values: 0..100, 255 unknown; free is a probability.
            raw = rgba[:, :, 0]
            allowed = (raw <= free * 100) & (raw <= 100)
        else:
            allowed = occupancy <= free
        allowed &= rgba[:, :, 3] == 255
        # Image rows run downward; the planning grid runs upward from origin.
        self.free = np.flipud(allowed).copy()
        if radius < 0 or not math.isfinite(radius):
            raise ValueError("invalid footprint radius")
        if radius:
            padded = np.pad(self.free, 1, constant_values=False)
            clearance = distance_transform_edt(padded)[1:-1, 1:-1] * self.resolution
            self.free &= clearance > radius
        self.height, self.width = self.free.shape
        self.fingerprint = digest(
            {
                "metadata": meta,
                "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                "radius": radius,
                "free": free,
            }
        )

    def cell(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        return math.floor((c * dx + s * dy) / self.resolution), math.floor(
            (-s * dx + c * dy) / self.resolution
        )

    def point(self, cell):
        x, y = (cell[0] + 0.5) * self.resolution, (cell[1] + 0.5) * self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return [ox + c * x - s * y, oy + s * x + c * y, 0.0]

    def reason(self, cell):
        x, y = cell
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return "超出地图范围"
        if not self.free[y, x]:
            return "落入不可通行栅格（障碍、未知或净空不足）"
        return ""

    def neighbors(self, cell):
        x, y = cell
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ):
            target = (x + dx, y + dy)
            if self.reason(target):
                continue
            if dx and dy and (self.reason((x + dx, y)) or self.reason((x, y + dy))):
                continue
            yield target, math.hypot(dx, dy) * self.resolution


class MapBundle:
    def __init__(self, directory, radius=0.3, free_override=None):
        root = Path(directory)
        self.maps = {}
        for path in sorted(root.glob("*.yaml")):
            value = yaml.safe_load(path.read_text())
            if isinstance(value, dict) and "image" in value:
                self.maps[path.stem] = Grid(path, radius, free_override)
        if not self.maps:
            raise ValueError(f"no Nav2 maps found: {root}")
        self.portals = {}
        offsets = {"ROOT": (0.0, 0.0)}
        relations = root / "map_relations.csv"
        if relations.exists():
            rows = list(csv.DictReader(relations.open()))
            for _ in range(len(rows) + 1):
                for row in rows:
                    a, b = row["from_map"], row["to_map"]
                    delta = float(row["dx"]), float(row["dy"])
                    if a in offsets:
                        candidate = tuple(offsets[a][i] + delta[i] for i in (0, 1))
                        if b in offsets and math.dist(candidate, offsets[b]) > 1e-5:
                            raise ValueError("inconsistent map relations")
                        offsets[b] = candidate
                    elif b in offsets:
                        offsets[a] = tuple(offsets[b][i] - delta[i] for i in (0, 1))
        transitions = root / "transition_points.csv"
        if transitions.exists():
            for row in csv.DictReader(transitions.open()):
                a, b = row["from_map"], row["to_map"]
                if (
                    a not in self.maps
                    or b not in self.maps
                    or a not in offsets
                    or b not in offsets
                ):
                    raise ValueError(
                        "transition map is missing or has no ROOT transform"
                    )
                world = float(row["world_x"]), float(row["world_y"])
                ca = self.maps[a].cell(*(world[i] - offsets[a][i] for i in (0, 1)))
                cb = self.maps[b].cell(*(world[i] - offsets[b][i] for i in (0, 1)))
                self.validate(a, ca, "transition " + row["transition_id"])
                self.validate(b, cb, "transition " + row["transition_id"])
                self.portals.setdefault((a, *ca), []).append(((b, *cb), 1.0))
                if row.get("bidirectional", "").lower() in {"true", "1", "yes"}:
                    self.portals.setdefault((b, *cb), []).append(((a, *ca), 1.0))
        self.fingerprint = digest(
            {
                "maps": {k: v.fingerprint for k, v in self.maps.items()},
                "relations": relations.read_text() if relations.exists() else "",
                "transitions": transitions.read_text() if transitions.exists() else "",
            }
        )

    def validate(self, name, cell, who):
        if name not in self.maps:
            raise ValueError(f"{who}：未知地图 {name}")
        grid = self.maps[name]
        reason = grid.reason(cell)
        if reason:
            raise ValueError(
                f"{who}，地图 {name}，栅格 {cell}，地图大小 {grid.width}×{grid.height}；原因：{reason}"
            )

    def vertex(self, location, who):
        name = location["frame_id"]
        if name not in self.maps:
            raise ValueError(f"{who}：未知地图 {name}")
        pos = location["pos"]
        cell = self.maps[name].cell(pos[0], pos[1])
        self.validate(name, cell, f"{who}，位置 ({pos[0]:.3f}, {pos[1]:.3f})m")
        return (name, *cell)

    def path(self, start, finish, max_expansions=500000):
        queue, costs, previous = [(0.0, start)], {start: 0.0}, {}
        expanded = 0
        while queue:
            cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            if current == finish:
                path = [current]
                while path[-1] != start:
                    path.append(previous[path[-1]])
                return list(reversed(path)), cost
            expanded += 1
            if expanded > max_expansions:
                raise SearchBudgetExceeded("grid search budget exhausted")
            name, x, y = current
            neighbors = [
                ((name, *cell), step)
                for cell, step in self.maps[name].neighbors((x, y))
            ]
            neighbors.extend(self.portals.get(current, []))
            for neighbor, step in neighbors:
                proposed = cost + step
                if proposed < costs.get(neighbor, float("inf")):
                    costs[neighbor], previous[neighbor] = proposed, current
                    heapq.heappush(queue, (proposed, neighbor))
        raise ValueError(f"no traversable path: {start} -> {finish}")
