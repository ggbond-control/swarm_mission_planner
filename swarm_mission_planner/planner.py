"""Adapter for the independently installed C++ planner."""

import hashlib
import os
from pathlib import Path
import subprocess

import yaml
from swarm_protocol.contracts import MAX_MESSAGE_BYTES, canonical, decode, digest


def _map_identity(directory, radius, free_override):
    root = Path(directory)
    files = {}
    for path in sorted(root.glob("*.yaml")):
        raw = path.read_bytes()
        metadata = yaml.safe_load(raw)
        if not isinstance(metadata, dict) or not metadata.get("image"):
            raise ValueError(f"invalid Nav2 map: {path.name}")
        with (path.parent / metadata["image"]).open("rb") as image:
            image_hash = hashlib.file_digest(image, "sha256").hexdigest()
        files[path.name] = [hashlib.sha256(raw).hexdigest(), image_hash]
    if not files:
        raise ValueError(f"no Nav2 maps found: {root}")
    for name in ("map_relations.csv", "transition_points.csv"):
        path = root / name
        if path.is_file():
            files[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest({"files": files, "radius": radius, "free_thresh": free_override,
                   "planner": "native-grid"})


def calculate(directory, *, radius=0.3, free_override=None, mission=None, robots=None,
              timeout=45.0):
    # Compare the exact inputs used by preflight and planning, independent of
    # the absolute path to the map directory on each device.
    radius = float(radius)
    free_override = None if free_override is None else float(free_override)
    identity = _map_identity(directory, radius, free_override)
    request = {"map_package": directory, "radius": radius}
    if free_override is not None:
        request["free_thresh"] = free_override
    if mission is None:
        request["preflight"] = True
    else:
        request.update(mission=mission, robots=robots)
    executable = os.environ.get("SWARM_PLANNER_NATIVE")
    if not executable:
        from ament_index_python.packages import get_package_prefix

        executable = (Path(get_package_prefix("swarm_mission_planner")) / "lib"
                      / "swarm_mission_planner" / "swarm_planner_native")
    try:
        process = subprocess.run([executable], input=canonical(request).encode(),
                                 capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as error:
        raise ValueError("planner wall-clock timeout") from error
    if process.returncode:
        raise ValueError(f"native planner exited with status {process.returncode}")
    if len(process.stdout) > MAX_MESSAGE_BYTES:
        raise ValueError("planner output exceeds message limit")
    result = decode(process.stdout)
    if result["success"]:
        if identity != _map_identity(directory, radius, free_override):
            raise ValueError("map inputs changed during planning")
        if mission is None:
            result["map_digest"] = identity
        else:
            result["plan"]["map_digest"] = identity
            result["plan"]["plan_digest"] = digest(result["plan"])
    return result


def map_fingerprint(directory, radius, free_override):
    result = calculate(directory, radius=radius, free_override=free_override)
    if not result["success"]:
        raise ValueError(result["error"])
    return result["map_digest"]


def plan(mission, robots, **options):
    result = calculate(mission["map_package"], mission=mission, robots=robots, **options)
    if not result["success"]:
        raise ValueError(result["error"])
    return result["plan"]
