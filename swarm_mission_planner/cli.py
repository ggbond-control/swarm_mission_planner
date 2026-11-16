import argparse
import json

from swarm_protocol.contracts import normalize_mission

from .planner import plan


def main():
    parser = argparse.ArgumentParser(
        description="Independent swarm grid planner (no robot motion)"
    )
    parser.add_argument("input", help="JSON containing mission, mission_id and robots")
    parser.add_argument("--radius", type=float, default=0.3)
    args = parser.parse_args()
    with open(args.input) as stream:
        value = json.load(stream)
    print(
        json.dumps(
            plan(
                normalize_mission(value["mission"], value["mission_id"]),
                value["robots"],
                radius=args.radius,
            ),
            ensure_ascii=False,
            indent=2,
        )
    )
