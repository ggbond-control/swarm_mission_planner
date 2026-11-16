from setuptools import setup

name = "swarm_mission_planner"
setup(
    name=name,
    version="0.1.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml", "README.md", "TEST_LOG.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "swarm_mission_planner_node = swarm_mission_planner.node:main",
            "swarm_plan = swarm_mission_planner.cli:main",
        ]
    },
)
