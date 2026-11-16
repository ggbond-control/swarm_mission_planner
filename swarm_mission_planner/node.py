"""ROS adapter; CPU-bound planning runs in a disposable child process."""

from rclpy.node import Node
from std_msgs.msg import String
from swarm_protocol.contracts import decode
from swarm_protocol.ros import main_node, publish

from .worker import PlanningWorker


class PlannerNode(Node):
    def __init__(self):
        super().__init__("swarm_mission_planner")
        self.declare_parameter("footprint_radius", 0.3)
        self.declare_parameter("free_thresh", 0.05)
        self.declare_parameter("planning_timeout", 45.0)
        self.output = self.create_publisher(String, "/swarm/planner/result", 10)
        self.worker = PlanningWorker(self.get_parameter("planning_timeout").value)
        self.create_subscription(String, "/swarm/planner/request", self.request, 10)
        self.create_timer(0.05, self.poll)

    def request(self, message):
        try:
            value = decode(message.data)
            request_id = value["request_id"]
            mission, robots = value["mission"], value["robots"]
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f"规划请求拒绝：{error}")
            return
        if self.worker.busy:
            if request_id == self.worker.request_id:
                return
            # A new request after Leader/round change supersedes a pure
            # calculation. It cannot leave an obsolete job blocking admission.
            self.worker.close()
        radius, free = (
            self.get_parameter("footprint_radius").value,
            self.get_parameter("free_thresh").value,
        )
        self.worker.submit(
            request_id,
            mission,
            robots,
            radius=radius,
            free_override=None if free < 0 else free,
        )

    def poll(self):
        result = self.worker.poll()
        if result is not None:
            publish(self.output, result)

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    main_node(PlannerNode, args)
