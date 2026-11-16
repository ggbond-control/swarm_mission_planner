"""One disposable planner process; no ROS or control interfaces in the child."""

import multiprocessing
import resource
import time

from swarm_protocol.contracts import MAX_MESSAGE_BYTES, canonical, decode, finite

from .planner import plan


def _calculate(connection, mission, robots, options):
    started = time.perf_counter()
    try:
        try:
            result = {"success": True, "plan": plan(mission, robots, **options)}
        except Exception as error:
            result = {"success": False, "error": str(error)}
        result["metrics"] = {
            "compute_seconds": time.perf_counter() - started,
            "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        }
        raw = canonical(result).encode()
        if len(raw) > MAX_MESSAGE_BYTES:
            raw = canonical(
                {"success": False, "error": "planner output exceeds message limit"}
            ).encode()
        connection.send_bytes(raw)
    finally:
        connection.close()


class PlanningWorker:
    """Single-flight planning with a parent-enforced wall-clock limit.

    Spawn avoids forking a running ROS executor. Timeout includes child startup,
    map loading and planning; the parent never waits for a Python search thread.
    This is a resource bound under a functioning OS, not a hard real-time claim.
    """

    def __init__(self, timeout=45.0):
        self.timeout = finite(timeout, "planner timeout")
        if self.timeout <= 0:
            raise ValueError("planner timeout must be positive")
        self.context = multiprocessing.get_context("spawn")
        self.process = None
        self.connection = None
        self.request_id = None
        self.deadline = 0.0

    @property
    def busy(self):
        return self.process is not None

    def submit(self, request_id, mission, robots, **options):
        if self.busy:
            return False
        parent, child = self.context.Pipe(duplex=False)
        process = self.context.Process(
            target=_calculate, args=(child, mission, robots, options), daemon=True
        )
        self.deadline = time.monotonic() + self.timeout
        try:
            process.start()
        except Exception:
            parent.close()
            child.close()
            raise
        child.close()
        self.process, self.connection, self.request_id = process, parent, request_id
        return True

    def poll(self):
        if not self.busy:
            return None
        result = None
        if time.monotonic() >= self.deadline:
            result = {
                "success": False,
                "error": "planner wall-clock timeout; worker terminated",
            }
        elif self.connection.poll():
            try:
                result = decode(self.connection.recv_bytes(MAX_MESSAGE_BYTES))
            except (EOFError, OSError, ValueError):
                result = {
                    "success": False,
                    "error": "planner worker returned no valid result",
                }
        elif not self.process.is_alive():
            result = {"success": False, "error": "planner worker exited without result"}
        if result is None:
            return None
        request_id = self.request_id
        self.close()
        return {"request_id": request_id, **result}

    def close(self):
        if self.process is None:
            return
        self.process.join(timeout=0.01)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=0.1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=0.1)
        if self.process.is_alive():
            raise RuntimeError(
                "planner worker could not be reaped; refusing replacement"
            )
        self.process.close()
        self.connection.close()
        self.process = self.connection = self.request_id = None
