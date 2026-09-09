"""Exit regressions using isolated ROS nodes, never hardware or motor topics."""

import os
from pathlib import Path
import subprocess
import sys


def test_controller_exit_keeps_context_until_callbacks_finish():
    program = r"""
import os
import signal
import sys
import time
from threading import Event, current_thread, main_thread
import rclpy
from rclpy.node import Node
from bxi_example_py_elf3.control.ros_runtime import run_controller

mode = sys.argv[1]
state = {"callback_finished": False, "destroyed": False}
class Probe(Node):
    def __init__(self):
        super().__init__("controller_exit_probe", enable_rosout=False,
                         start_parameter_services=False)
        self.shutdown_requested = Event()
        self.timer = self.create_timer(0.01, self.callback)

    def callback(self):
        assert current_thread() is main_thread(), "controller must use a single-threaded executor"
        self.timer.cancel()
        try:
            if mode == "error":
                raise RuntimeError("normal callback failure")
            if mode == "safety":
                self.shutdown_requested.set()
            else:
                os.kill(os.getpid(), getattr(signal, mode))
            time.sleep(0.15)
            assert rclpy.ok(), "context closed while callback was active"
        finally:
            state["callback_finished"] = True

    def destroy_node(self):
        assert state["callback_finished"], "node destroyed before callback finished"
        assert rclpy.ok(), "context closed before node cleanup"
        state["destroyed"] = True
        return super().destroy_node()

try:
    run_controller(Probe)
except RuntimeError as exc:
    assert mode == "error" and str(exc) == "normal callback failure"
else:
    assert mode != "error", "normal callback exception was swallowed"
assert state["destroyed"] and not rclpy.ok()
print("PASS", mode)
"""
    env = dict(os.environ, ROS_DOMAIN_ID="199", ROS_LOCALHOST_ONLY="0",
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_STATIC_PEERS="",
               PYTHONDONTWRITEBYTECODE="1")
    package_root = str(Path(__file__).parents[1])
    env["PYTHONPATH"] = package_root + os.pathsep + env.get("PYTHONPATH", "")
    for mode in ("SIGINT", "SIGTERM", "safety", "error"):
        result = subprocess.run([sys.executable, "-B", "-c", program, mode],
                                env=env, capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stdout + result.stderr
        assert "PASS " + mode in result.stdout
        assert "context is invalid" not in result.stderr
        assert "exception was never retrieved" not in result.stderr


def test_shutdown_blocks_control_publish_and_reset():
    import ast
    from threading import Event, RLock
    from types import SimpleNamespace

    path = Path(__file__).parents[1] / "bxi_example_py_elf3" / "bxi_example_vibration.py"
    tree = ast.parse(path.read_text())
    controller = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                      and n.name == "VibrationTestNode")
    names = {"_timer_callback", "_publish_command", "_call_robot_reset"}
    methods = [n for n in controller.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = {"time": SimpleNamespace(monotonic=lambda: 0.0)}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
    stopped = Event()
    stopped.set()
    node = SimpleNamespace(shutdown_requested=stopped, state_lock=RLock(), safety_fault=False)
    assert namespace["_timer_callback"](node) is None
    assert namespace["_publish_command"](node, None, None, None) is False
    assert namespace["_call_robot_reset"](node, 1, False, 0.0) is False


if __name__ == "__main__":
    test_controller_exit_keeps_context_until_callbacks_finish()
    test_shutdown_blocks_control_publish_and_reset()
    print("PASS: SIGINT, SIGTERM, safety exit and callback error propagation")
