"""Publish fresh body-only or full-body feedback without filling missing joints."""
import time

import numpy as np
import rclpy
from communication.msg import ActuatorStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS


class FeedbackMerger:
    def __init__(self, names, *, arms=True, head=False):
        names = tuple(names)
        head_names = ("head_z_joint", "head_y_joint") if head else ()
        self.names = (names if arms else names[:15]) + head_names
        self.groups = [frozenset(names[:15] + head_names)]
        if arms:
            self.groups.extend((frozenset(names[15:22]), frozenset(names[22:])))
        self.groups = [group for group in self.groups if group]
        self.sources = {}
        self.last_stamp = 0

    def receive(self, msg, now_ns, steady):
        names = msg.name
        group = frozenset(names)
        if group not in self.groups:
            return False
        index = self.groups.index(group)
        self.sources.pop(index, None)
        fields = (msg.position, msg.velocity, msg.effort, msg.driver_temperature, msg.motor_temperature)
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (len(names) != len(group) or any(len(values) != len(names) for values in fields)
                or not all(np.isfinite(values).all() for values in fields[:3])
                or not -5_000_000 <= now_ns - stamp <= 100_000_000):
            return False
        self.sources[index] = (msg, stamp, steady)
        return True

    def sample(self, now_ns, steady):
        if len(self.sources) != len(self.groups):
            return None
        values = list(self.sources.values())
        stamps = [stamp for _, stamp, _ in values]
        oldest = min(stamps)
        if (oldest <= self.last_stamp or max(stamps) - oldest > 30_000_000
                or any(steady - received > 0.1 or not -5_000_000 <= now_ns - stamp <= 100_000_000
                       for _, stamp, received in values)):
            return None
        result = ActuatorStates()
        result.header.frame_id = "elf3"
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(oldest, 10**9)
        fields = ("position", "velocity", "effort", "driver_temperature", "motor_temperature")
        lookup = {name: (msg, i) for msg, _, _ in values for i, name in enumerate(msg.name)}
        result.name = list(self.names)
        for field in fields:
            setattr(result, field, [getattr(lookup[name][0], field)[lookup[name][1]] for name in self.names])
        self.last_stamp = oldest
        return result


class FeedbackNode(Node):
    def __init__(self):
        super().__init__("xmz_feedback_merger")
        arms = self.declare_parameter("arms", True).value
        head = self.declare_parameter("head", False).value
        self.merger = FeedbackMerger(ELF3_POLICY_JOINTS.names, arms=arms, head=head)
        self.get_logger().info(f"Feedback requires {len(self.merger.names)} real joints (arms={arms}, head={head})")
        self.publisher = self.create_publisher(ActuatorStates, "/xmz/actuator_states", qos_profile_sensor_data)
        self.create_subscription(ActuatorStates, "/hardware/actuator_states", self.receive, qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)

    def receive(self, msg):
        if not self.merger.receive(msg, self.get_clock().now().nanoseconds, time.monotonic()):
            self.get_logger().error("Rejected incomplete/invalid body or arm feedback", throttle_duration_sec=5)

    def tick(self):
        msg = self.merger.sample(self.get_clock().now().nanoseconds, time.monotonic())
        if msg is not None:
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
