import numpy as np
import time

from communication.msg import MotionCommands
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from bxi_example_py_elf3.framework.mod_api import JointLayout
from bxi_example_py_elf3.framework.mod_api import RobotControlState
from bxi_example_py_elf3.framework.mod_api.transition import EntryFrameProvider, RunningFrameProvider


class XmzWalkState(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    def __init__(self, name, state_id, policy):
        super().__init__(name, state_id, resources=(policy,))
        self.policy = policy
        self.layout = None
        self.target = None
        self.command_received = 0.0
        self.feedback_indices = None
        self.trace_publishers = {}

    def on_bind(self, ctx):
        self.subscription = ctx.ros_node.create_subscription(
            MotionCommands, "/motion_commands", self.on_command, qos_profile_sensor_data)
        self.trace_publishers = {
            name: ctx.ros_node.create_publisher(
                Float32MultiArray, f"/xmz/policy/{name}", qos_profile_sensor_data)
            for name in ("observation", "action_raw", "action_clipped", "target")}

    def on_command(self, msg):
        self.command_received = time.monotonic()

    def on_unbind(self, ctx):
        ctx.ros_node.destroy_subscription(self.subscription)
        for publisher in self.trace_publishers.values():
            ctx.ros_node.destroy_publisher(publisher)
        self.trace_publishers.clear()

    def get_cmd_vel(self, ctx):
        command = super().get_cmd_vel(ctx)
        if time.monotonic() - self.command_received > 0.5:
            command[:] = 0
        return command

    def _observe(self, ctx, command):
        joints = ctx.robot_joints
        age = ctx.ros_node.get_clock().now().nanoseconds - joints.timestamp_ns
        if not -5_000_000 <= age <= 100_000_000:
            raise ValueError(f"xmz requires fresh complete joint feedback within 100 ms (age={age / 1e6:.3f} ms)")
        policy = self.policy.get()
        return policy.observe(policy.names, joints.position[self.feedback_indices],
                              joints.velocity[self.feedback_indices], ctx.current_quat_wxyz,
                              ctx.current_omega, command)

    def on_prepare(self, ctx, from_state):
        policy = self.policy.get()
        policy.reset()
        self.layout = JointLayout.create(policy.names, label="xmz ELF4 policy")
        names = ctx.robot_joints.layout.names
        if not set(policy.names).issubset(names):
            raise ValueError("XMZ requires all 29 policy joints in real feedback")
        self.feedback_indices = np.array([names.index(n) for n in policy.names])
        # Validate all enabled joints before accepting the transition.
        self._observe(ctx, np.zeros(3))
        self.target = policy.default.copy()
        policy.reset()

    def on_enter(self, ctx):
        self.logger.info(f"xmz ELF4 walking active: {ctx.robot_layout.dof_num} real joints, 50 Hz CPU ONNX")

    def get_entry_frame(self, ctx):
        policy = self.policy.get()
        return self._motor_frame(ctx, policy.default, policy.kp, policy.kd, layout=self.layout)

    def sample_running_frame(self, ctx, dt, *, advance):
        policy = self.policy.get()
        if advance:
            observation = self._observe(ctx, self.get_cmd_vel(ctx))
            self.target = policy.infer(observation)
            values = {"observation": observation.ravel(), "action_raw": policy.raw_action,
                      "action_clipped": policy.previous, "target": self.target}
            try:
                for name, publisher in self.trace_publishers.items():
                    if publisher.get_subscription_count():
                        publisher.publish(Float32MultiArray(data=values[name].tolist()))
            except Exception as exc:
                # Recording must not interrupt the motor command path.
                self.logger.warning(f"policy telemetry publish failed: {exc}")
        return self._motor_frame(ctx, self.target, policy.kp, policy.kd, layout=self.layout)

    def on_update(self, ctx, dt):
        self._apply_frame(ctx, self.sample_running_frame(ctx, dt, advance=True))
