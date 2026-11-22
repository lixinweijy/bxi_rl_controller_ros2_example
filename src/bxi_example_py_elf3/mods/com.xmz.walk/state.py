import numpy as np
import time

from communication.msg import MotionCommands
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String

from bxi_example_py_elf3.framework.mod_api import JointLayout
from bxi_example_py_elf3.framework.mod_api import RobotControlState
from bxi_example_py_elf3.framework.mod_api.transition import EntryFrameProvider, RunningFrameProvider


POLICY_SWITCH_SECONDS = .1


class XmzWalkState(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    def __init__(self, name, state_id, policy, moving_policy):
        super().__init__(name, state_id, resources=(policy, moving_policy))
        self.policy = policy
        self.moving_policy = moving_policy
        self.active_policy = None
        self.layout = None
        self.target = None
        self.kp = self.kd = None
        self.switch_from = None
        self.switch_elapsed = 0.0
        self.command_received = 0.0
        self.feedback_indices = None
        self.trace_publishers = {}
        self.active_publisher = None

    def on_bind(self, ctx):
        self.subscription = ctx.ros_node.create_subscription(
            MotionCommands, "/motion_commands", self.on_command, qos_profile_sensor_data)
        self.trace_publishers = {
            name: ctx.ros_node.create_publisher(
                Float32MultiArray, f"/xmz/policy/{name}", qos_profile_sensor_data)
            for name in ("observation", "action_raw", "action_clipped", "target")}
        self.active_publisher = ctx.ros_node.create_publisher(
            String, "/xmz/policy/active", qos_profile_sensor_data)

    def on_command(self, msg):
        self.command_received = time.monotonic()

    def on_unbind(self, ctx):
        ctx.ros_node.destroy_subscription(self.subscription)
        for publisher in self.trace_publishers.values():
            ctx.ros_node.destroy_publisher(publisher)
        ctx.ros_node.destroy_publisher(self.active_publisher)
        self.trace_publishers.clear()

    def get_cmd_vel(self, ctx):
        command = super().get_cmd_vel(ctx)
        if time.monotonic() - self.command_received > 0.5:
            command[:] = 0
        policy = self.moving_policy.get() if np.any(np.abs(command) > 1e-6) else self.policy.get()
        return self._publish_cmd_vel(ctx, np.clip(command, policy.ranges[:, 0], policy.ranges[:, 1]))

    def _observe(self, ctx, command):
        joints = ctx.robot_joints
        age = ctx.ros_node.get_clock().now().nanoseconds - joints.timestamp_ns
        if not -5_000_000 <= age <= 100_000_000:
            raise ValueError(f"xmz requires fresh complete joint feedback within 100 ms (age={age / 1e6:.3f} ms)")
        policy = self.active_policy
        return policy.observe(policy.names, joints.position[self.feedback_indices],
                              joints.velocity[self.feedback_indices], ctx.current_quat_wxyz,
                              ctx.current_omega, command)

    def on_prepare(self, ctx, from_state):
        policy = self.policy.get()
        moving = self.moving_policy.get()
        if policy.names != moving.names:
            raise ValueError("XMZ idle/moving policies require the same joint order")
        for p in (policy, moving):
            p.reset()
        self.active_policy = policy
        self.switch_from = None
        self.switch_elapsed = 0.0
        self.layout = JointLayout.create(policy.names, label="xmz ELF4 policy")
        names = ctx.robot_joints.layout.names
        if not set(policy.names).issubset(names):
            raise ValueError("XMZ requires all 29 policy joints in real feedback")
        self.feedback_indices = np.array([names.index(n) for n in policy.names])
        # Validate all enabled joints before accepting the transition.
        self._observe(ctx, np.zeros(3))
        self.target = policy.default.copy()
        self.kp, self.kd = np.array(policy.kp), np.array(policy.kd)
        policy.reset()

    def on_enter(self, ctx):
        self.logger.info(f"xmz ELF4 dual policy active: {ctx.robot_layout.dof_num} real joints, 50 Hz CPU ONNX; idle=policy.onnx, moving=policy_1.onnx")

    def get_entry_frame(self, ctx):
        policy = self.policy.get()
        return self._motor_frame(ctx, policy.default, policy.kp, policy.kd, layout=self.layout)

    def sample_running_frame(self, ctx, dt, *, advance):
        if advance:
            command = self.get_cmd_vel(ctx)
            moving = bool(np.any(np.abs(command) > 1e-6))
            policy = self.moving_policy.get() if moving else self.policy.get()
            if policy is not self.active_policy:
                self.switch_from = (self.target.copy(), self.kp.copy(), self.kd.copy())
                self.switch_elapsed = 0.0
                policy.reset()
                # Translate the last issued target into this model's action units.
                policy.previous[:] = np.clip((self.target - policy.default) / policy.action_scale,
                                             -policy.cfg["clip_actions"], policy.cfg["clip_actions"])
                policy.previous[policy.disabled] = 0
                self.active_policy = policy
                self.logger.info(f"xmz policy switch: {'policy_1.onnx' if moving else 'policy.onnx'} ({POLICY_SWITCH_SECONDS:.2f} s blend)")
            observation = self._observe(ctx, command)
            target = policy.infer(observation)
            kp, kd = np.asarray(policy.kp), np.asarray(policy.kd)
            if self.switch_from is not None:
                progress = min(self.switch_elapsed / POLICY_SWITCH_SECONDS, 1.0)
                alpha = progress * progress * (3.0 - 2.0 * progress)
                target, kp, kd = [old + alpha * (new - old) for old, new in
                                  zip(self.switch_from, (target, kp, kd))]
                self.switch_elapsed += dt
                if progress >= 1.0:
                    self.switch_from = None
            self.target, self.kp, self.kd = target, kp, kd
            values = {"observation": observation.ravel(), "action_raw": policy.raw_action,
                      "action_clipped": policy.previous, "target": self.target}
            try:
                for name, publisher in self.trace_publishers.items():
                    if publisher.get_subscription_count():
                        publisher.publish(Float32MultiArray(data=values[name].tolist()))
                if self.active_publisher is not None and self.active_publisher.get_subscription_count():
                    self.active_publisher.publish(String(data="policy_1.onnx" if moving else "policy.onnx"))
            except Exception as exc:
                # Recording must not interrupt the motor command path.
                self.logger.warning(f"policy telemetry publish failed: {exc}")
        return self._motor_frame(ctx, self.target, self.kp, self.kd, layout=self.layout)

    def on_update(self, ctx, dt):
        self._apply_frame(ctx, self.sample_running_frame(ctx, dt, advance=True))
