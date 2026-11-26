import math

import numpy as np

from bxi_example_py_elf3.framework.mod_api import (
    EntryFrameProvider, JointLayout, MotorFrame, RobotControlState, RunningFrameProvider,
)

from .policy import DEFAULT_ARM_CLIP, DEFAULT_ARM_SPEED, GESTURES


# max_tilt_rad in the stand_xmz training run's env.yaml.
MAX_TILT_RAD = 0.65
MIN_UPRIGHT_Z = math.cos(MAX_TILT_RAD)
ZERO_TORQUE = "com.bxi.basic_actions/zero_torque"


class XmzStandState(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    def __init__(self, name, state_id, policy, *, play_arm_motion=True, arm_clip=DEFAULT_ARM_CLIP,
                 arm_speed=DEFAULT_ARM_SPEED):
        super().__init__(name, state_id, resources=(policy,))
        if arm_clip not in ("cycle", "training", "blend", *GESTURES):
            raise ValueError(f"Unknown XMZ arm clip: {arm_clip}")
        if not math.isfinite(arm_speed) or arm_speed <= 0:
            raise ValueError("XMZ arm_speed must be finite and positive")
        self.policy = policy
        self._play_arm_motion = play_arm_motion
        self._arm_clip = arm_clip
        self._arm_speed = arm_speed
        self._input_layout = None
        self._output_layout = None
        self._indices = None
        self._position = None
        self._velocity = None
        self._zero_frame = None
        self._faulted = False

    @staticmethod
    def _check_feedback(ctx):
        joints = ctx.robot_joints
        age_ns = ctx.ros_node.get_clock().now().nanoseconds - joints.timestamp_ns
        if not -5_000_000 <= age_ns <= 100_000_000:
            raise ValueError(f"XMZ joint feedback age is {age_ns / 1e6:.1f} ms")
        if not (np.isfinite(joints.position).all() and np.isfinite(joints.velocity).all()):
            raise ValueError("XMZ joint feedback contains nonfinite values")
        quat = np.asarray(ctx.current_quat_wxyz)
        omega = np.asarray(ctx.current_omega)
        if quat.shape != (4,) or omega.shape != (3,) or not np.isfinite(omega).all():
            raise ValueError("Invalid XMZ torso IMU feedback")
        norm_squared = float(quat @ quat)
        if not 0.9**2 <= norm_squared <= 1.1**2:
            raise ValueError("Invalid XMZ torso quaternion norm")
        up_z = 1.0 - 2.0 * (quat[1]**2 + quat[2]**2) / norm_squared
        if up_z < MIN_UPRIGHT_Z:
            raise ValueError(f"XMZ torso tilt exceeds {MAX_TILT_RAD} rad")

    def is_available(self, ctx):
        try:
            self._check_feedback(ctx)
        except (ValueError, RuntimeError):
            return False
        # The first request starts the resource load. The state machine checks
        # availability again when the resource is ready and its contract is known.
        return self.policy.status != "ready" or set(self.policy.get().names).issubset(
            ctx.robot_joints.layout.names
        )

    def on_prepare(self, ctx, from_state):
        self._check_feedback(ctx)
        policy = self.policy.get()
        joints = ctx.robot_joints
        self._input_layout = joints.layout
        self._output_layout = JointLayout.create(policy.names, label="XMZ standing")
        missing = self._output_layout.missing_from(joints.layout)
        if missing:
            raise ValueError(f"XMZ standing requires feedback for {', '.join(missing)}")
        self._indices = np.array([joints.layout.index(name) for name in policy.names], dtype=np.intp)
        self._position = np.empty(len(policy.names), dtype=joints.position.dtype)
        self._velocity = np.empty(len(policy.names), dtype=joints.velocity.dtype)
        # All five fields are zero across the full robot, including platform-only joints.
        zeros = np.zeros(ctx.robot_layout.dof_num, dtype=np.float32)
        self._zero_frame = MotorFrame.create(ctx.robot_layout, zeros, zeros, zeros)
        self._faulted = False
        policy.reset(play_arm_motion=self._play_arm_motion, arm_clip=self._arm_clip, arm_speed=self._arm_speed)

    def get_entry_frame(self, ctx):
        policy = self.policy.get()
        return self._motor_frame(ctx, policy.nominal, policy.kp, policy.kd, layout=self._output_layout)

    def sample_running_frame(self, ctx, dt, *, advance):
        if self._faulted:
            return self._zero_frame
        policy = self.policy.get()
        if advance:
            try:
                self._check_feedback(ctx)
                joints = ctx.robot_joints
                if joints.layout != self._input_layout:
                    raise ValueError("XMZ joint layout changed during standing")
                np.take(joints.position, self._indices, out=self._position)
                np.take(joints.velocity, self._indices, out=self._velocity)
                policy.step(self._position, self._velocity, ctx.current_quat_wxyz, ctx.current_omega)
            except Exception as exc:
                self._faulted = True
                self.logger.error(f"XMZ standing stopped: {exc}")
                return self._zero_frame
        return self._motor_frame(ctx, policy.target, policy.kp, policy.kd, layout=self._output_layout)

    def on_update(self, ctx, dt):
        self._apply_frame(ctx, self.sample_running_frame(ctx, dt, advance=True))
        if self._faulted:
            ctx.request_state(ZERO_TORQUE, trigger="xmz_stand_fault", transition="instant")
