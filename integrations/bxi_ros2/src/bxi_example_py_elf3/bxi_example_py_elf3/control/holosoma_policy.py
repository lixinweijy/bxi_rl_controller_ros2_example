"""Run Holosoma locomotion inside the existing single ROS safety controller.

Holosoma owns observations, inference, action scaling and model PD gains. This
module only maps named ELF3 feedback/targets to its ELF4 canonical joint order.
"""

from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from .elf3 import JOINT_NAMES, JOINT_POSITION_MAX, JOINT_POSITION_MIN


class _LocalInterface:
    """In-process Holosoma transport; never publishes ROS motor commands."""

    def __init__(self, robot_config):
        self.robot_config = robot_config
        self.state = None
        self.command = None
        self.kp_level = 1.0
        self.kd_level = 1.0

    def update_config(self, robot_config):
        self.robot_config = robot_config

    def get_low_state(self):
        return self.state

    def send_low_command(self, cmd_q, cmd_dq, cmd_tau, dof_pos_latest=None,
                         kp_override=None, kd_override=None):
        del dof_pos_latest
        self.command = (
            np.asarray(cmd_q, dtype=np.float64).copy(),
            np.asarray(kp_override if kp_override is not None else self.robot_config.motor_kp,
                       dtype=np.float64) * self.kp_level,
            np.asarray(kd_override if kd_override is not None else self.robot_config.motor_kd,
                       dtype=np.float64) * self.kd_level,
        )


class HolosomaWalkingPolicy:
    """One Holosoma ELF4 policy step with ELF3 named feedback and targets."""

    def __init__(self, model_path):
        if not model_path or not Path(model_path).is_file():
            raise ValueError("holosoma_model_path must name an existing ELF4 ONNX model")
        try:
            from holosoma_inference.policies.locomotion import LocomotionPolicy
            from .elf4_preset import elf4_29dof_loco
        except ImportError as exc:
            raise RuntimeError("install holosoma-inference from this Holosoma checkout") from exc

        class _EmbeddedLocomotionPolicy(LocomotionPolicy):
            def _init_sdk_components(self):
                self.sdk_type = "ros_embedded"

            def _init_communication_components(self):
                self.interface = _LocalInterface(self.robot_config)

            def _init_input_handlers(self):
                # The ROS controller owns the loop and remote input.
                from loguru import logger
                self.logger = logger
                self.use_joystick = False
                self.use_keyboard = False

        config = replace(
            elf4_29dof_loco,
            task=replace(elf4_29dof_loco.task, model_path=str(model_path)),
        )
        self.policy = _EmbeddedLocomotionPolicy(config)
        names = tuple(self.policy.dof_names)
        if len(names) != 29 or set(names) != set(JOINT_NAMES):
            raise ValueError("Holosoma ELF4 joints do not match ELF3 hardware joints")
        self._to_holosoma = np.array([JOINT_NAMES.index(name) for name in names])
        self._to_elf3 = np.array([names.index(name) for name in JOINT_NAMES])
        self.default_joint_pos = self.policy.default_dof_angles[self._to_elf3].copy()
        self.joint_stiffness = np.asarray(self.policy.robot_config.motor_kp)[self._to_elf3]
        self.joint_damping = np.asarray(self.policy.robot_config.motor_kd)[self._to_elf3]
        if any(array.shape != (29,) or not np.all(np.isfinite(array)) or np.any(array <= 0)
               for array in (self.joint_stiffness, self.joint_damping)):
            raise ValueError("ELF4 ONNX needs finite positive 29-joint PD gains")
        if np.any(self.default_joint_pos < JOINT_POSITION_MIN) or np.any(self.default_joint_pos > JOINT_POSITION_MAX):
            raise ValueError("Holosoma default pose exceeds ELF3 software joint limits")
        self.command_limits = self._read_command_limits(model_path)
        self.last_observation = None
        self.last_applied_velocity = np.zeros(3)
        self.reset()

    def _read_command_limits(self, model_path):
        import onnx
        metadata = {item.key: json.loads(item.value)
                    for item in onnx.load(model_path).metadata_props}
        if tuple(metadata.get("joint_order", ())) != tuple(self.policy.dof_names):
            raise ValueError("ELF4 ONNX joint_order differs from the Holosoma preset")
        session = self.policy.onnx_policy_session
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != "actor_obs" or inputs[0].shape[-1] != 100:
            raise ValueError("ELF4 ONNX needs one 100-dimensional actor_obs input")
        if not outputs or outputs[0].shape[-1] != 29:
            raise ValueError("ELF4 ONNX needs a 29-dimensional action output")
        default_positions = np.asarray(metadata.get("default_joint_positions", []), dtype=np.float64)
        if default_positions.shape != (29,) or not np.allclose(
            default_positions, self.policy.default_dof_angles, atol=1e-5
        ):
            raise ValueError("ELF4 ONNX default joint positions differ from Holosoma preset")
        if tuple(metadata.get("actor_observation_terms", ())) != tuple(
            self.policy.obs_terms_sorted["actor_obs"]
        ) or metadata.get("observation_normalization", {}).get("embedded_in_onnx") is not True:
            raise ValueError("ELF4 ONNX observation contract differs from Holosoma preset")
        if not np.isclose(metadata.get("policy_dt", np.nan), 1.0 / self.policy.rl_rate, atol=1e-6):
            raise ValueError("ELF4 ONNX policy_dt differs from Holosoma RL rate")
        ranges = metadata.get("command_ranges", {})
        limits = []
        for key in ("lin_vel_x", "lin_vel_y", "ang_vel_yaw"):
            pair = np.asarray(ranges.get(key), dtype=np.float64)
            if pair.shape != (2,) or not np.all(np.isfinite(pair)) or pair[0] >= pair[1]:
                raise ValueError(f"ELF4 ONNX needs a valid {key} command range")
            limits.append(pair)
        raw_scale = metadata.get("action_scale")
        action_scale = np.asarray(raw_scale, dtype=np.float64) if raw_scale is not None else np.array([])
        if action_scale.shape != (29,) or not np.all(np.isfinite(action_scale)) or not np.allclose(
            action_scale, self.policy.policy_action_scale, atol=1e-6
        ):
            raise ValueError("ELF4 ONNX action_scale differs from the Holosoma preset")
        return np.asarray(limits)

    def reset(self):
        p = self.policy
        p.last_policy_action.fill(0.0)
        p.scaled_policy_action.fill(0.0)
        for group in p.obs_history_buffers.values():
            for history in group.values():
                history.clear()
        p.phase = np.array([[0.0, np.pi]])
        p.is_standing = False
        p.use_policy_action = True
        p.get_ready_state = False

    def step(self, q, dq, quat_xyzw, omega, command):
        p = self.policy
        q = np.asarray(q, dtype=np.float64)
        dq = np.asarray(dq, dtype=np.float64)
        quat = np.asarray(quat_xyzw, dtype=np.float64)
        omega = np.asarray(omega, dtype=np.float64)
        command = np.asarray(command, dtype=np.float64)
        if any(array.shape != shape or not np.all(np.isfinite(array)) for array, shape in (
            (q, (29,)), (dq, (29,)), (quat, (4,)),
            (omega, (3,)), (command, (3,)),
        )):
            raise ValueError("Holosoma feedback/velocity must have finite ELF3 dimensions")
        norm = np.linalg.norm(quat)
        if norm < 1e-6:
            raise ValueError("invalid IMU quaternion")
        applied = np.clip(command, self.command_limits[:, 0], self.command_limits[:, 1])
        self.last_applied_velocity[:] = applied
        p.stand_command[0, 0] = int(np.linalg.norm(applied) > 1e-3)
        p.lin_vel_command[0] = applied[:2]
        p.ang_vel_command[0, 0] = applied[2]
        p.update_phase_time()
        state = np.concatenate((
            np.zeros(3), quat[[3, 0, 1, 2]] / norm,
            q[self._to_holosoma], np.zeros(3), omega, dq[self._to_holosoma],
        )).reshape(1, 71)
        p.interface.state = state
        p.interface.command = None
        p.latency_tracker.start_cycle()
        p.policy_action()
        p.latency_tracker.end_cycle()
        self.last_observation = p.obs_buf_dict["actor_obs"].copy()
        target, kp, kd = p.interface.command
        target, kp, kd = (array[self._to_elf3] for array in (target, kp, kd))
        if not all(array.shape == (29,) and np.all(np.isfinite(array)) for array in (target, kp, kd)):
            raise ValueError("Holosoma returned an invalid motor command")
        if np.any(target < JOINT_POSITION_MIN) or np.any(target > JOINT_POSITION_MAX):
            raise ValueError("Holosoma target exceeds ELF3 software joint limits")
        if np.any(kp < 0) or np.any(kd < 0):
            raise ValueError("Holosoma returned negative PD gains")
        return target, kp, kd
