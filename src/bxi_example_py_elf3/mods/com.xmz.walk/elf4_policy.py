"""The xmz checkpoint's named-joint observation and action contract."""

import json

import numpy as np
import onnxruntime as ort


class Elf4Policy:
    def __init__(self, config_path, policy_path):
        self.cfg = json.loads(config_path.read_text())
        self.names = tuple(self.cfg["joint_order"])
        if len(self.names) != 29 or len(set(self.names)) != 29:
            raise ValueError("ELF4 policy requires 29 unique joints")
        self.disabled = np.array([self.names.index(name) for name in
                                  self.cfg.get("policy_disabled_joints", [])], dtype=np.intp)
        self.default = np.array([self.cfg["default_joint_pos"][name] for name in self.names])
        self.kp = [float(self.cfg["joint_control"][name]["stiffness"]) for name in self.names]
        self.kd = [float(self.cfg["joint_control"][name]["damping"]) for name in self.names]
        self.ranges = np.array([self.cfg["command_ranges"][key] for key in
                                ("lin_vel_x", "lin_vel_y", "ang_vel_z")])
        self.period = self.cfg["physics_dt"] * self.cfg["decimation"]
        self.history = np.zeros((self.cfg["actor_history"], 96), dtype=np.float32)
        if self.history.shape != (10, 96):
            raise ValueError("ELF4 checkpoint requires 10x96 history")
        self.previous = np.zeros(29, dtype=np.float32)
        self.raw_action = np.zeros(29, dtype=np.float32)
        self.first = True
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(policy_path), sess_options=options,
                                           providers=["CPUExecutionProvider"])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape != [1, 960] or inputs[0].type != "tensor(float)":
            raise ValueError("Expected a float32 [1, 960] policy input")
        if len(outputs) != 1 or outputs[0].shape != [1, 29]:
            raise ValueError("Expected one [1, 29] policy output")
        self.input_name, self.output_name = inputs[0].name, outputs[0].name

    def reset(self):
        self.history.fill(0)
        self.previous.fill(0)
        self.raw_action.fill(0)
        self.first = True

    def observe(self, names, position, velocity, quat_wxyz, angular_velocity, command):
        missing = set(self.names) - set(names)
        disabled = {self.names[index] for index in self.disabled}
        if len(set(names)) != len(names) or set(names) - set(self.names) - {"head_z_joint", "head_y_joint"} or missing - disabled:
            raise ValueError("Feedback must contain all enabled joints from the policy's 29 unique joints")
        position = np.asarray(position, dtype=np.float64)
        velocity = np.asarray(velocity, dtype=np.float64)
        if position.shape != (len(names),) or velocity.shape != (len(names),):
            raise ValueError("Joint position/velocity lengths must match names")
        if not np.isfinite(position).all() or not np.isfinite(velocity).all():
            raise ValueError("Nonfinite joint feedback")
        selected = [i for i, name in enumerate(names) if name in self.names]
        indices = [self.names.index(names[i]) for i in selected]
        # Absent disabled joints affect only masked actor inputs, never hardware feedback.
        q, dq = self.default.copy(), np.zeros(29)
        q[indices], dq[indices] = position[selected], velocity[selected]
        quat = np.asarray(quat_wxyz, dtype=np.float64)
        omega = np.asarray(angular_velocity, dtype=np.float64)
        cmd = np.asarray(command, dtype=np.float64)
        if quat.shape != (4,) or omega.shape != (3,) or cmd.shape != (3,):
            raise ValueError("Invalid IMU or command dimensions")
        if not all(np.isfinite(value).all() for value in (q, dq, quat, omega, cmd)):
            raise ValueError("Nonfinite policy input")
        norm = np.linalg.norm(quat)
        if not 0.9 <= norm <= 1.1:
            raise ValueError("Invalid IMU quaternion norm")
        quat = quat / norm
        gravity = np.array([0., 0., -1.])
        t = 2 * np.cross(quat[1:], gravity)
        gravity = gravity - quat[0] * t + np.cross(quat[1:], t)
        if -gravity[2] < np.cos(self.cfg["max_tilt_angle_rad"]):
            raise ValueError("Robot tilt exceeds the policy's safety threshold")
        cmd = np.clip(cmd, self.ranges[:, 0], self.ranges[:, 1])
        scales = self.cfg["obs_scales"]
        frame = np.concatenate((omega * scales["ang_vel"], gravity * scales["projected_gravity"],
                                cmd * scales["commands"], (q - self.default) * scales["joint_pos"],
                                dq * scales["joint_vel"], self.previous * scales["actions"])).astype(np.float32)
        # Mask only the actor input; real feedback remains available to PD and safety checks.
        frame[9 + self.disabled] = 0
        frame[38 + self.disabled] = 0
        frame[67 + self.disabled] = 0
        if self.first:
            self.history[:] = frame
            self.first = False
        else:
            self.history[:-1] = self.history[1:]
            self.history[-1] = frame
        return np.clip(self.history.reshape(1, -1), -self.cfg["clip_observations"],
                       self.cfg["clip_observations"])

    def infer(self, observation):
        action = self.session.run([self.output_name], {self.input_name: observation})[0]
        if action.shape != (1, 29) or not np.isfinite(action).all():
            raise ValueError("Invalid policy actions")
        self.raw_action[:] = action[0]
        self.previous[:] = np.clip(action[0], -self.cfg["clip_actions"], self.cfg["clip_actions"])
        self.previous[self.disabled] = 0
        return self.default + self.cfg["action_scale"] * self.previous
