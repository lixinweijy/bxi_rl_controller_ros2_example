"""Run the exported ELF4 AMP policy in MuJoCo without importing Isaac Lab.

The adjacent config.json is a snapshot of the checkpoint's training config.
Joint indices are resolved by name, so MuJoCo and Isaac joint orders may differ.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort


HERE = Path(__file__).resolve().parent
ACTOR_OBS_SIZE = 96
NUM_JOINTS = 29


def quat_rotate_inverse_wxyz(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate a world vector into the free root's local frame."""
    qvec = quat[1:]
    t = 2.0 * np.cross(qvec, vec)
    return vec - quat[0] * t + np.cross(qvec, t)


class Elf4Sim2Sim:
    def __init__(self, policy_path: Path, model_path: Path, config_path: Path, command: np.ndarray):
        self.cfg = json.loads(config_path.read_text())
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = float(self.cfg["physics_dt"])
        self.data = mujoco.MjData(self.model)
        self.joint_names = tuple(self.cfg["joint_order"])
        if len(self.joint_names) != NUM_JOINTS or len(set(self.joint_names)) != NUM_JOINTS:
            raise ValueError("ELF4 config must contain 29 unique joint names")
        if self.model.nq != 36 or self.model.nv != 35:
            raise ValueError(f"Expected ELF4 free root and 29 joints; got nq={self.model.nq}, nv={self.model.nv}")
        if not np.any((self.model.geom_contype != 0) & (self.model.geom_bodyid != 0)):
            raise ValueError("MuJoCo model has no robot collision geometry; rebuild deploy/elf4/model/elf4.xml")

        ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.joint_names]
        if min(ids) < 0:
            missing = [name for name, idx in zip(self.joint_names, ids) if idx < 0]
            raise ValueError(f"MuJoCo ELF4 model is missing joints: {missing}")
        self.qpos_adr = self.model.jnt_qposadr[ids]
        self.dof_adr = self.model.jnt_dofadr[ids]
        if not np.array_equal(np.sort(self.dof_adr), np.arange(6, 35)):
            raise ValueError("ELF4 model contains unexpected joint degrees of freedom")
        if np.any(self.model.dof_armature[self.dof_adr] <= 0):
            raise ValueError("ELF4 model lacks joint armature; rebuild model/elf4.xml with build_model.py")
        self.torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")

        self.default_pos = np.array([self.cfg["default_joint_pos"][name] for name in self.joint_names], dtype=np.float64)
        control = self.cfg["joint_control"]
        self.kp = np.array([control[name]["stiffness"] for name in self.joint_names], dtype=np.float64)
        self.kd = np.array([control[name]["damping"] for name in self.joint_names], dtype=np.float64)
        self.effort_limit = np.array([control[name]["effort_limit"] for name in self.joint_names], dtype=np.float64)
        self.velocity_limit = np.array([control[name]["velocity_limit"] for name in self.joint_names], dtype=np.float64)
        self.action_scale = float(self.cfg["action_scale"])
        self.decimation = int(self.cfg["decimation"])
        self.command = command.astype(np.float64).copy()
        self.command_ranges = np.array([self.cfg["command_ranges"][key] for key in
                                        ("lin_vel_x", "lin_vel_y", "ang_vel_z")], dtype=np.float64)
        self.command = np.clip(self.command, self.command_ranges[:, 0], self.command_ranges[:, 1])
        self.scales = self.cfg["obs_scales"]
        self.previous_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self.history = np.zeros((int(self.cfg["actor_history"]), ACTOR_OBS_SIZE), dtype=np.float32)
        if self.history.shape != (10, 96):
            raise ValueError(f"Checkpoint expects 10x96 actor history, got {self.history.shape}")
        self.first_observation = True
        self.reset_count = 0

        self.session = ort.InferenceSession(str(policy_path), providers=["CPUExecutionProvider"])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape[-1] != self.history.size:
            raise ValueError(f"Policy input must be [1, {self.history.size}]; got {[(x.name, x.shape) for x in inputs]}")
        if len(outputs) != 1 or outputs[0].shape[-1] != NUM_JOINTS:
            raise ValueError(f"Policy output must have {NUM_JOINTS} actions")
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.reset()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, 1.332)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self.qpos_adr] = self.default_pos
        self.data.qvel[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        self.previous_action[:] = 0.0
        self.history[:] = 0.0
        self.first_observation = True
        mujoco.mj_forward(self.model, self.data)

    def observe(self) -> np.ndarray:
        spatial_velocity = np.empty(6, dtype=np.float64)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.torso_id, spatial_velocity, 1)
        gravity_b = quat_rotate_inverse_wxyz(self.data.qpos[3:7], np.array([0.0, 0.0, -1.0]))
        joint_pos = self.data.qpos[self.qpos_adr] - self.default_pos
        joint_vel = self.data.qvel[self.dof_adr]  # Isaac default joint velocities are zero.
        frame = np.concatenate((
            spatial_velocity[:3] * self.scales["ang_vel"],
            gravity_b * self.scales["projected_gravity"],
            self.command * self.scales["commands"],
            joint_pos * self.scales["joint_pos"],
            joint_vel * self.scales["joint_vel"],
            self.previous_action * self.scales["actions"],
        )).astype(np.float32)
        if frame.shape != (ACTOR_OBS_SIZE,) or not np.isfinite(frame).all():
            raise ValueError("Nonfinite or incorrectly sized ELF4 actor observation")
        if self.first_observation:
            # Isaac Lab's CircularBuffer copies the first appended frame into
            # every history slot after an environment reset.
            self.history[:] = frame
            self.first_observation = False
        else:
            self.history[:-1] = self.history[1:]
            self.history[-1] = frame
        return np.clip(self.history.reshape(1, -1),
                       -self.cfg["clip_observations"], self.cfg["clip_observations"])

    def infer(self, observation: np.ndarray) -> np.ndarray:
        action = self.session.run([self.output_name], {self.input_name: observation})[0]
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (NUM_JOINTS,) or not np.isfinite(action).all():
            raise ValueError("Policy returned nonfinite or incorrectly sized actions")
        return np.clip(action, -self.cfg["clip_actions"], self.cfg["clip_actions"])

    def step(self, action: np.ndarray) -> bool:
        target = self.default_pos + self.action_scale * action.astype(np.float64)
        for _ in range(self.decimation):
            pos = self.data.qpos[self.qpos_adr]
            vel = self.data.qvel[self.dof_adr]
            torque = self.kp * (target - pos) - self.kd * vel
            # MuJoCo qfrc_applied bypasses actuator limits, so clip explicitly.
            torque = np.clip(torque, -self.effort_limit, self.effort_limit)
            self.data.qfrc_applied[self.dof_adr] = torque
            mujoco.mj_step(self.model, self.data)
        self.previous_action[:] = action
        gravity_b = quat_rotate_inverse_wxyz(self.data.qpos[3:7], np.array([0.0, 0.0, -1.0]))
        tilt = np.arccos(np.clip(-gravity_b[2], -1.0, 1.0))
        fell = (not np.isfinite(self.data.qpos).all() or
                self.data.qpos[2] < 0.65 or
                tilt > float(self.cfg["max_tilt_angle_rad"]))
        return fell

    def on_key(self, key: int) -> None:
        # MuJoCo's GLFW callback passes ASCII codes for these keys.
        mapping = {ord("W"): (0, 0.1), ord("S"): (0, -0.1),
                   ord("A"): (1, 0.1), ord("D"): (1, -0.1),
                   ord("Q"): (2, 0.1), ord("E"): (2, -0.1)}
        if key == ord(" "):
            self.command[:] = 0.0
        elif key in mapping:
            axis, delta = mapping[key]
            self.command[axis] = np.clip(self.command[axis] + delta,
                                         self.command_ranges[axis, 0], self.command_ranges[axis, 1])
        else:
            return
        print(f"command: vx={self.command[0]:+.2f}, vy={self.command[1]:+.2f}, yaw={self.command[2]:+.2f}", flush=True)

    def run(self, duration: float, headless: bool, video: Path | None, realtime: bool) -> None:
        if video is not None and not headless:
            raise ValueError("--video requires --headless")
        viewer = None
        renderer = None
        writer = None
        if not headless:
            from mujoco import viewer as mj_viewer

            viewer = mj_viewer.launch_passive(self.model, self.data, key_callback=self.on_key)
            print("Keys: W/S forward/back, A/D left/right, Q/E turn left/right, Space stop", flush=True)
        if video is not None:
            import imageio.v2 as imageio

            video.parent.mkdir(parents=True, exist_ok=True)
            renderer = mujoco.Renderer(self.model, width=960, height=720)
            writer = imageio.get_writer(video, fps=30, codec="libx264", quality=8)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.distance = 3.0
            camera.elevation = -12
            next_video_time = 0.0
        start = time.monotonic()
        step_dt = self.model.opt.timestep * self.decimation
        steps = int(np.ceil(duration / step_dt))
        try:
            for step in range(steps):
                if viewer is not None and not viewer.is_running():
                    break
                obs = self.observe()
                action = self.infer(obs)
                fell = self.step(action)
                if fell:
                    self.reset_count += 1
                    print(f"Fall reset at t={step * step_dt:.2f}s (count={self.reset_count})", flush=True)
                    self.reset()
                if viewer is not None:
                    with viewer.lock():
                        viewer.cam.lookat[:] = self.data.qpos[:3] + [0, 0, -0.5]
                    viewer.sync()
                if writer is not None and step * step_dt >= next_video_time:
                    camera.lookat[:] = self.data.qpos[:3] + [0, 0, -0.5]
                    forward = self.data.xmat[self.torso_id].reshape(3, 3)[:, 0]
                    camera.azimuth = np.degrees(np.arctan2(forward[1], forward[0])) + 225
                    renderer.update_scene(self.data, camera=camera)
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                    writer.append_data(renderer.render())
                    next_video_time += 1.0 / 30.0
                if realtime and viewer is not None:
                    delay = start + (step + 1) * step_dt - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                if step % 250 == 0:
                    print(f"t={step * step_dt:.1f}s root_z={self.data.qpos[2]:.3f} "
                          f"vx_cmd={self.command[0]:+.2f} resets={self.reset_count}", flush=True)
        finally:
            if writer is not None:
                writer.close()
            if renderer is not None:
                renderer.close()
            if viewer is not None:
                viewer.close()
        print(f"Finished; fall resets: {self.reset_count}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=HERE / "policy.onnx")
    parser.add_argument("--model", type=Path, default=HERE / "model" / "elf4.xml")
    parser.add_argument("--config", type=Path, default=HERE / "config.json")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--command", nargs=3, type=float, default=(0.5, 0.0, 0.0),
                        metavar=("VX", "VY", "YAW"), help="initial body-frame velocity command")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--video", type=Path, help="write an MP4 while running headless")
    parser.add_argument("--no-realtime", action="store_true", help="run as fast as possible")
    args = parser.parse_args()
    runner = Elf4Sim2Sim(args.policy, args.model, args.config, np.array(args.command))
    runner.run(args.duration, args.headless, args.video, not args.no_realtime)


if __name__ == "__main__":
    main()
