"""The XMZ standing actor and its synchronized 50 Hz arm motion."""

from bisect import bisect_right
import hashlib
import io
import json
import math
from pathlib import Path

import numpy as np
import onnxruntime as ort


# The 50 Hz training clips have stationary key poses 30 frames from either end.
BLEND_EDGE_FRAMES = 30
BLEND_SECONDS = 1.0
DEFAULT_ARM_SPEED = 0.5
DEFAULT_ARM_CLIP = "cycle"
GESTURES = ("clap", "wave")


def _finite_array(values, shape, label):
    array = np.asarray(values)
    if array.shape != shape or array.dtype.kind not in "fiu" or not np.isfinite(array).all():
        raise ValueError(f"{label} requires finite numeric values of shape {shape}")
    return array


def _read_arm_motion(source, config, *, gesture=None):
    with np.load(source, allow_pickle=False) as archive:
        metadata = json.loads(archive["metadata"].item())
        if (metadata["asset_signature"] != config["asset_signature"]
                or metadata["controller_profile"] != config["profile"]
                or archive["control_dt"].item() != config["control_dt_s"]):
            raise ValueError("Arm motion belongs to a different robot or sampling period")
        names = tuple(archive["joint_names"].tolist())
        clips = archive["targets"]
    expected_names = set(config["joint_names"]) - set(config["policy_joint_names"])
    if len(names) != 14 or set(names) != expected_names:
        raise ValueError("Arm motion must cover the 14 joints outside the actor")
    indices = np.asarray([config["joint_names"].index(name) for name in names], dtype=np.intp)
    if (clips.dtype != np.float32 or clips.ndim != 3 or clips.shape[0] == 0
            or clips.shape[1] < 2 or clips.shape[2] != 14 or not np.isfinite(clips).all()):
        raise ValueError("Invalid standing arm clips")
    nominal = np.asarray(config["default_joint_position_rad"])[indices]
    if not np.allclose(clips[:, (0, -1), :], nominal, rtol=0, atol=1e-6):
        raise ValueError("Every standing arm clip must begin and end at the nominal pose")
    if gesture is not None:
        if metadata.get("motion") != gesture or len(clips) != 1:
            raise ValueError(f"Expected one named {gesture} motion")
        if metadata.get("source_xml_sha256") != config["source_xml_sha256"]:
            raise ValueError(f"{gesture} motion belongs to a different XMZ geometry")
        lower, upper = np.asarray(config["soft_position_limits_rad"])[indices].T
        max_step = np.asarray(config["velocity_limits_rad_s"])[indices] * config["control_dt_s"]
        if (np.any(clips < lower) or np.any(clips > upper)
                or np.any(np.abs(np.diff(clips[0], axis=0)) > max_step + 1e-6)):
            raise ValueError(f"{gesture} motion exceeds arm position or velocity limits")
    return indices, clips


class ArmTimeline:
    """Sample authored keyframes and fixed-duration pose transitions at the control rate."""

    def __init__(self, speed, bridge_ticks):
        self.speed = speed
        self.bridge_ticks = bridge_ticks
        self._segments = []
        self._ends = []
        self.ticks = 0
        self.loop_start = 0

    def add_motion(self, frames, *, followed_by_bridge=False):
        # The outgoing bridge supplies the final keyframe at its own tick zero.
        span = len(frames) - 1 if followed_by_bridge else len(frames)
        if span:
            self.ticks += math.ceil(span / self.speed)
            self._ends.append(self.ticks)
            self._segments.append((frames, False))

    def add_bridge(self, start, end):
        self.ticks += self.bridge_ticks
        self._ends.append(self.ticks)
        self._segments.append(((start, end), True))

    def sample(self, tick, out):
        index = bisect_right(self._ends, tick)
        local_tick = tick - (self._ends[index - 1] if index else 0)
        frames, is_bridge = self._segments[index]
        if is_bridge:
            u = local_tick / self.bridge_ticks
            weight = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
            a, b = frames
        else:
            position = min(local_tick * self.speed, len(frames) - 1)
            left = int(position)
            weight = position - left
            a = frames[left]
            b = frames[min(left + 1, len(frames) - 1)]
        np.subtract(b, a, out=out)
        out *= weight
        out += a


class StandPolicy:
    def __init__(self, config_path, onnx_path, arms_path, *, clap_path=None, wave_path=None):
        config_bytes = Path(config_path).read_bytes()
        config = json.loads(config_bytes)
        expected = {
            "robot_model": "elf4_xmz",
            "profile": "elf4_xmz_stand_540",
            "control_dt_s": 0.02,
            "physics_dt_s": 0.005,
            "actor_observation_frame_dim": 54,
            "actor_obs_history_length": 10,
            "observe_arm_motion": False,
            "observe_standing_state": False,
            "clip_actions": None,
        }
        for key, value in expected.items():
            if config[key] != value:
                raise ValueError(f"Unsupported XMZ standing parameter {key}: {config[key]!r}")

        self.names = tuple(config["joint_names"])
        leg_names = tuple(config["policy_joint_names"])
        if len(self.names) != 29 or len(set(self.names)) != 29:
            raise ValueError("The XMZ standing profile requires 29 named joints")
        if len(leg_names) != 15 or len(set(leg_names)) != 15:
            raise ValueError("The standing actor requires 15 named waist and leg joints")
        self._leg_indices = np.asarray([self.names.index(name) for name in leg_names], dtype=np.intp)
        self.nominal = _finite_array(
            config["default_joint_position_rad"], (29,), "standing pose"
        ).astype(np.float32)
        self.kp = _finite_array(config["kp"], (29,), "kp").astype(np.float32)
        self.kd = _finite_array(config["kd"], (29,), "kd").astype(np.float32)
        self._scale = _finite_array(config["action_scale_rad"], (15,), "action scale").astype(np.float32)
        self._lower, self._upper = _finite_array(
            config["soft_position_limits_rad"], (29, 2), "joint limits"
        ).astype(np.float32).T
        if np.any(self.kp < 0) or np.any(self.kd < 0) or np.any(self._scale <= 0):
            raise ValueError("Invalid standing gains or action scales")
        if (np.any(self._lower >= self._upper) or np.any(self.nominal < self._lower)
                or np.any(self.nominal > self._upper)):
            raise ValueError("The standing pose must lie within the joint limits")
        self._leg_nominal = self.nominal[self._leg_indices]

        arm_bytes = Path(arms_path).read_bytes()
        if hashlib.sha256(arm_bytes).hexdigest() != config["arm_database_sha256"]:
            raise ValueError("Arm motion SHA-256 differs from the standing checkpoint")
        self._arm_indices, training_clips = _read_arm_motion(io.BytesIO(arm_bytes), config)
        self._arm_motions = {"training": training_clips}
        self._arm_buffer = np.empty(14, dtype=np.float32)
        self._bridge_ticks = round(BLEND_SECONDS / config["control_dt_s"])
        for name, path in (("clap", clap_path), ("wave", wave_path)):
            if path is not None:
                indices, clips = _read_arm_motion(path, config, gesture=name)
                source_indices = indices.tolist()
                order = [source_indices.index(i) for i in self._arm_indices]
                self._arm_motions[name] = np.take(clips, order, axis=2)

        max_step = np.maximum.reduce([
            np.abs(np.diff(clips, axis=1)).max(axis=(0, 1)) for clips in self._arm_motions.values()
        ]).astype(np.float64)
        self._longest_clip = max(clips.shape[1] for clips in self._arm_motions.values())
        allowed_step = np.asarray(config["velocity_limits_rad_s"])[self._arm_indices] * config["control_dt_s"]
        self._max_arm_speed = np.divide(allowed_step, max_step, out=np.full_like(max_step, np.inf),
                                        where=max_step > 0).min()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = ort.InferenceSession(
            str(onnx_path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self._session.get_inputs(), self._session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape != [1, 540] or inputs[0].type != "tensor(float)":
            raise ValueError("Standing ONNX must take one float32 [1, 540] input")
        if len(outputs) != 1 or outputs[0].shape != [1, 15] or outputs[0].type != "tensor(float)":
            raise ValueError("Standing ONNX must return one float32 [1, 15] action")
        model_meta = self._session.get_modelmeta().custom_metadata_map
        config_hash = hashlib.sha256(config_bytes).hexdigest()
        if (model_meta.get("profile") != config["profile"]
                or model_meta.get("controller_parameters_sha256") != config_hash):
            raise ValueError("Standing ONNX and controller parameters belong to different exports")
        self._input = inputs[0].name
        self._outputs = [outputs[0].name]
        self.history = np.empty((10, 54), dtype=np.float32)
        self._next_history = np.empty_like(self.history)
        self.previous = np.zeros(15, dtype=np.float32)
        self.target = self.nominal.copy()
        self.reset(play_arm_motion=False)
        # Warm the session on the standing pose during resource loading.
        self.step(self.nominal, np.zeros(29), (1., 0., 0., 0.), (0., 0., 0.))
        self.reset()

    def _arm_timeline(self, mode, speed):
        timeline = ArmTimeline(speed, self._bridge_ticks)
        if mode not in ("cycle", "blend"):
            for frames in self._arm_motions[mode]:
                timeline.add_motion(frames)
            return timeline

        training = self._arm_motions["training"]
        edge = min(BLEND_EDGE_FRAMES, (training.shape[1] - 1) // 2)
        body = training[:, edge:training.shape[1] - edge]
        if mode == "cycle":
            wave = self._arm_motions["wave"][0]
            clap = self._arm_motions["clap"][0]
            clips = []
            for exercise in body:
                clips.extend((wave, exercise, clap))
        else:
            clips = [self._arm_motions[name][0] for name in GESTURES if name in self._arm_motions]
            if not clips and edge:
                # A training-only blend lifts from nominal once before looping the trimmed clips.
                timeline.add_motion(training[0, :edge + 1], followed_by_bridge=True)
                timeline.loop_start = timeline.ticks
            clips.extend(body)

        for i, frames in enumerate(clips):
            timeline.add_motion(frames, followed_by_bridge=True)
            timeline.add_bridge(frames[-1], clips[(i + 1) % len(clips)][0])
        return timeline

    def reset(self, *, play_arm_motion=True, arm_clip=None, arm_speed=DEFAULT_ARM_SPEED):
        """Start from the nominal pose and select arm playback for this session."""
        has_gestures = all(name in self._arm_motions for name in GESTURES)
        if arm_clip is None:
            arm_clip = DEFAULT_ARM_CLIP if has_gestures else "training"
        if arm_clip == "cycle" and not has_gestures:
            raise ValueError("XMZ arm cycle requires both wave and clap motions")
        if arm_clip not in ("cycle", "blend") and arm_clip not in self._arm_motions:
            raise ValueError(f"Unknown or unavailable XMZ arm clip: {arm_clip}")
        if not math.isfinite(arm_speed) or arm_speed <= 0 or not math.isfinite(self._longest_clip / arm_speed):
            raise ValueError("Arm speed must give a finite positive playback duration")
        if play_arm_motion and arm_speed > self._max_arm_speed:
            raise ValueError(f"Arm speed exceeds the trajectory velocity limit ({self._max_arm_speed:.3f}x)")
        self._playback = self._arm_timeline(arm_clip, arm_speed) if play_arm_motion else None
        self._arm_tick = 0
        self.history.fill(0)
        self.previous.fill(0)
        self.target[:] = self.nominal
        self._first = True

    def step(self, position, velocity, quat_wxyz, omega):
        """Advance one policy tick with 29 named joints and torso-frame angular velocity."""
        position = _finite_array(position, (29,), "joint position")
        velocity = _finite_array(velocity, (29,), "joint velocity")
        omega = _finite_array(omega, (3,), "torso angular velocity")
        quat = _finite_array(quat_wxyz, (4,), "torso quaternion")
        w, x, y, z = map(float, quat)
        norm_sq = w * w + x * x + y * y + z * z
        if not 0.9 ** 2 <= norm_sq <= 1.1 ** 2:
            raise ValueError("Invalid torso quaternion norm")

        frame = self._next_history[-1]
        frame[:3] = omega
        # World down expressed in the torso frame, i.e. minus row 2 of R_world_torso.
        factor = 2.0 / norm_sq
        frame[3:6] = (
            factor * (w * y - x * z),
            -factor * (w * x + y * z),
            factor * (x * x + y * y) - 1.,
        )
        frame[6:9] = 0  # The standing task has zero forward, lateral and yaw commands.
        np.subtract(position[self._leg_indices], self._leg_nominal, out=frame[9:24])
        np.multiply(velocity[self._leg_indices], 0.1, out=frame[24:39])
        frame[39:] = self.previous
        np.clip(frame, -100., 100., out=frame)
        if self._first:
            self._next_history[:-1] = frame
        else:
            self._next_history[:-1] = self.history[1:]
        action = self._session.run(self._outputs, {self._input: self._next_history.reshape(1, 540)})[0][0]
        _finite_array(action, (15,), "standing actor output")

        self.target[self._leg_indices] = self._leg_nominal + self._scale * action
        if self._playback is not None:
            self._playback.sample(self._arm_tick, self._arm_buffer)
            self._arm_tick += 1
            if self._arm_tick == self._playback.ticks:
                self._arm_tick = self._playback.loop_start
            self.target[self._arm_indices] = self._arm_buffer
        np.clip(self.target, self._lower, self._upper, out=self.target)
        self.history, self._next_history = self._next_history, self.history
        self._first = False
        self.previous[:] = action
        return self.target
