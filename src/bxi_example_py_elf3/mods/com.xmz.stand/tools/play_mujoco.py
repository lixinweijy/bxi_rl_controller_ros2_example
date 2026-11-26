"""Play the XMZ standing Mod in its training model, or check it headlessly."""

import argparse
from bisect import bisect_right
from contextlib import contextmanager, nullcontext, suppress
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time

import mujoco
import numpy as np


# This tool is installed in the Mod's tools directory alongside its assets.
MOD_ROOT = Path(__file__).resolve().parent.parent
ASSETS = MOD_ROOT / "assets"
sys.path.insert(0, str(MOD_ROOT))
from policy import BLEND_EDGE_FRAMES, BLEND_SECONDS, DEFAULT_ARM_SPEED, StandPolicy


# Failure thresholds from the stand_xmz run's params/env.yaml.
MIN_HEIGHT_M = 1.073644607123304
MAX_TILT_RAD = 0.65


class ArmSequence:
    """Read authored keyframes independently for the viewer and target check."""

    def __init__(self, paths, arm_names, speed, bridge_ticks, *, cycle=False):
        self.speed = speed
        self.bridge_ticks = bridge_ticks
        banks = {}
        for motion, path in paths.items():
            with np.load(path, allow_pickle=False) as archive:
                names = archive["joint_names"].tolist()
                bank = np.take(archive["targets"], [names.index(n) for n in arm_names], axis=2)
            if motion == "training":
                if bridge_ticks:
                    length = bank.shape[1]
                    edge = min(BLEND_EDGE_FRAMES, (length - 1) // 2)
                    bank = bank[:, edge:length - edge]
                banks[motion] = [(f"Training {i}/{len(bank)}", frames)
                                 for i, frames in enumerate(bank, start=1)]
            else:
                banks[motion] = [(motion.capitalize(), bank[0])]

        ordered = []
        if cycle:
            for training in banks["training"]:
                ordered.extend((banks["wave"][0], training, banks["clap"][0]))
        else:
            for clips in banks.values():
                ordered.extend(clips)
        self.labels, self.clips = zip(*ordered)
        self.starts = [0]
        for frames in self.clips:
            span = len(frames) - 1 if bridge_ticks else len(frames)
            self.starts.append(self.starts[-1] + math.ceil(span / speed) + bridge_ticks)
        self.cycle_ticks = self.starts[-1]
        # The training bank stores equal-length clips, so every three-part round is equally long.
        self.round_ticks = self.starts[3] if cycle else None

    def phase(self, tick):
        local = tick % self.cycle_ticks
        clip = bisect_right(self.starts, local) - 1
        body_ticks = self.starts[clip + 1] - self.starts[clip] - self.bridge_ticks
        return clip, local - self.starts[clip], body_ticks

    def pose(self, tick):
        clip, elapsed, body_ticks = self.phase(tick)
        frames = self.clips[clip]
        if elapsed >= body_ticks:
            u = (elapsed - body_ticks) / self.bridge_ticks
            v = 1.0 - u
            weight = u**3 * (10.0 * v**2 + 5.0 * u * v + u**2)
            following = self.clips[(clip + 1) % len(self.clips)]
            return (1.0 - weight) * frames[-1] + weight * following[0]
        source = min(elapsed * self.speed, len(frames) - 1)
        left = int(source)
        frac = source - left
        return (1.0 - frac) * frames[left] + frac * frames[min(left + 1, len(frames) - 1)]

    def caption(self, tick):
        clip, elapsed, body_ticks = self.phase(tick)
        round_label = f"Round {clip // 3 + 1}/{len(self.clips) // 3}  " if self.round_ticks else ""
        if elapsed == 0:
            return self.labels[clip], f"{round_label}{self.speed:g}x"
        if elapsed == body_ticks:
            following = self.labels[(clip + 1) % len(self.clips)]
            return f"{self.labels[clip]} -> {following}", f"{round_label}Blend {BLEND_SECONDS:.2f} s"
        return None


class TargetCheck:
    """Independently reproduce the training runner's actor input and full target."""

    def __init__(self, profile, policy, arm_names, sequence):
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(ASSETS / "policy_tang.onnx"), options,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.body = [policy.names.index(name) for name in profile["policy_joint_names"]]
        self.arms = [policy.names.index(name) for name in arm_names]
        self.sequence = sequence
        self.scale = np.array(profile["action_scale_rad"])
        self.nominal = np.array(profile["default_joint_position_rad"])
        self.history = np.empty((10, 54), dtype=np.float32)
        self.action = np.zeros(15, dtype=np.float32)

    def compare(self, tick, q, dq, omega, rotation, limits, target):
        if tick == 0:
            self.action.fill(0)
        frame = np.concatenate((omega, -rotation[2], np.zeros(3),
                                q[self.body] - self.nominal[self.body],
                                0.1 * dq[self.body], self.action)).clip(-100, 100)
        if tick == 0:
            self.history[:] = frame
        else:
            self.history[:-1] = self.history[1:]
            self.history[-1] = frame
        self.action = self.session.run(None, {self.input_name: self.history.reshape(1, 540)})[0][0]
        expected = self.nominal.copy()
        expected[self.body] += self.scale * self.action
        if self.sequence is not None:
            expected[self.arms] = self.sequence.pose(tick)
        np.clip(expected, limits[:, 0], limits[:, 1], out=expected)
        np.testing.assert_allclose(target, expected, rtol=1e-5, atol=2e-5,
                                   err_msg=f"Mod and reference targets differ at tick {tick}")
        return float(np.max(np.abs(target - expected)))


@contextmanager
def open_viewer(model, data, reset_request):
    import glfw
    from mujoco import viewer

    def on_key(key):
        if key in (glfw.KEY_ENTER, glfw.KEY_R):
            reset_request.set()

    window = viewer.launch_passive(model, data, key_callback=on_key,
                                   show_left_ui=False, show_right_ui=False)
    try:
        yield window
    finally:
        window.close()
        # close() requests exit asynchronously. Keep Python alive while GLFW tears down.
        with suppress(mujoco.UnexpectedError):
            while window.m is not None:
                time.sleep(0.01)


def play(args):
    config_path = ASSETS / "controller_parameters.json"
    arms_path = ASSETS / "standing_arm_motion.npz"
    motion_paths = {}
    blend = args.arms in ("blend", "cycle")
    if blend or args.arms == "clap":
        motion_paths["clap"] = ASSETS / "clap_motion.npz"
    if blend or args.arms == "wave":
        motion_paths["wave"] = ASSETS / "wave_motion.npz"
    if blend or args.arms == "play":
        motion_paths["training"] = arms_path
    profile = json.loads(config_path.read_text())
    if hashlib.sha256(args.xml.read_bytes()).hexdigest() != profile["source_xml_sha256"]:
        raise ValueError("The MuJoCo XML hash differs from the training checkpoint")
    policy = StandPolicy(config_path, ASSETS / "policy_tang.onnx", arms_path,
                         clap_path=motion_paths.get("clap"), wave_path=motion_paths.get("wave"))
    if list(policy.names) != profile["joint_names"]:
        raise ValueError("StandPolicy output order differs from the controller profile")
    model = mujoco.MjModel.from_xml_path(str(args.xml.resolve()))
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    period = profile["control_dt_s"]
    substeps = round(period / dt)
    if model.nu or not math.isclose(dt, profile["physics_dt_s"]) or not math.isclose(substeps * dt, period):
        raise ValueError("Expected the training XMZ direct-torque model and timestep")
    torso = model.body("torso_link").id
    qidx = [model.joint(name).qposadr[0] for name in policy.names]
    vidx = [model.joint(name).dofadr[0] for name in policy.names]
    spawn = json.loads((ASSETS / "mujoco/config.json").read_text())["root_pos"]
    limits = np.array(profile["soft_position_limits_rad"])
    effort = np.array(profile["effort_limits_nm"])
    velocity = np.array(profile["velocity_limits_rad_s"])
    for actual, key in ((policy.nominal, "default_joint_position_rad"), (policy.kp, "kp"), (policy.kd, "kd")):
        np.testing.assert_allclose(actual, profile[key], atol=1e-6)
    play_arm_motion = args.arms != "hold"
    arm_names = [name for name in policy.names if name not in profile["policy_joint_names"]]
    arm_speed = getattr(args, "arm_speed", None)
    if arm_speed is None:
        arm_speed = DEFAULT_ARM_SPEED
    arm_indices = [policy.names.index(name) for name in arm_names]
    interpolate_arms = play_arm_motion and profile.get("interpolate_arm_targets", False)
    physics_target = np.empty_like(policy.nominal)

    force_n, push_steps = 0., 0
    if args.push_iteration is not None:
        curriculum = profile["torso_force_curriculum"]
        progress = min(1., args.push_iteration / curriculum["full_iteration"])
        force_n = float(np.interp(progress, [0, 1],
                                  [curriculum["initial_force_n"], curriculum["final_force_n"]]))
        duration = np.interp(progress, [0, 1],
                             [curriculum["initial_duration_s"], curriculum["final_duration_s"]])
        push_steps = round(duration / dt)
    push_start = round(2.0 / dt)

    def reset():
        mujoco.mj_resetData(model, data)
        data.joint("root").qpos[:3] = spawn
        data.qpos[qidx] = policy.nominal
        mujoco.mj_forward(model, data)
        arm_clip = "training" if args.arms in ("play", "hold") else args.arms
        policy.reset(play_arm_motion=play_arm_motion, arm_clip=arm_clip, arm_speed=arm_speed)

    reset()
    bridge_ticks = round(BLEND_SECONDS / period) if blend else 0
    sequence = ArmSequence(motion_paths, arm_names, arm_speed, bridge_ticks,
                           cycle=args.arms == "cycle") if play_arm_motion else None
    check = TargetCheck(profile, policy, arm_names, sequence) if args.check_targets else None
    reset_request = threading.Event()
    display = open_viewer(model, data, reset_request) if args.viewer else nullcontext()
    angular_jac = np.empty((3, model.nv))
    ticks = None
    if args.seconds is not None:
        ticks = math.ceil(args.seconds / period)
    elif not (args.viewer and (args.loop or args.arms == "hold")):
        if args.arms in ("clap", "wave"):
            ticks = sequence.cycle_ticks
        elif args.arms == "cycle":
            ticks = sequence.round_ticks
        elif args.arms == "blend":
            # Include both complete gestures even at an explicitly slower speed.
            ticks = max(math.ceil(30.0 / period), sequence.starts[2])
        else:
            ticks = math.ceil(10.0 / period)
    stop_reason = None
    with display as window:
        if window is not None:
            with window.lock():
                window.cam.lookat[:] = [0., 0., 1.]
                window.cam.distance, window.cam.elevation, window.cam.azimuth = 3., -12., 157.
                if args.arms in ("clap", "wave"):
                    window.cam.lookat[:] = [.09, 0., 1.2]
                    window.cam.distance, window.cam.elevation, window.cam.azimuth = 1.65, -7., 168.
                elif blend:
                    window.cam.lookat[:] = [.06, 0., 1.05]
                    window.cam.distance, window.cam.elevation, window.cam.azimuth = 2.45, -8., 165.
                window.opt.geomgroup[3] = 0
                window.perturb.select = torso
            window.sync()
        tick, applied_steps = 0, 0
        min_height, max_tilt, max_error = float(data.xpos[torso, 2]), 0., 0.
        deadline = time.monotonic()
        while (ticks is None or tick < ticks) and (window is None or window.is_running()):
            if reset_request.is_set():
                reset_request.clear()
                reset()
                tick, applied_steps = 0, 0
                min_height, max_tilt, max_error = float(data.xpos[torso, 2]), 0., 0.
            rotation = data.xmat[torso].reshape(3, 3)
            mujoco.mj_jacBodyCom(model, data, None, angular_jac, torso)
            omega = rotation.T @ (angular_jac @ data.qvel)
            q, dq = data.qpos[qidx], data.qvel[vidx]
            if interpolate_arms:
                previous_arms = policy.target[arm_indices]
            target = policy.step(q, dq, data.xquat[torso], omega)
            if (not np.isfinite(target).all() or np.any(target < limits[:, 0] - 1e-6)
                    or np.any(target > limits[:, 1] + 1e-6)):
                raise RuntimeError(f"Invalid policy target at t={data.time:.3f}s")
            if check is not None:
                max_error = max(max_error, check.compare(tick, q, dq, omega, rotation, limits, target))
            physics_target[:] = target
            if interpolate_arms:
                # The training controller reaches the clipped keyframe on the last physics step.
                arm_steps = np.linspace(previous_arms, target[arm_indices], substeps + 1, dtype=target.dtype)[1:]
            for substep in range(substeps):
                if interpolate_arms:
                    physics_target[arm_indices] = arm_steps[substep]
                dq = data.qvel[vidx]
                torque = np.clip(policy.kp * (physics_target - data.qpos[qidx]) - policy.kd * dq, -effort, effort)
                torque[(np.abs(dq) >= velocity) & (torque * dq > 0)] = 0.
                data.qfrc_applied[vidx] = torque
                pushing = push_start <= tick * substeps + substep < push_start + push_steps
                if pushing:
                    data.xfrc_applied[torso, 1] += force_n
                    applied_steps += 1
                mujoco.mj_step(model, data)
                if pushing:
                    data.xfrc_applied[torso, 1] -= force_n
            mujoco.mj_forward(model, data)
            if (np.any(data.warning.number) or not np.isfinite(data.qpos).all()
                    or not np.isfinite(data.qvel).all()):
                raise RuntimeError(f"MuJoCo numerical failure at t={data.time:.3f}s: {data.warning.number}")
            height = float(data.xpos[torso, 2])
            tilt = math.acos(np.clip(data.xmat[torso, 8], -1., 1.))
            min_height, max_tilt = min(min_height, height), max(max_tilt, tilt)
            if height < MIN_HEIGHT_M or tilt > MAX_TILT_RAD:
                reason = (f"Standing limit exceeded at t={data.time:.3f}s: height={height:.4f}m "
                          f"(min {MIN_HEIGHT_M:.4f}m), tilt={tilt:.4f}rad (max {MAX_TILT_RAD:.4f}rad)")
                if window is None or play_arm_motion or ticks is not None:
                    raise RuntimeError(reason)
                print(f"{reason}. Paused. Press Enter or R to reset, or close the window.", flush=True)
                window.set_texts((None, None, reason, "Paused. Enter / R to reset."))
                while window.is_running() and not reset_request.is_set():
                    window.sync()
                    time.sleep(0.05)
                if reset_request.is_set():
                    window.clear_texts()
                    deadline = time.monotonic()
                    continue
                stop_reason = reason
                break
            if window is not None:
                if sequence is not None:
                    caption = sequence.caption(tick)
                    if caption:
                        print(*caption, flush=True)
                        window.set_texts((None, None, *caption))
                window.sync()
                deadline = max(deadline + period, time.monotonic())
                time.sleep(max(0., deadline - time.monotonic()))
            tick += 1
    cycles = tick // sequence.cycle_ticks if sequence is not None else 0
    report = dict(simulated_s=round(data.time, 3), arms=args.arms, arm_cycles=cycles,
                  min_height_m=round(min_height, 5), max_tilt_rad=round(max_tilt, 5),
                  push_n=round(force_n, 3), push_s=round(applied_steps * dt, 3))
    if sequence is not None:
        report.update(arm_speed=arm_speed, arm_cycle_s=round(sequence.cycle_ticks * period, 3))
        if sequence.round_ticks:
            train_count = len(sequence.clips) // 3
            last_round = (tick - 1) // sequence.round_ticks
            report.update(
                arm_rounds=tick // sequence.round_ticks,
                arm_round_s=round(sequence.round_ticks * period, 3),
                training_clip_count=train_count,
                round_training_clip=last_round % train_count + 1 if tick else None,
            )
    if bridge_ticks:
        report["blend_seconds"] = bridge_ticks * period
    if stop_reason:
        report["stop_reason"] = stop_reason
    if check is not None:
        report["max_target_error_rad"] = max_error
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, default=ASSETS / "mujoco/elf4_xmz.xml",
                        help="Matching XMZ training XML")
    parser.add_argument("--viewer", action="store_true",
                        help="Open the viewer. Enter or R resets the pose and clips")
    parser.add_argument("--loop", action="store_true",
                        help="Repeat the selected arm motion until the viewer closes")
    parser.add_argument("--arms", choices=("cycle", "blend", "clap", "wave", "play", "hold"), default="cycle",
                        help="Cycle through wave, the next training clip and clap (default). "
                             "Blend plays clap, wave, then the entire training bank. "
                             "Clap, wave, play and hold preview the gestures, training bank or standing pose.")
    parser.add_argument("--arm-speed", type=float, help="Arm playback speed "
                        f"(default {DEFAULT_ARM_SPEED:g}x, 1 is the authored speed). Each blend lasts {BLEND_SECONDS:g} s.")
    parser.add_argument("--seconds", type=float,
                        help="Simulated seconds. Defaults to one wave/training/clap round for cycle, "
                             "a preview of both gestures (at least 30 s) for "
                             "blend, one cycle for clap or wave, or 10 s for play or hold. "
                             "Viewer loops and hold run until closed.")
    parser.add_argument("--push-iteration", type=int,
                        help="At 2 s apply the +Y torso push from this training iteration")
    parser.add_argument("--check-targets", action="store_true",
                        help="Compare every target with an independent runner")
    options = parser.parse_args()
    if options.arm_speed is not None:
        if not math.isfinite(options.arm_speed) or options.arm_speed <= 0:
            parser.error("--arm-speed must be positive and finite")
    if options.loop and not options.viewer and options.seconds is None:
        parser.error("A headless loop needs a --seconds limit")
    if options.seconds is not None and (not math.isfinite(options.seconds) or options.seconds <= 0):
        parser.error("--seconds must be positive and finite")
    if options.push_iteration is not None and options.push_iteration < 0:
        parser.error("--push-iteration must be nonnegative")
    play(options)
