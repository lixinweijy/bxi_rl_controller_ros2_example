"""ELF4 waist/legs DWAQ with a separate 5 x 58 observation history."""

from __future__ import annotations
import json
import math
import re
from pathlib import Path

import numpy as np
from bxi_example_py_elf3.policies.joints import ELF3_ISAAC_PARAMETERS
from bxi_example_py_elf3.framework.inference.api import InferenceFrame
from bxi_example_py_elf3.framework.inference.contract import (
    PolicyJointContract,
)
from bxi_example_py_elf3.framework.inference.history import HistoryBuffer
from bxi_example_py_elf3.framework.inference.model import ModelSpec
from bxi_example_py_elf3.framework.inference.policy import JointPolicy
from bxi_example_py_elf3.framework.inference.runtime import (
    InferenceRuntime,
    default_runtime,
)
from bxi_example_py_elf3.framework.joints import JointLayout
from bxi_example_py_elf3.framework.mod_api.geometry import (
    get_gravity_orientation,
)

ROOT = Path(__file__).resolve().parent
PROFILE = json.loads(
    (ROOT / "assets/policy_1500_dwaq_deploy.json").read_text()
)
_metadata_path = ROOT / PROFILE["metadata"]["path"]
METADATA = json.loads(_metadata_path.read_text())
CONTRACT = METADATA["contract"]
DWAQ_JOINTS = JointLayout(tuple(CONTRACT["joint_names"]), label="ELF4 DWAQ waist/legs")
DWAQ_BODY = JointLayout(
    tuple(CONTRACT["all_joint_names"]), label="ELF4 DWAQ full body"
)
CONTROL_DT = float(METADATA["control_dt"])
GAIT_PERIOD = float(METADATA["gait_period"])
GAIT_OFFSET = float(METADATA["gait_offset"])


def _vector(value, size, label):
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(
            f"ELF4 DWAQ {label} must be a finite ({size},) vector"
        )
    return result


def _gain(name, field):
    # The four replacement BXI85 axes use the deployed hardware gains.
    if name in ("l_hip_y_joint", "r_hip_y_joint", "l_knee_y_joint", "r_knee_y_joint"):
        index = ELF3_ISAAC_PARAMETERS.layout.names.index(name)
        values = ELF3_ISAAC_PARAMETERS.kp if field == "stiffness" else ELF3_ISAAC_PARAMETERS.kd
        return float(values[index])
    matches = [
        float(value)
        for group in CONTRACT["actuators"].values()
        for expression, value in group[field].items()
        if re.fullmatch(expression, name)
    ]
    if len(matches) != 1 or not math.isfinite(matches[0]) or matches[0] <= 0:
        raise ValueError(f"ELF4 DWAQ requires one positive {field} for {name}")
    return matches[0]


def _validate_contract():
    expected = {
        "robot": "elf4",
        "version": "elf4_waist_legs_feedback_v1",
        "num_obs": 58,
        "history_length": 5,
        "policy_input_dim": 290,
        "num_actions": 15,
        "control_dt": 0.02,
        "context_mode": "mean",
        "history_order": "oldest_to_newest; repeat first frame on reset",
    }
    for key, value in expected.items():
        if METADATA.get(key) != value:
            raise ValueError(f"ELF4 DWAQ metadata mismatch: {key}")
    if (
        len(DWAQ_JOINTS.names) != 15
        or len(set(DWAQ_JOINTS.names)) != 15
        or len(DWAQ_BODY.names) != 29
        or len(set(DWAQ_BODY.names)) != 29
        or tuple(METADATA["joint_names"]) != DWAQ_JOINTS.names
        or not set(DWAQ_JOINTS.names) < set(DWAQ_BODY.names)
        or set(CONTRACT["upper_body_joint_names"])
        != set(DWAQ_BODY.names) - set(DWAQ_JOINTS.names)
    ):
        raise ValueError("ELF4 DWAQ needs 15 waist/leg joints and 14 held arm joints")
    slices = {
        "root_ang_vel_b": [0, 3],
        "projected_gravity_b": [3, 6],
        "commands": [6, 9],
        "joint_pos_minus_default": [9, 24],
        "joint_vel": [24, 39],
        "previous_action": [39, 54],
        "gait_sin_cos": [54, 58],
    }
    if CONTRACT["observation_slices"] != slices:
        raise ValueError("ELF4 DWAQ observation slices do not match")
    if (
        CONTRACT["physical_root"] != "torso_link"
        or CONTRACT["upper_body_control"]
        != "arms_and_wrists_implicit_PD_default_pose_no_policy_action"
        or CONTRACT["physics_dt"] * CONTRACT["control_decimation"]
        != CONTROL_DT
        or METADATA["deployment"]["extra_velocity_filter"] is not False
        or METADATA["deployment"]["previous_action"]
        != "unclipped normalized action from previous control step"
        or GAIT_PERIOD <= 0
        or not math.isfinite(GAIT_PERIOD)
        or not math.isfinite(GAIT_OFFSET)
    ):
        raise ValueError("ELF4 DWAQ control/feedback contract does not match")
    for name in (
        "ang_vel",
        "projected_gravity",
        "commands",
        "joint_pos",
        "joint_vel",
        "actions",
    ):
        if METADATA["observation_scales"][name] != 1.0:
            raise ValueError(
                f"ELF4 DWAQ unsupported observation scale: {name}"
            )
    if METADATA["clip_actions"] != 100 or METADATA["clip_observations"] != 100:
        raise ValueError("ELF4 DWAQ clipping contract does not match")


_validate_contract()
DEFAULT = _vector(
    [CONTRACT["default_joint_pos"][name] for name in DWAQ_BODY.names],
    29,
    "default pose",
)
KP = _vector([_gain(name, "stiffness") for name in DWAQ_BODY.names], 29, "kp")
KD = _vector([_gain(name, "damping") for name in DWAQ_BODY.names], 29, "kd")
CONTROL_INDICES = np.array([DWAQ_BODY.index(name) for name in DWAQ_JOINTS.names])
SCALE = _vector(
    [CONTRACT["action_scale"][name] for name in DWAQ_JOINTS.names], 15, "scale"
)
if np.any(SCALE <= 0):
    raise ValueError("ELF4 DWAQ action scales must be positive")
COMMAND_MIN = _vector(PROFILE["command_min"], 3, "command minimum")
COMMAND_MAX = _vector(PROFILE["command_max"], 3, "command maximum")


class Elf4DwaqPolicy(JointPolicy):
    """Predict fifteen waist/leg actions and hold arms at their defaults."""

    joint_contract = PolicyJointContract(DWAQ_JOINTS, DWAQ_BODY)

    def __init__(
        self,
        model: str | Path,
        *,
        runtime: InferenceRuntime | None = None,
    ):
        super().__init__()
        model = Path(model)
        self._runtime = runtime or default_runtime()
        self._backend = self._runtime.open_backend(
            ModelSpec.onnx(
                model,
                input_names=("history",),
                output_names=("actions",),
                providers=("CPUExecutionProvider",),
            ),
            backend="onnxruntime",
        )
        try:
            input_shape = tuple(self._backend.input_shape("history"))
            output_shape = tuple(self._backend.output_shape("actions"))
            if (
                len(input_shape) != 2
                or input_shape[-1] != 290
                or len(output_shape) != 2
                or output_shape[-1] != 15
                or isinstance(input_shape[0], int)
                and input_shape[0] != 1
                or isinstance(output_shape[0], int)
                and output_shape[0] != 1
            ):
                raise ValueError(
                    "ELF4 DWAQ expects history=(1,290)/actions=(1,15)"
                )
        except BaseException:
            self._backend.close()
            raise
        self._history = HistoryBuffer(5, (58,))
        self._single = np.zeros(58, dtype=np.float32)
        self._input = np.zeros((1, 290), dtype=np.float32)
        self._inputs = {"history": self._input}
        self._previous_action = np.zeros(15, dtype=np.float32)
        self._phase_time = 0.0
        self._target = self._target_buffer.position
        self.publish_output(DEFAULT, KP, KD)

    def _build_single(self, frame: InferenceFrame):
        joints = self.bind_joints(frame)
        quaternion = _vector(frame.quat_wxyz, 4, "torso quaternion")
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 1e-8:
            raise ValueError("ELF4 DWAQ torso quaternion has invalid norm")
        self._single[:3] = _vector(
            frame.angular_velocity, 3, "angular velocity"
        )
        self._single[3:6] = get_gravity_orientation(quaternion / norm)
        self._single[6:9] = np.clip(
            _vector(frame.command, 3, "command"), COMMAND_MIN, COMMAND_MAX
        )
        self._single[9:24] = (
            _vector(joints.position, 15, "waist/leg position") - DEFAULT[CONTROL_INDICES]
        )
        self._single[24:39] = _vector(joints.velocity, 15, "waist/leg velocity")
        self._single[39:54] = self._previous_action
        left = (self._phase_time % GAIT_PERIOD) / GAIT_PERIOD
        phases = np.asarray((left, (left + GAIT_OFFSET) % 1.0))
        self._single[54:56] = np.sin(2 * math.pi * phases)
        self._single[56:58] = np.cos(2 * math.pi * phases)
        np.clip(self._single, -100, 100, out=self._single)
        return self._single

    def reset(self, frame: InferenceFrame):
        self._previous_action.fill(0)
        self._phase_time = 0.0
        self._history.fill(self._build_single(frame))
        self.publish_output(DEFAULT, KP, KD)

    def step(self, frame: InferenceFrame, dt: float, *, advance=True):
        if not self._history.initialized:
            self.reset(frame)
        single = self._build_single(frame)
        if advance:
            self._history.append(single)
            self._history.write_into(self._input[0])
        else:
            self._history.preview_append_into(single, self._input[0])
        raw = _vector(
            np.asarray(self._backend.run(self._inputs)["actions"]).reshape(-1),
            15,
            "actions",
        )
        np.copyto(self._target, DEFAULT)
        action = np.clip(raw, -100, 100)
        self._target[CONTROL_INDICES] += action * SCALE
        if advance:
            np.copyto(self._previous_action, raw)
            self._phase_time += CONTROL_DT
        return self.publish_output(self._target, KP, KD)

    def close(self):
        self._backend.close()
