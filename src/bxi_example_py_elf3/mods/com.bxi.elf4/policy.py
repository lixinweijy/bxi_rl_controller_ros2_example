"""ELF4 actor with its exported 10 x 96 observation contract."""

from __future__ import annotations
import json
from pathlib import Path

import numpy as np
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
from bxi_example_py_elf3.framework.joints import JointLayout, JointParameterSet
from bxi_example_py_elf3.framework.mod_api.geometry import (
    get_gravity_orientation,
)

ELF4_CONTROL_DT = 0.02
ELF4_HISTORY_LENGTH = 10
ELF4_SINGLE_OBS_DIM = 96
ELF4_OBSERVATION_CLIP = 100.0
ELF4_ACTION_CLIP = 100.0


def _vector(value: object, size: int, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(
            f"ELF4 {label} must be a finite ({size},) vector; "
            f"got {array.shape}"
        )
    return array


def _read_contract(path: Path) -> dict:
    contract = json.loads(path.read_text())
    expected_sections = {
        "onnx": {
            "input_name": "obs",
            "input_shape": [1, 960],
            "output_name": "actions",
            "output_shape": [1, 29],
            "dtype": "float32",
        },
        "observations": {
            "frame_dim": 96,
            "history_length": 10,
            "history_order": "oldest_to_newest",
            "reset_history": "repeat_first_frame",
            "clip": 100.0,
            "empirical_normalization": False,
            "angular_velocity_frame": "torso_link body frame",
            "projected_gravity": "R_world_from_torso.T @ [0, 0, -1]",
            "command_frame": "body",
            "command_order": ["vx_m_s", "vy_m_s", "yaw_rad_s"],
            "previous_action": (
                "previous raw network output, before clipping and scaling"
            ),
            "layout": [
                {"name": name, "start": start, "stop": stop, "scale": 1.0}
                for name, start, stop in (
                    ("ang_vel", 0, 3),
                    ("projected_gravity", 3, 6),
                    ("commands", 6, 9),
                    ("joint_pos", 9, 38),
                    ("joint_vel", 38, 67),
                    ("actions", 67, 96),
                )
            ],
        },
        "control": {
            "policy_dt": ELF4_CONTROL_DT,
            "policy_hz": 50.0,
            "action_clip": ELF4_ACTION_CLIP,
            "action_delay_enabled": False,
            "target_velocity": 0.0,
        },
    }
    if contract.get("schema_version") != 1:
        raise ValueError(f"Unsupported ELF4 contract schema: {path}")
    for section, fields in expected_sections.items():
        for key, expected in fields.items():
            if contract.get(section, {}).get(key) != expected:
                raise ValueError(
                    f"ELF4 contract mismatch: {section}.{key} ({path})"
                )
    names = tuple(contract["joint_names_policy"])
    controller_names = tuple(contract["joint_names_controller"])
    if (
        len(names) != 29
        or len(set(names)) != 29
        or set(controller_names) != set(names)
    ):
        raise ValueError(
            f"ELF4 contract requires 29 unique named joints: {path}"
        )
    for field in (
        "default_joint_pos",
        "default_joint_vel",
        "action_scale",
        "stiffness",
        "damping",
        "effort_limit",
        "velocity_limit",
    ):
        _vector(contract[field], 29, field)
    for field in (
        "action_scale",
        "stiffness",
        "damping",
        "effort_limit",
        "velocity_limit",
    ):
        if np.any(np.asarray(contract[field]) <= 0):
            raise ValueError(f"ELF4 {field} must be positive: {path}")
    return contract


_CONTRACT = _read_contract(Path(__file__).with_name("assets") / "deploy.json")
ELF4_JOINTS = JointLayout(
    tuple(_CONTRACT["joint_names_policy"]),
    label=f"ELF4 model_{_CONTRACT['checkpoint_iteration']}",
)
ELF4_PARAMETERS = JointParameterSet.from_rows(
    ELF4_JOINTS,
    tuple(
        (name, default, kp, kd, scale)
        for name, default, kp, kd, scale in zip(
            ELF4_JOINTS.names,
            _CONTRACT["default_joint_pos"],
            _CONTRACT["stiffness"],
            _CONTRACT["damping"],
            _CONTRACT["action_scale"],
            strict=True,
        )
    ),
)
_DEFAULT_VELOCITY = np.asarray(
    _CONTRACT["default_joint_vel"], dtype=np.float32
)
ELF4_COMMAND_MIN = np.asarray(
    [
        _CONTRACT["training_command_ranges"][axis][0]
        for axis in ("vx", "vy", "yaw")
    ],
    dtype=np.float32,
)
ELF4_COMMAND_MAX = np.asarray(
    [
        _CONTRACT["training_command_ranges"][axis][1]
        for axis in ("vx", "vy", "yaw")
    ],
    dtype=np.float32,
)


class Elf4Policy(JointPolicy):
    """Name-mapped full-body ELF4 actor; no simulator or ROS dependency."""

    joint_contract = PolicyJointContract(ELF4_JOINTS, ELF4_JOINTS)

    def __init__(
        self,
        model: str | Path | ModelSpec,
        *,
        contract: str | Path | None = None,
        runtime: InferenceRuntime | None = None,
        backend: str = "onnxruntime",
    ) -> None:
        super().__init__()
        if (
            contract is not None
            and _read_contract(Path(contract)) != _CONTRACT
        ):
            raise ValueError(
                "ELF4 model contract differs from the Mod's joint contract"
            )
        self._runtime = runtime or default_runtime()
        spec = (
            model
            if isinstance(model, ModelSpec)
            else ModelSpec.onnx(
                model,
                input_names=("obs",),
                output_names=("actions",),
                providers=("CPUExecutionProvider",),
            )
        )
        self._backend = self._runtime.open_backend(spec, backend=backend)
        try:
            input_shape = tuple(self._backend.input_shape("obs"))
            output_shape = tuple(self._backend.output_shape("actions"))
            if input_shape != (1, 960) or output_shape != (1, 29):
                raise ValueError(
                    "ELF4 expects obs=(1, 960)/actions=(1, 29), "
                    f"got obs={input_shape}, actions={output_shape}"
                )
        except Exception:
            self._backend.close()
            raise

        self._params = ELF4_PARAMETERS
        self._input = np.zeros((1, 960), dtype=np.float32)
        self._inputs = {"obs": self._input}
        self._history = HistoryBuffer(
            ELF4_HISTORY_LENGTH, (ELF4_SINGLE_OBS_DIM,)
        )
        self._previous_action = np.zeros(29, dtype=np.float32)
        self._single = np.zeros(96, dtype=np.float32)
        self._target = self._target_buffer.position
        self.publish_output(
            self._params.default_position, self._params.kp, self._params.kd
        )

    def _build_single(self, frame: InferenceFrame) -> np.ndarray:
        joints = self.bind_joints(frame)
        if frame.command is None:
            raise ValueError("ELF4 requires a velocity command")
        quat = _vector(frame.quat_wxyz, 4, "torso quaternion")
        norm = np.linalg.norm(quat)
        if not np.isfinite(norm) or norm < 1e-8:
            raise ValueError("ELF4 torso quaternion has invalid norm")
        self._single[:3] = _vector(
            frame.angular_velocity, 3, "angular velocity"
        )
        self._single[3:6] = get_gravity_orientation(quat / norm)
        np.clip(
            _vector(frame.command, 3, "velocity command"),
            ELF4_COMMAND_MIN,
            ELF4_COMMAND_MAX,
            out=self._single[6:9],
        )
        self._single[9:38] = (
            _vector(joints.position, 29, "joint position")
            - self._params.default_position
        )
        self._single[38:67] = (
            _vector(joints.velocity, 29, "joint velocity") - _DEFAULT_VELOCITY
        )
        self._single[67:96] = self._previous_action
        if not np.isfinite(self._single).all():
            raise ValueError("ELF4 observation contains non-finite values")
        return self._single

    def reset(self, frame: InferenceFrame) -> None:
        self._previous_action.fill(0.0)
        self._history.fill(self._build_single(frame))
        self.publish_output(
            self._params.default_position, self._params.kp, self._params.kd
        )

    def step(self, frame: InferenceFrame, dt: float, *, advance: bool = True):
        if not self._history.initialized:
            self.reset(frame)
        single = self._build_single(frame)
        if advance:
            self._history.append(single)
            self._history.write_into(self._input[0])
        else:
            self._history.preview_append_into(single, self._input[0])
        # Preserve raw observations in history; clip only the flattened input,
        # matching the supplied Isaac export and independent reference runtime.
        np.clip(
            self._input,
            -ELF4_OBSERVATION_CLIP,
            ELF4_OBSERVATION_CLIP,
            out=self._input,
        )
        raw_action = _vector(
            np.asarray(self._backend.run(self._inputs)["actions"]).reshape(-1),
            29,
            "action",
        )
        if advance:
            np.copyto(self._previous_action, raw_action)
        # Last-action observations retain the raw network output. Clipping
        # must use separate storage so it cannot change that recurrent input.
        action = np.clip(raw_action, -ELF4_ACTION_CLIP, ELF4_ACTION_CLIP)
        np.add(
            self._params.default_position,
            action * self._params.action_scale,
            out=self._target,
        )
        return self.publish_output(
            self._target, self._params.kp, self._params.kd
        )

    def close(self) -> None:
        self._backend.close()
