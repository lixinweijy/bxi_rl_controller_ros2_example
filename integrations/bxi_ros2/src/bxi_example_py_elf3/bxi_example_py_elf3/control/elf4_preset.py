"""ELF4 inference preset carried into the official Holosoma extension.

Joint order, pose and observation values originate from the local Holosoma
ELF4 source snapshot. The robot_test ROS integration supplies the transport.
"""

from holosoma_inference.config.config_types.inference import InferenceConfig
from holosoma_inference.config.config_types.observation import ObservationConfig
from holosoma_inference.config.config_types.robot import RobotConfig
from holosoma_inference.config.config_types.task import TaskConfig

ELF4_29DOF_NAMES = (
    "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint",
    "l_elbow_y_joint", "l_wrist_x_joint", "l_wrist_y_joint", "l_wrist_z_joint",
    "r_shoulder_y_joint", "r_shoulder_x_joint", "r_shoulder_z_joint",
    "r_elbow_y_joint", "r_wrist_x_joint", "r_wrist_y_joint", "r_wrist_z_joint",
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
    "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
    "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
)

_ELF4_DEFAULTS_BY_NAME = {
    "l_shoulder_y_joint": 0.2,
    "r_shoulder_y_joint": 0.2,
    "l_shoulder_x_joint": 0.2,
    "r_shoulder_x_joint": -0.2,
    "l_elbow_y_joint": 0.6,
    "r_elbow_y_joint": 0.6,
    "l_hip_y_joint": -0.3,
    "r_hip_y_joint": -0.3,
    "l_knee_y_joint": 0.6,
    "r_knee_y_joint": 0.6,
    "l_ankle_y_joint": -0.3,
    "r_ankle_y_joint": -0.3,
}
_ELF4_DEFAULTS = tuple(_ELF4_DEFAULTS_BY_NAME.get(name, 0.0) for name in ELF4_29DOF_NAMES)
_ELF4_ARMS = tuple(name for name in ELF4_29DOF_NAMES if "shoulder" in name or "elbow" in name or "wrist" in name)
_ELF4_UPPER = _ELF4_ARMS
_ELF4_LOWER = tuple(name for name in ELF4_29DOF_NAMES if name not in _ELF4_UPPER)

elf4_29dof = RobotConfig(
    robot_type="elf4_29dof",
    robot="elf4",
    # Native loopback simulation transport; no robot SDK is involved.
    sdk_type="bxi_ros2",
    motor_type="serial",
    message_type="HG",
    use_sensor=False,
    num_motors=29,
    num_joints=29,
    num_upper_body_joints=14,
    default_dof_angles=_ELF4_DEFAULTS,
    default_motor_angles=_ELF4_DEFAULTS,
    motor2joint=tuple(range(29)),
    joint2motor=tuple(range(29)),
    dof_names=ELF4_29DOF_NAMES,
    dof_names_upper_body=_ELF4_UPPER,
    dof_names_lower_body=_ELF4_LOWER,
    torso_link_name="torso_link",
    left_hand_link_name="l_hand",
    right_hand_link_name="r_hand",
    unitree_legged_const=None,
    weak_motor_joint_index=None,
    motion={"body_name_ref": ["torso_link"]},
)


loco_elf4_29dof = ObservationConfig(
    obs_dict={
        "actor_obs": [
            "base_ang_vel",
            "projected_gravity",
            "command_lin_vel",
            "command_ang_vel",
            "dof_pos",
            "dof_vel",
            "actions",
            "sin_phase",
            "cos_phase",
        ]
    },
    obs_dims={
        "base_lin_vel": 3,
        "base_ang_vel": 3,
        "projected_gravity": 3,
        "command_lin_vel": 2,
        "command_ang_vel": 1,
        "dof_pos": 29,
        "dof_vel": 29,
        "actions": 29,
        "sin_phase": 2,
        "cos_phase": 2,
    },
    obs_scales={
        "base_lin_vel": 2.0,
        "base_ang_vel": 0.25,
        "projected_gravity": 1.0,
        "command_lin_vel": 1.0,
        "command_ang_vel": 1.0,
        "dof_pos": 1.0,
        "dof_vel": 0.05,
        "actions": 1.0,
        "sin_phase": 1.0,
        "cos_phase": 1.0,
    },
    history_length_dict={"actor_obs": 1},
)


locomotion_elf4 = TaskConfig(
    model_path="",  # Required: an ELF4 ONNX model with the canonical joint contract.
    rl_rate=50,
    policy_action_scale=0.25,
    use_phase=True,
    gait_period=1.0,
    desired_base_height=1.0,
    residual_upper_body_action=False,
    domain_id=0,
    interface="lo",
    velocity_input="keyboard",
    state_input="keyboard",
    joystick_type="xbox",
    joystick_device=0,
)


elf4_29dof_loco = InferenceConfig(
    robot=elf4_29dof,
    observation=loco_elf4_29dof,
    task=locomotion_elf4,
)
