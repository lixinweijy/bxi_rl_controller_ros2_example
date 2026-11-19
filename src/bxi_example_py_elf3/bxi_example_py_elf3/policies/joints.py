"""Named joint layouts and parameters owned by the built-in ELF3 policies."""

from bxi_example_py_elf3.framework.joints import JointLayout, JointParameterSet


ELF3_POLICY_JOINTS = JointLayout(
    (
        "waist_y_joint",
        "waist_x_joint",
        "waist_z_joint",
        "l_hip_y_joint",
        "l_hip_x_joint",
        "l_hip_z_joint",
        "l_knee_y_joint",
        "l_ankle_y_joint",
        "l_ankle_x_joint",
        "r_hip_y_joint",
        "r_hip_x_joint",
        "r_hip_z_joint",
        "r_knee_y_joint",
        "r_ankle_y_joint",
        "r_ankle_x_joint",
        "l_shoulder_y_joint",
        "l_shoulder_x_joint",
        "l_shoulder_z_joint",
        "l_elbow_y_joint",
        "l_wrist_x_joint",
        "l_wrist_y_joint",
        "l_wrist_z_joint",
        "r_shoulder_y_joint",
        "r_shoulder_x_joint",
        "r_shoulder_z_joint",
        "r_elbow_y_joint",
        "r_wrist_x_joint",
        "r_wrist_y_joint",
        "r_wrist_z_joint",
    ),
    label="ELF3 29-joint policy",
)

ELF3_LOWER_BODY_JOINTS = ELF3_POLICY_JOINTS.select(
    ELF3_POLICY_JOINTS.names[:15],
    label="ELF3 waist and legs policy",
)

ELF3_ISAAC_JOINTS = ELF3_POLICY_JOINTS.select(
    (
        "l_shoulder_y_joint",
        "r_shoulder_y_joint",
        "waist_y_joint",
        "l_shoulder_x_joint",
        "r_shoulder_x_joint",
        "waist_x_joint",
        "l_shoulder_z_joint",
        "r_shoulder_z_joint",
        "waist_z_joint",
        "l_elbow_y_joint",
        "r_elbow_y_joint",
        "l_hip_y_joint",
        "r_hip_y_joint",
        "l_wrist_x_joint",
        "r_wrist_x_joint",
        "l_hip_x_joint",
        "r_hip_x_joint",
        "l_wrist_y_joint",
        "r_wrist_y_joint",
        "l_hip_z_joint",
        "r_hip_z_joint",
        "l_wrist_z_joint",
        "r_wrist_z_joint",
        "l_knee_y_joint",
        "r_knee_y_joint",
        "l_ankle_y_joint",
        "r_ankle_y_joint",
        "l_ankle_x_joint",
        "r_ankle_x_joint",
    ),
    label="ELF3 Isaac policy",
)


# Every row is: (joint name, default position, kp, kd, action scale).
# Keeping the name beside its values makes the model order reviewable without
# translating between several parallel numeric arrays.
ELF3_ISAAC_PARAMETERS = JointParameterSet.from_rows(
    ELF3_ISAAC_JOINTS,
    (
        ('l_shoulder_y_joint', 0.2, 93.897, 5.978, 0.2),
        ('r_shoulder_y_joint', 0.2, 93.897, 5.978, 0.2),
        ('waist_y_joint', 0.0, 187.793, 11.955, 0.2),
        ('l_shoulder_x_joint', 0.2, 93.897, 5.978, 0.2),
        ('r_shoulder_x_joint', -0.2, 93.897, 5.978, 0.2),
        ('waist_x_joint', 0.0, 281.69, 17.933, 0.133),
        ('l_shoulder_z_joint', 0.0, 19.111, 1.217, 0.458),
        ('r_shoulder_z_joint', 0.0, 19.111, 1.217, 0.458),
        ('waist_z_joint', 0.0, 221.491, 14.101, 0.192),
        ('l_elbow_y_joint', 0.6, 68.082, 4.334, 0.184),
        ('r_elbow_y_joint', 0.6, 68.082, 4.334, 0.184),
        ('l_hip_y_joint', -0.3, 516.059, 32.853, 0.155),
        ('r_hip_y_joint', -0.3, 516.059, 32.853, 0.155),
        ('l_wrist_x_joint', 0.0, 15.035, 0.957, 0.333),
        ('r_wrist_x_joint', 0.0, 15.035, 0.957, 0.333),
        ('l_hip_x_joint', -0.017, 280.673, 17.868, 0.205),
        ('r_hip_x_joint', 0.017, 280.673, 17.868, 0.205),
        ('l_wrist_y_joint', 0.0, 6.143, 0.391, 0.61),
        ('r_wrist_y_joint', 0.0, 6.143, 0.391, 0.61),
        ('l_hip_z_joint', -0.072, 93.897, 5.978, 0.2),
        ('r_hip_z_joint', 0.072, 93.897, 5.978, 0.2),
        ('l_wrist_z_joint', 0.0, 6.143, 0.391, 0.61),
        ('r_wrist_z_joint', 0.0, 6.143, 0.391, 0.61),
        ('l_knee_y_joint', 0.6, 516.059, 32.853, 0.155),
        ('r_knee_y_joint', 0.6, 516.059, 32.853, 0.155),
        ('l_ankle_y_joint', -0.3087777, 136.163, 8.668, 0.184),
        ('r_ankle_y_joint', -0.3087777, 136.163, 8.668, 0.184),
        ('l_ankle_x_joint', 0.00658509, 88.506, 5.634, 0.113),
        ('r_ankle_x_joint', -0.00658509, 88.506, 5.634, 0.113),
    ),
)

__all__ = [
    "ELF3_ISAAC_JOINTS",
    "ELF3_ISAAC_PARAMETERS",
    "ELF3_LOWER_BODY_JOINTS",
    "ELF3_POLICY_JOINTS",
]
