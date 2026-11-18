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
        ('l_shoulder_y_joint', 0.2, 116.12, 7.392, 0.161),
        ('r_shoulder_y_joint', 0.2, 116.12, 7.392, 0.161),
        ('waist_y_joint', 0.0, 232.24, 14.785, 0.161),
        ('l_shoulder_x_joint', 0.2, 116.12, 7.392, 0.161),
        ('r_shoulder_x_joint', -0.2, 116.12, 7.392, 0.161),
        ('waist_x_joint', 0.0, 348.36, 22.177, 0.108),
        ('l_shoulder_z_joint', 0.0, 28.473, 1.813, 0.307),
        ('r_shoulder_z_joint', 0.0, 28.473, 1.813, 0.307),
        ('waist_z_joint', 0.0, 256.25, 16.313, 0.166),
        ('l_elbow_y_joint', 0.6, 67.426, 4.292, 0.185),
        ('r_elbow_y_joint', 0.6, 67.426, 4.292, 0.185),
        ('l_hip_y_joint', -0.3, 582.521, 37.084, 0.137),
        ('r_hip_y_joint', -0.3, 582.521, 37.084, 0.137),
        ('l_wrist_x_joint', 0.0, 379.864, 24.183, 0.013),
        ('r_wrist_x_joint', 0.0, 379.864, 24.183, 0.013),
        ('l_hip_x_joint', -0.017, 277.246, 17.65, 0.207),
        ('r_hip_x_joint', 0.017, 277.246, 17.65, 0.207),
        ('l_wrist_y_joint', 0.0, 16.747, 1.066, 0.224),
        ('r_wrist_y_joint', 0.0, 16.747, 1.066, 0.224),
        ('l_hip_z_joint', -0.072, 116.12, 7.392, 0.161),
        ('r_hip_z_joint', 0.072, 116.12, 7.392, 0.161),
        ('l_wrist_z_joint', 0.0, 16.747, 1.066, 0.224),
        ('r_wrist_z_joint', 0.0, 16.747, 1.066, 0.224),
        ('l_knee_y_joint', 0.6, 582.521, 37.084, 0.137),
        ('r_knee_y_joint', 0.6, 582.521, 37.084, 0.137),
        ('l_ankle_y_joint', -0.3087777, 134.852, 8.585, 0.185),
        ('r_ankle_y_joint', -0.3087777, 134.852, 8.585, 0.185),
        ('l_ankle_x_joint', 0.00658509, 87.654, 5.58, 0.114),
        ('r_ankle_x_joint', -0.00658509, 87.654, 5.58, 0.114),
    ),
)

__all__ = [
    "ELF3_ISAAC_JOINTS",
    "ELF3_ISAAC_PARAMETERS",
    "ELF3_LOWER_BODY_JOINTS",
    "ELF3_POLICY_JOINTS",
]
