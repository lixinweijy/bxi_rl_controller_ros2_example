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
        ('l_shoulder_y_joint', 0.2, 52.817, 3.362, 0.355),
        ('r_shoulder_y_joint', 0.2, 52.817, 3.362, 0.355),
        ('waist_y_joint', 0.0, 105.634, 6.725, 0.355),
        ('l_shoulder_x_joint', 0.2, 52.817, 3.362, 0.355),
        ('r_shoulder_x_joint', -0.2, 52.817, 3.362, 0.355),
        ('waist_x_joint', 0.0, 158.451, 10.087, 0.237),
        ('l_shoulder_z_joint', 0.0, 10.75, 0.684, 0.814),
        ('r_shoulder_z_joint', 0.0, 10.75, 0.684, 0.814),
        ('waist_z_joint', 0.0, 124.589, 7.932, 0.341),
        ('l_elbow_y_joint', 0.6, 38.296, 2.438, 0.326),
        ('r_elbow_y_joint', 0.6, 38.296, 2.438, 0.326),
        ('l_hip_y_joint', -0.3, 290.283, 18.48, 0.276),
        ('r_hip_y_joint', -0.3, 290.283, 18.48, 0.276),
        ('l_wrist_x_joint', 0.0, 8.457, 0.538, 0.591),
        ('r_wrist_x_joint', 0.0, 8.457, 0.538, 0.591),
        ('l_hip_x_joint', -0.017, 157.879, 10.051, 0.364),
        ('r_hip_x_joint', 0.017, 157.879, 10.051, 0.364),
        ('l_wrist_y_joint', 0.0, 3.456, 0.22, 1.085),
        ('r_wrist_y_joint', 0.0, 3.456, 0.22, 1.085),
        ('l_hip_z_joint', -0.072, 52.817, 3.362, 0.355),
        ('r_hip_z_joint', 0.072, 52.817, 3.362, 0.355),
        ('l_wrist_z_joint', 0.0, 3.456, 0.22, 1.085),
        ('r_wrist_z_joint', 0.0, 3.456, 0.22, 1.085),
        ('l_knee_y_joint', 0.6, 290.283, 18.48, 0.276),
        ('r_knee_y_joint', 0.6, 290.283, 18.48, 0.276),
        ('l_ankle_y_joint', -0.3087777, 76.592, 4.876, 0.326),
        ('r_ankle_y_joint', -0.3087777, 76.592, 4.876, 0.326),
        ('l_ankle_x_joint', 0.00658509, 49.785, 3.169, 0.201),
        ('r_ankle_x_joint', -0.00658509, 49.785, 3.169, 0.201),
    ),
)

__all__ = [
    "ELF3_ISAAC_JOINTS",
    "ELF3_ISAAC_PARAMETERS",
    "ELF3_LOWER_BODY_JOINTS",
    "ELF3_POLICY_JOINTS",
]
