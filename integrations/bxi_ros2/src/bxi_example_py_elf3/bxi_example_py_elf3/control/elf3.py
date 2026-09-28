"""Authoritative Elf3 joint ordering and controller constants.

All full-body command producers import this module so joint ordering cannot
silently diverge between running, vibration and future test modes.
"""

import numpy as np


ROBOT_NAME = "elf3"

JOINT_NAMES = (
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
)

DOF_NUM = len(JOINT_NAMES)


def joint_feedback_29(msg):
    """Read named 29/31-joint feedback in the ROS actuator order."""
    positions = np.asarray(msg.position, dtype=np.float64)
    velocities = np.asarray(msg.velocity, dtype=np.float64)
    if positions.shape not in ((DOF_NUM,), (31,)) or velocities.shape != positions.shape:
        raise ValueError("expected matching position/velocity feedback for 29 or 31 joints")
    if msg.name:
        if len(msg.name) != len(positions) or len(set(msg.name)) != len(msg.name):
            raise ValueError("joint feedback names must be unique and match array lengths")
        indices_by_name = {name: index for index, name in enumerate(msg.name)}
        missing = [name for name in JOINT_NAMES if name not in indices_by_name]
        if missing:
            raise ValueError("joint feedback missing model joints: " + ", ".join(missing))
        indices = [indices_by_name[name] for name in JOINT_NAMES]
        positions, velocities = positions[indices], velocities[indices]
    elif len(positions) != DOF_NUM:
        raise ValueError("31-joint feedback requires names to map the 29-joint policy safely")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
        raise ValueError("non-finite model joint feedback")
    return positions, velocities


def quaternion_to_euler_array(quat):
    x, y, z, w = quat
    return np.array([
        np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)),
        np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    ])

JOINT_KP = np.array(
    [
        108.448, 162.672, 176.421,
        176.421, 176.421, 54.224, 176.421, 33.493, 21.771,
        176.421, 176.421, 54.224, 176.421, 33.493, 21.771,
        54.224, 54.224, 16.747, 54.224, 16.747, 16.747, 16.747,
        54.224, 54.224, 16.747, 54.224, 16.747, 16.747, 16.747,
    ],
    dtype=np.float64,
)

JOINT_KD = np.array(
    [
        6.904, 10.356, 11.231,
        11.231, 11.231, 3.452, 11.231, 2.132, 1.386,
        11.231, 11.231, 3.452, 11.231, 2.132, 1.386,
        3.452, 3.452, 1.066, 3.452, 1.066, 1.066, 1.066,
        3.452, 3.452, 1.066, 3.452, 1.066, 1.066, 1.066,
    ],
    dtype=np.float64,
)

JOINT_NOMINAL_POS = np.array(
    [
        0.0, 0.0, 0.0,
        -0.3, 0.0, 0.0, 0.6, -0.3, 0.0,
        -0.3, 0.0, 0.0, 0.6, -0.3, 0.0,
        0.2, 0.3, 0.0, 0.6, 0.0, 0.0, 0.0,
        0.2, -0.3, 0.0, 0.6, 0.0, 0.0, 0.0,
    ],
    dtype=np.float64,
)

# The recorded suspended-running trajectory was tuned around the legacy
# test-wire posture, whose shoulder-x values differ from the vibration center.
# Keep both named profiles so refactoring does not silently change test data.
SUSPENDED_RUN_NOMINAL_POS = JOINT_NOMINAL_POS.copy()
SUSPENDED_RUN_NOMINAL_POS[16] = 0.2
SUSPENDED_RUN_NOMINAL_POS[23] = -0.2

# Conservative software limits copied from data/elf3.xml. The hardware driver
# retains its own independent position, speed and torque protection.
JOINT_POSITION_MIN = np.array(
    [
        -0.5236, -0.2618, -2.8798,
        -2.8798, -0.48869, -2.8798, -0.087266, -0.87266, -0.34907,
        -2.8798, -3.0543, -2.8798, -0.087266, -0.87266, -0.34907,
        -2.8798, -0.34907, -2.8798, -0.95993, -2.8798, -1.309, -0.7854,
        -2.8798, -3.0543, -2.8798, -0.95993, -2.8798, -1.309, -0.7854,
    ],
    dtype=np.float64,
)

JOINT_POSITION_MAX = np.array(
    [
        0.5236, 0.2618, 2.8798,
        2.8798, 3.0543, 2.8798, 2.618, 0.7854, 0.34907,
        2.8798, 0.48869, 2.8798, 2.618, 0.7854, 0.34907,
        2.8798, 3.0543, 2.8798, 1.6581, 2.8798, 1.309, 0.7854,
        2.8798, 0.34907, 2.8798, 1.6581, 2.8798, 1.309, 0.7854,
    ],
    dtype=np.float64,
)


def validate_joint_vector(name, values):
    """Return a finite float64 Elf3 vector or raise ``ValueError``."""
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (DOF_NUM,):
        raise ValueError(
            "%s must contain exactly %d values; got shape %s"
            % (name, DOF_NUM, vector.shape)
        )
    invalid = np.flatnonzero(~np.isfinite(vector))
    if invalid.size:
        raise ValueError(
            "%s contains non-finite values for: %s"
            % (name, ", ".join(JOINT_NAMES[int(i)] for i in invalid))
        )
    return vector


def position_limit_violations(positions, margin_rad=0.0):
    """Describe joints outside the shared software position limits."""
    vector = validate_joint_vector("positions", positions)
    lower = JOINT_POSITION_MIN + float(margin_rad)
    upper = JOINT_POSITION_MAX - float(margin_rad)
    indices = np.flatnonzero((vector < lower) | (vector > upper))
    return [
        "%s=%.6f rad outside [%.6f, %.6f]"
        % (JOINT_NAMES[int(i)], vector[i], lower[i], upper[i])
        for i in indices
    ]
