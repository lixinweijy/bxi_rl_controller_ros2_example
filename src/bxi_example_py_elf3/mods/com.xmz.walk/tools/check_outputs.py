#!/usr/bin/env python3
"""Offline simulation/ELF-limit audit; no ROS or hardware calls.

Usage: check_outputs.py ELF4_DIR OPT_BODY_ELF CURRENT_BODY_ELF PARALLEL_SO OUTPUT_DIR
The SO is built from the target's parallel_joint.cpp and parallel_probe.cpp.
"""
import csv
import ctypes
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

import mujoco
import numpy as np


def read_motor_table(path):
    """Read motor_info from an unstripped little-endian ELF64; never execute it."""
    raw = path.read_bytes()
    if raw[:6] != b"\x7fELF\x02\x01":
        raise ValueError("Expected little-endian ELF64")
    symbols = subprocess.check_output(["nm", "-S", "--defined-only", str(path)], text=True)
    symbol = next(line.split() for line in symbols.splitlines() if line.split()[-1] == "motor_info")
    address, size = int(symbol[0], 16), int(symbol[1], 16)
    phoff = struct.unpack_from("<Q", raw, 32)[0]
    phsize, phcount = struct.unpack_from("<HH", raw, 54)
    offset = None
    for i in range(phcount):
        kind, _, file_offset, vaddr, _, filesz, _, _ = struct.unpack_from("<IIQQQQQQ", raw, phoff + i * phsize)
        if kind == 1 and vaddr <= address and address + size <= vaddr + filesz:
            offset = file_offset + address - vaddr
    if offset is None or size != 17 * 56:
        raise ValueError("Unexpected 17-motor table location or ABI")
    keys = ("bus", "id", "direction", "type", "protocol_pos_max", "protocol_pos_min",
            "protocol_vel_max", "protocol_torque_max", "feedback_pos_min", "feedback_pos_max",
            "feedback_vel_max", "feedback_torque_max", "protocol_kd_max", "enabled")
    table = [dict(zip(keys, struct.unpack_from("<4i9fi", raw, offset + i * 56))) for i in range(17)]
    if not all(row["protocol_pos_min"] < row["protocol_pos_max"] and row["protocol_torque_max"] > 0 for row in table):
        raise ValueError("Invalid decoded motor limits")
    return dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(), motors=table)


def main():
    elf4, opt_elf, current_elf, parallel_so, output = map(Path, sys.argv[1:])
    output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(elf4.resolve()))
    from amp_sim2sim_elf4 import Elf4Sim2Sim

    tables = {"opt": read_motor_table(opt_elf), "current": read_motor_table(current_elf)}
    (output / "motor_tables.json").write_text(json.dumps(tables, indent=2) + "\n")
    runner = Elf4Sim2Sim(elf4 / "policy.onnx", elf4 / "model/elf4.xml", elf4 / "config.json", np.zeros(3))
    names = list(runner.joint_names)
    body_names = ["waist_y_joint", "waist_x_joint", "waist_z_joint"]
    for side in ("l", "r"):
        body_names += [f"{side}_{joint}_joint" for joint in ("hip_y", "hip_x", "hip_z", "knee_y", "ankle_y", "ankle_x")]
    body_indices = np.array([names.index(n) for n in body_names])
    parallel_names = [f"{s}_ankle_{a}_joint" for s in ("l", "r") for a in ("y", "x")] + body_names[:2]
    parallel_indices = np.array([names.index(n) for n in parallel_names])
    parallel_caps = np.array([50., 20., 50., 20., 90., 100.])
    raw_motor_caps = np.array([32., 32., 32., 32., 45., 45.])
    joint_ids = np.array([mujoco.mj_name2id(runner.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names])
    ranges = runner.model.jnt_range[joint_ids]
    probe = ctypes.CDLL(str(parallel_so.resolve()))
    pointer = ctypes.POINTER(ctypes.c_double)
    probe.xmz_parallel.argtypes = [pointer, pointer, pointer]
    probe.xmz_parallel.restype = None

    def convert(q):
        q = np.ascontiguousarray(q, dtype=np.float64)
        motor, jac = np.empty(6), np.empty(36)
        probe.xmz_parallel(q.ctypes.data_as(pointer), motor.ctypes.data_as(pointer), jac.ctypes.data_as(pointer))
        return motor, jac.reshape(6, 6, order="F")

    _, zero_jac = convert(np.zeros(6))
    zero_inverse_transpose = np.linalg.inv(zero_jac.T)
    collision_model = mujoco.MjModel.from_xml_path(str(elf4 / "model/elf4.xml"))
    robot_geoms = np.flatnonzero(collision_model.geom_group == 3)
    collision_model.geom_conaffinity[robot_geoms] |= 1
    collision_data = mujoco.MjData(collision_model)

    def contacts(qpos):
        collision_data.qpos[:] = qpos
        collision_data.qvel[:] = 0
        mujoco.mj_forward(collision_model, collision_data)
        result = {}
        for contact in collision_data.contact:
            if contact.dist >= -0.001 or not all(collision_model.geom_bodyid[g] > 0 for g in contact.geom):
                continue
            pair = "/".join(sorted(mujoco.mj_id2name(collision_model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in contact.geom))
            result[pair] = min(result.get(pair, 0), float(contact.dist))
        return result

    initial_contacts = contacts(runner.data.qpos)
    scenarios = {"zero": [0, 0, 0], "forward": [.5, 0, 0], "backward": [-.5, 0, 0],
                 "max_forward": [1.5, 0, 0], "max_backward": [-1, 0, 0],
                 "left": [0, .5, 0], "right": [0, -.5, 0],
                 "turn_left": [0, 0, 1.57], "turn_right": [0, 0, -1.57], "mixed": [.5, .3, .5]}
    report = dict(physics_dt=runner.model.opt.timestep, policy_hz=50, requested_seconds=20,
                  initial_approximate_contacts=initial_contacts, cases=[])
    rows = []
    for case, command in scenarios.items():
        for hardware_caps in ((False, True) if case in ("forward", "max_forward", "turn_left") else (False,)):
            runner.reset()
            runner.command[:] = command
            targets, actual, velocity, torques, raw_torques, motor_torques, motor_positions = [], [], [], [], [], [], []
            height, action_peak = [], 0.
            crossings = np.zeros(29, dtype=int)
            limited = np.zeros(29, dtype=int)
            contact_min, contact_frames = {}, 0
            target_contact_min, target_contact_frames = {}, 0
            fell_at = None
            for step in range(1000):
                action = runner.infer(runner.observe())
                action_peak = max(action_peak, float(np.abs(action).max()))
                target = runner.default_pos + runner.action_scale * action
                crossings += (target < ranges[:, 0]) | (target > ranges[:, 1])
                targets.append(target.copy())
                for substep in range(runner.decimation):
                    q = runner.data.qpos[runner.qpos_adr].copy()
                    dq = runner.data.qvel[runner.dof_adr].copy()
                    raw = runner.kp * (target - q) - runner.kd * dq
                    tau = np.clip(raw, -runner.effort_limit, runner.effort_limit)
                    if hardware_caps:
                        tau[parallel_indices] = np.clip(raw[parallel_indices], -parallel_caps, parallel_caps)
                        for b, i in enumerate(body_indices):
                            if i not in parallel_indices:
                                bound = tables["current"]["motors"][b]["protocol_torque_max"]
                                tau[i] = np.clip(raw[i], -bound, bound)
                    limited += np.abs(raw) > np.abs(tau) + 1e-6
                    motor_torques.append(zero_inverse_transpose @ np.clip(raw[parallel_indices], -parallel_caps, parallel_caps))
                    if hardware_caps:
                        tau[parallel_indices] = zero_jac.T @ np.clip(motor_torques[-1], -raw_motor_caps, raw_motor_caps)
                    actual.append(q)
                    velocity.append(dq)
                    raw_torques.append(raw)
                    torques.append(tau.copy())
                    runner.data.qfrc_applied[runner.dof_adr] = tau
                    mujoco.mj_step(runner.model, runner.data)
                runner.previous_action[:] = action
                height.append(float(runner.data.qpos[2]))
                if step % 5 == 0:
                    motor_positions.append(convert(runner.data.qpos[runner.qpos_adr][parallel_indices])[0])
                    found = contacts(runner.data.qpos)
                    contact_frames += bool(found)
                    for pair, distance in found.items():
                        contact_min[pair] = min(contact_min.get(pair, 0), distance)
                    desired_pose = runner.data.qpos.copy()
                    desired_pose[runner.qpos_adr] = target
                    found = contacts(desired_pose)
                    target_contact_frames += bool(found)
                    for pair, distance in found.items():
                        target_contact_min[pair] = min(target_contact_min.get(pair, 0), distance)
                if runner.data.qpos[2] < .65 or not np.isfinite(runner.data.qpos).all():
                    fell_at = (step + 1) * .02
                    break
                quat = runner.data.qpos[3:7]
                if 1 - 2 * (quat[1]**2 + quat[2]**2) < np.cos(runner.cfg["max_tilt_angle_rad"]):
                    fell_at = (step + 1) * .02
                    break
            target, q, dq, tau, raw, mt, mp = map(np.array, (targets, actual, velocity, torques, raw_torques, motor_torques, motor_positions))
            tag = case + ("_hardware_torque_caps" if hardware_caps else "_training_caps")
            np.savez_compressed(output / (tag + ".npz"), target=target, q=q, dq=dq, torque=tau, raw_torque=raw,
                                parallel_motor_torque_before_clip=mt, parallel_motor_position=mp)
            case_report = dict(case=tag, command=command, completed_s=len(target)*.02, fell_at_s=fell_at,
                               root_height_min=min(height), final_xyz=runner.data.qpos[:3].tolist(), action_abs_max=action_peak,
                               target_limit_violations={n: int(count) for n, count in zip(names, crossings) if count},
                               approximate_contact_frames=contact_frames, approximate_contacts=contact_min,
                               approximate_target_contact_frames=target_contact_frames,
                               approximate_target_contacts=target_contact_min,
                               parallel_motor_torque_max_before_clip=np.abs(mt).max(axis=0).tolist(),
                               parallel_motor_clip_counts=(np.abs(mt) > raw_motor_caps).sum(axis=0).tolist(),
                               parallel_motor_position_abs_max=np.nanmax(np.abs(mp), axis=0).tolist(),
                               parallel_math_finite=bool(np.isfinite(mt).all() and np.isfinite(mp).all()))
            for i, name in enumerate(names):
                rows.append(dict(case=tag, joint=name, urdf_min=ranges[i, 0], urdf_max=ranges[i, 1],
                                 target_min=target[:, i].min(), target_max=target[:, i].max(),
                                 actual_min=q[:, i].min(), actual_max=q[:, i].max(), velocity_abs_max=np.abs(dq[:, i]).max(),
                                 raw_torque_abs_max=np.abs(raw[:, i]).max(), torque_abs_max=np.abs(tau[:, i]).max(),
                                 target_outside_urdf_frames=crossings[i], torque_clipped_substeps=limited[i]))
            report["cases"].append(case_report)
            print(tag, "fall=", fell_at, "height_min=", min(height), "target_violations=", case_report["target_limit_violations"], flush=True)
    with (output / "joint_ranges.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "simulation_report.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
