#!/usr/bin/env python3
"""Run the real Mod state machine against local MuJoCo, without hardware nodes.

Usage: check_framework_sim.py ELF4_DIR EXAMPLE_SOURCE_PACKAGE OUTPUT_DIR
"""
import json
import os
from pathlib import Path
import sys

os.environ["ROS_DOMAIN_ID"] = "187"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
import yaml


def main():
    elf4, package, output = map(Path, sys.argv[1:])
    sys.path[:0] = [str(elf4.resolve()), str(package.resolve())]
    from amp_sim2sim_elf4 import Elf4Sim2Sim
    from communication.msg import MotionCommands
    from bxi_example_py_elf3.framework.joints import JointLayout, JointStateView
    from bxi_example_py_elf3.framework.platform.api import RobotObservation
    from bxi_example_py_elf3.framework.runtime.controller import RobotControlFramework
    from bxi_example_py_elf3.framework.runtime.logging import LoggingConfig, ScopedLoggers
    from bxi_example_py_elf3.framework.joints import JointCommandDefaults

    output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parent.parent
    rclpy.init()
    node = Node("xmz_offline_framework_sim")
    config = yaml.safe_load((package / "config/elf3_state_machine.yaml").read_text())
    config["initial_state"] = "com.bxi.basic_actions/pd_brake"
    config["graph"]["export"] = {}
    config["mod_paths"] = [str(root)]
    results = []
    try:
        for case, command in (("forward", [.5/1.5, 0, 0]), ("turn", [0, 0, .5/1.57])):
            runner = Elf4Sim2Sim(elf4 / "policy.onnx", elf4 / "model/elf4.xml", elf4 / "config.json", np.zeros(3))
            framework = RobotControlFramework(config, built_in_mod_root=package / "mods",
                command_defaults=JointCommandDefaults({}), ros_node=node,
                loggers=ScopedLoggers(node.get_logger(), LoggingConfig()))
            positions, targets, states, heights, base_positions, base_quats, commands = [], [], [], [], [], [], []
            fall = None
            try:
                for step in range(1250):
                    names = list(runner.joint_names)
                    layout = JointLayout.create(names)
                    joints = JointStateView(layout, runner.data.qpos[runner.qpos_adr].copy(),
                        runner.data.qvel[runner.dof_adr].copy(), node.get_clock().now().nanoseconds)
                    spatial = np.empty(6)
                    mujoco.mj_objectVelocity(runner.model, runner.data, mujoco.mjtObj.mjOBJ_BODY,
                                             runner.torso_id, spatial, 1)
                    quat = runner.data.qpos[3:7].copy()
                    observation = RobotObservation(joints, quat[[1,2,3,0]], quat, spatial[:3], np.array(command))
                    message = MotionCommands()
                    message.vel_des.x, message.vel_des.y, message.yawdot_des = map(float, command)
                    if step == 100:
                        message.btn_1 = 1  # RB+X
                    if step == 1150:
                        message.btn_3 = 1  # PD exit
                    framework.robot_states["com.xmz.walk/walk"].on_command(message)
                    events = framework.extract_remote_events(message)
                    frame = framework.update(observation, events, .02)
                    assert frame is not None
                    indices = [frame.layout.names.index(name) for name in names]
                    target, kp, kd = frame.qpos[indices], frame.kp[indices], frame.kd[indices]
                    assert all(np.isfinite(v).all() for v in (target, kp, kd))
                    # PD has no balance policy: emulate hoist support until walking is active,
                    # and restore support before testing the return-to-PD exit.
                    supported = framework.current_state_name != "com.xmz.walk/walk" or step >= 1150
                    for _ in range(4):
                        if supported:
                            runner.data.qpos[:7] = [0, 0, 1.332, 1, 0, 0, 0]
                            runner.data.qvel[:6] = 0
                            mujoco.mj_forward(runner.model, runner.data)
                        torque = kp * (target - runner.data.qpos[runner.qpos_adr]) - kd * runner.data.qvel[runner.dof_adr]
                        runner.data.qfrc_applied[runner.dof_adr] = np.clip(torque, -runner.effort_limit, runner.effort_limit)
                        mujoco.mj_step(runner.model, runner.data)
                    positions.append(runner.data.qpos[runner.qpos_adr].copy())
                    targets.append(target.copy())
                    states.append(framework.current_state_name)
                    heights.append(float(runner.data.qpos[2]))
                    base_positions.append(runner.data.qpos[:3].copy())
                    base_quats.append(runner.data.qpos[3:7].copy())
                    commands.append(framework.current_cmd_vel.copy())
                    quat = runner.data.qpos[3:7]
                    if not supported and (runner.data.qpos[2] < .65 or 1 - 2*(quat[1]**2 + quat[2]**2) < np.cos(1.)):
                        fall = (step+1)*.02
                        break
                if fall is None:
                    assert framework.current_state_name == "com.bxi.basic_actions/pd_brake"
                    message = MotionCommands()
                    message.btn_2 = 1
                    frame = framework.update(observation, framework.extract_remote_events(message), .02)
                    assert framework.current_state_name == "com.bxi.basic_actions/zero_torque"
                    assert not frame.kp.any() and not frame.kd.any()
                walk_steps = [i for i, state in enumerate(states) if state == "com.xmz.walk/walk" and i < 1150]
                assert len(walk_steps) >= 950
                expected = np.array(command) * [1.5, .5, 1.57]
                np.testing.assert_allclose(commands[walk_steps[-1]], expected, atol=1e-6)
                quats = np.array(base_quats)[walk_steps]
                yaw = np.unwrap(np.arctan2(2*(quats[:, 0]*quats[:, 3]+quats[:, 1]*quats[:, 2]),
                                           1-2*(quats[:, 2]**2+quats[:, 3]**2)))
                results.append(dict(case=case, seconds=len(heights)*.02, fall_at_s=fall,
                                    pd_has_hoist_support=True,
                                    unsupported_walk_seconds=len(walk_steps)*.02,
                                    command=expected.tolist(), walk_yaw_change_rad=float(yaw[-1]-yaw[0]),
                                    walk_end_xyz=base_positions[walk_steps[-1]].tolist(),
                                    height_min=min(heights), states=sorted(set(states)),
                                    zero_torque_exit_verified=fall is None))
                np.savez_compressed(output / f"framework_{case}.npz", target=targets, q=positions,
                                    height=heights, base_position=base_positions, base_quat=base_quats,
                                    command=commands, state=states)
            finally:
                framework.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (output / "framework_simulation.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    assert all(case["fall_at_s"] is None and "com.xmz.walk/walk" in case["states"] for case in results)


if __name__ == "__main__":
    main()
