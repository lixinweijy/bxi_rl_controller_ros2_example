import importlib
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import yaml

from bxi_example_py_elf3.policies.joints import ELF3_ISAAC_PARAMETERS


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MOD_ROOT = PACKAGE_ROOT / "mods/com.bxi.basic_actions"
EXPECTED_STATES = {"zero_torque", "pd_brake", "zero_position", "startup"}


def test_safe_initial_pd_and_zero_position_are_registered():
    manifest = yaml.safe_load((MOD_ROOT / "mod.yaml").read_text())
    assert set(manifest["states"]) == EXPECTED_STATES
    assert set(manifest["events"]) == {"pd_brake", "zero_position", "zero_torque"}
    assert all(
        route["from"] in EXPECTED_STATES
        and route["to"] in EXPECTED_STATES
        and route["event"] in EXPECTED_STATES
        for route in manifest["routes"]
    )
    assert yaml.safe_load(
        (PACKAGE_ROOT / "config/elf3_state_machine.yaml").read_text()
    )["mod_paths"] == []

    package_name = "_basic_actions_mod_test"
    package = ModuleType(package_name)
    package.__path__ = [str(MOD_ROOT)]
    sys.modules[package_name] = package
    model_default_position = ELF3_ISAAC_PARAMETERS.default_position.copy()
    try:
        plugin = importlib.import_module(f"{package_name}.plugin")
        definition = plugin.create_mod(None)
        assert set(definition.state_factories) == EXPECTED_STATES
        pd_state = definition.state_factories["pd_brake"](
            SimpleNamespace(name="pd_brake", state_id=1)
        )
        assert pd_state.required_resources == ()
        frame = pd_state.get_entry_frame(SimpleNamespace())
        assert frame.layout == ELF3_ISAAC_PARAMETERS.layout
        expected_position = {
            "waist_y_joint": 0.0,
            "waist_x_joint": 0.0,
            "waist_z_joint": 0.0,
            "l_hip_y_joint": -0.3,
            "l_hip_x_joint": -0.017,
            "l_hip_z_joint": -0.072,
            "l_knee_y_joint": 0.6,
            "l_ankle_y_joint": -0.3,
            "l_ankle_x_joint": 0.0,
            "r_hip_y_joint": -0.3,
            "r_hip_x_joint": 0.0017,
            "r_hip_z_joint": 0.072,
            "r_knee_y_joint": 0.6,
            "r_ankle_y_joint": -0.3,
            "r_ankle_x_joint": 0.0,
            "l_shoulder_y_joint": 0.2,
            "l_shoulder_x_joint": 0.2,
            "l_shoulder_z_joint": 0.0,
            "l_elbow_y_joint": 0.6,
            "l_wrist_x_joint": 0.0,
            "l_wrist_y_joint": 0.0,
            "l_wrist_z_joint": 0.0,
            "r_shoulder_y_joint": 0.2,
            "r_shoulder_x_joint": -0.2,
            "r_shoulder_z_joint": 0.0,
            "r_elbow_y_joint": 0.6,
            "r_wrist_x_joint": 0.0,
            "r_wrist_y_joint": 0.0,
            "r_wrist_z_joint": 0.0
        }
        expected_qpos = np.array(
            [expected_position[name] for name in frame.layout.names], dtype=np.float32
        )
        np.testing.assert_array_equal(frame.qpos, expected_qpos)
        np.testing.assert_array_equal(
            ELF3_ISAAC_PARAMETERS.default_position, model_default_position
        )
        for actual in (frame,
                       pd_state.sample_running_frame(SimpleNamespace(), .02, advance=False),
                       pd_state.sample_running_frame(SimpleNamespace(), .02, advance=True)):
            np.testing.assert_array_equal(actual.qpos, expected_qpos)
            np.testing.assert_array_equal(actual.kp, ELF3_ISAAC_PARAMETERS.kp)
            np.testing.assert_array_equal(actual.kd, ELF3_ISAAC_PARAMETERS.kd)
    finally:
        for name in list(sys.modules):
            if name == package_name or name.startswith(package_name + "."):
                sys.modules.pop(name, None)


def test_base_parameters_match_requested_elf4_named_joint_configuration():
    config = json.loads((PACKAGE_ROOT / "mods/com.xmz.walk/assets/config.json").read_text())
    params = ELF3_ISAAC_PARAMETERS
    assert tuple(config["joint_order"]) == params.layout.names
    for i, name in enumerate(params.layout.names):
        np.testing.assert_allclose(
            [params.default_position[i], params.kp[i], params.kd[i]],
            [config["default_joint_pos"][name],
             config["joint_control"][name]["stiffness"],
             config["joint_control"][name]["damping"]], rtol=1e-6, atol=1e-7)
    arms = [i for i, name in enumerate(params.layout.names)
            if any(part in name for part in ("shoulder", "elbow", "wrist"))]
    assert len(arms) == 14
    np.testing.assert_array_equal(params.kp[arms], 50)
    np.testing.assert_array_equal(params.kd[arms], 10)


def test_zero_torque_event_releases_all_31_joints_from_every_mode():
    from bxi_example_py_elf3.framework.joints import JointLayout, JointStateView
    from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
    from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
    from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter, RobotStateMachine
    from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS

    config = yaml.safe_load((PACKAGE_ROOT / "config/elf3_state_machine.yaml").read_text())
    config["graph"] = {"validate": False}
    runtime = load_mod_runtime(config, built_in_root=PACKAGE_ROOT / "mods", extra_roots=[])
    try:
        target = "com.bxi.basic_actions/zero_torque"
        adapter = RemoteEventAdapter(runtime.config["remote_events"])
        assert adapter.extract_events(SimpleNamespace(btn_2=1)) == [target]
        assert adapter.extract_events(SimpleNamespace(btn_2=1)) == []
        assert adapter.extract_events(SimpleNamespace(btn_2=0)) == []
        assert adapter.extract_events(SimpleNamespace(btn_2=1)) == [target]
        states = build_robot_states(runtime.config, runtime.state_factories)
        layout = JointLayout.create(ELF3_POLICY_JOINTS.names + ("head_z_joint", "head_y_joint"))
        outputs = []
        ctx = SimpleNamespace(robot_layout=layout,
                              robot_joints=JointStateView(layout, np.full(31, .2), np.zeros(31)),
                              set_motor_target=outputs.append)
        logger = SimpleNamespace(info=lambda _: None, warning=lambda _: None, error=lambda _: None)
        for source in states:
            if source == target:
                continue
            rule = runtime.config["states"][source]["transitions"]["on_event"][target]
            assert rule == {"to": target, "transition": "instant"}
            machine = RobotStateMachine(ctx, {**runtime.config, "initial_state": source},
                                        states, logger=logger, enter_initial=False)
            machine.update(.02, [target])
            assert machine.current_state_name == target and not machine.in_transition
            machine.update_current_state(.02)
            frame = outputs[-1]
            assert frame.layout.dof_num == 31
            for field in ("kp", "kd", "vel", "torque"):
                np.testing.assert_array_equal(getattr(frame, field), 0)
            np.testing.assert_allclose(frame.qpos, ctx.robot_joints.position, rtol=1e-7)
        pd = "com.bxi.basic_actions/pd_brake"
        assert runtime.config["states"][target]["transitions"]["on_event"][pd]["to"] == pd
    finally:
        runtime.close()
