from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.framework.joints import JointCommandDefaults, JointCommandResolver, JointStateView
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter
from bxi_example_py_elf3.policies.joints import ELF3_ISAAC_PARAMETERS, ELF3_POLICY_JOINTS


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("dof", [15, 29])
def test_zero_position_keeps_pd_gains_and_all_controlled_targets_zero(dof):
    config = yaml.safe_load((ROOT / "config/elf3_state_machine.yaml").read_text())
    assert config["initial_state"] == "com.bxi.basic_actions/startup"
    runtime = load_mod_runtime(config, built_in_root=ROOT / "mods", extra_roots=[])
    try:
        target = "com.bxi.basic_actions/zero_position"
        pd = "com.bxi.basic_actions/pd_brake"
        torque = "com.bxi.basic_actions/zero_torque"
        walk = "com.xmz.walk/walk"
        dwaq = "com.bxi.elf4/elf4_dwaq_1500"
        adapter = RemoteEventAdapter(runtime.config["remote_events"])
        assert adapter.extract_events(SimpleNamespace(btn_4=1)) == [target]
        assert adapter.extract_events(SimpleNamespace(btn_4=1)) == []
        routes = runtime.config["states"]
        for source in (config["initial_state"], torque, pd, walk, dwaq):
            rule = routes[source]["transitions"]["on_event"][target]
            assert rule["to"] == target and rule["transition"] == "first_frame_switch"
        for event, destination in ((pd, pd), (walk, walk), ("com.bxi.elf4/activate_dwaq", dwaq)):
            assert routes[target]["transitions"]["on_event"][event]["to"] == destination
        states = build_robot_states(runtime.config, runtime.state_factories)
        state = states[target]
        assert state.required_resources == ()
        layout = ELF3_POLICY_JOINTS.select(ELF3_POLICY_JOINTS.names[:dof])
        ctx = SimpleNamespace(robot_layout=layout,
                              robot_joints=JointStateView(layout, np.full(dof, .3), np.zeros(dof)),
                              set_motor_target=lambda frame: outputs.append(frame))
        outputs = [state.get_entry_frame(ctx),
                   state.sample_running_frame(ctx, .02, advance=False),
                   state.sample_running_frame(ctx, .02, advance=True)]
        state.on_update(ctx, .02)
        reference = states[pd].get_entry_frame(ctx)
        defaults = ELF3_ISAAC_PARAMETERS.default_position.copy()
        resolver = JointCommandResolver(layout, JointCommandDefaults({}), warning_callback=lambda _: None)
        actual = MotorFrame.empty(layout)
        for frame in outputs:
            np.testing.assert_array_equal(frame.qpos, 0)
            np.testing.assert_array_equal(frame.kp, reference.kp)
            np.testing.assert_array_equal(frame.kd, reference.kd)
            resolver.resolve_into(frame, actual)
            assert actual.layout.dof_num == dof
            assert np.all(actual.kp > 0) and np.all(actual.kd > 0)
            for values in (actual.qpos, actual.vel, actual.torque):
                np.testing.assert_array_equal(values, 0)
        np.testing.assert_array_equal(ctx.robot_joints.position, .3)
        np.testing.assert_array_equal(reference.qpos, defaults)
        for values in (states[torque].get_entry_frame(ctx).kp, states[torque].get_entry_frame(ctx).kd):
            np.testing.assert_array_equal(values, 0)
    finally:
        runtime.close()
