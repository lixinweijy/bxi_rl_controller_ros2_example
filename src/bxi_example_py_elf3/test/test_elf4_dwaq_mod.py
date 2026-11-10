from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import yaml
import pytest

from bxi_example_py_elf3.framework.inference.api import InferenceFrame
from bxi_example_py_elf3.framework.joints import JointCommandDefaults, JointCommandResolver, JointStateView, JointLayout
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS
from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("head", [False, True])
def test_rb_a_routes_and_real_dwaq_inference(head):
    config = yaml.safe_load((ROOT / "config/elf3_state_machine.yaml").read_text())
    runtime = load_mod_runtime(config, built_in_root=ROOT / "mods", extra_roots=[])
    try:
        event = "com.bxi.elf4/activate_dwaq"
        target = "com.bxi.elf4/elf4_dwaq_1500"
        pd = "com.bxi.basic_actions/pd_brake"
        walk = "com.xmz.walk/walk"
        adapter = RemoteEventAdapter(runtime.config["remote_events"])
        assert adapter.extract_events(SimpleNamespace(btn_10=41)) == [event]
        assert adapter.extract_events(SimpleNamespace(btn_10=41)) == []
        routes = runtime.config["states"]
        for source in (pd, walk):
            assert routes[source]["transitions"]["on_event"][event]["to"] == target
        assert routes[target]["transitions"]["on_event"][pd]["to"] == pd
        assert routes[target]["transitions"]["on_event"][walk]["to"] == walk
        states = build_robot_states(runtime.config, runtime.state_factories)
        handle = states[target]._policy
        handle.request()
        deadline = time.monotonic() + 10
        while handle.status not in ("ready", "failed") and time.monotonic() < deadline:
            time.sleep(.01)
        assert handle.status == "ready", handle.error
        policy = states[target].policy
        module = __import__(policy.__class__.__module__, fromlist=["DEFAULT"])
        assert "policy_53500_dwaq.onnx" in (ROOT / "mods/com.bxi.elf4/plugin.py").read_text()
        frame = InferenceFrame(
            joints=JointStateView(
                JointLayout.create(ELF3_POLICY_JOINTS.names[:15] + (("head_z_joint", "head_y_joint") if head else ())),
                np.array([module.DEFAULT[module.DWAQ_BODY.index(name)]
                          for name in ELF3_POLICY_JOINTS.names[:15]] + ([.2, -.2] if head else [])),
                np.zeros(17 if head else 15)),
            quat_wxyz=np.array([1., 0., 0., 0.]),
            angular_velocity=np.zeros(3), command=np.zeros(3),
        )
        policy.reset(frame)
        previous = policy._previous_action.copy()
        policy.step(frame, .02, advance=False)
        np.testing.assert_array_equal(policy._previous_action, previous)
        assert policy._phase_time == 0
        held = np.setdiff1d(np.arange(29), module.CONTROL_INDICES)
        for _ in range(10):
            output = policy.step(frame, .02).joints
            assert np.isfinite(output.position).all()
            np.testing.assert_array_equal(output.position[held], module.DEFAULT[held])
        assert policy._phase_time > 0
        from test_head_control import command_defaults
        names = frame.joints.layout.names
        layout = JointLayout.create(names)
        resolved = MotorFrame.empty(layout)
        natural = states[target].get_entry_frame(SimpleNamespace())
        resolver = JointCommandResolver(layout, command_defaults(),
                                        warning_callback=lambda _: None)
        resolver.resolve_into(natural, resolved)
        assert resolved.layout.names == names
        assert len(resolved.qpos) == (17 if head else 15)
        if head:
            np.testing.assert_array_equal(resolved.qpos[-2:], 0)
            np.testing.assert_allclose(resolved.kp[-2:], 16.747)
            np.testing.assert_allclose(resolved.kd[-2:], 1.066)
        states[target].on_exit(SimpleNamespace())
    finally:
        runtime.close()
