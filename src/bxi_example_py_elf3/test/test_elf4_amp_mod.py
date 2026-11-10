from pathlib import Path
import hashlib
import runpy
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.framework.inference.api import InferenceFrame
from communication.msg import ActuatorStates

from bxi_example_py_elf3.framework.joints import JointStateView, JointCommandDefaults, JointCommandResolver
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS

ROOT = Path(__file__).resolve().parents[1]


def test_lb_b_routes_and_real_amp_inference():
    config = yaml.safe_load((ROOT / 'config/elf3_state_machine.yaml').read_text())
    runtime = load_mod_runtime(config, built_in_root=ROOT / 'mods', extra_roots=[])
    try:
        event, target = 'com.bxi.elf4/activate', 'com.bxi.elf4/elf4'
        pd, zero = 'com.bxi.basic_actions/pd_brake', 'com.bxi.basic_actions/zero_position'
        walk, dwaq = 'com.xmz.walk/walk', 'com.bxi.elf4/elf4_dwaq_1500'
        adapter = RemoteEventAdapter(runtime.config['remote_events'])
        assert adapter.extract_events(SimpleNamespace(btn_10=40)) == [event]
        assert adapter.extract_events(SimpleNamespace(btn_10=40)) == []
        routes = runtime.config['states']
        for source in (pd, zero, walk, dwaq):
            assert routes[source]['transitions']['on_event'][event]['to'] == target
        for action, destination in ((pd, pd), (zero, zero), (walk, walk),
                                    ('com.bxi.elf4/activate_dwaq', dwaq)):
            assert routes[target]['transitions']['on_event'][action]['to'] == destination
        model = ROOT / 'mods/com.bxi.elf4/assets/policy.onnx'
        assert hashlib.sha256(model.read_bytes()).hexdigest() == (
            '83ab3bd76a55af008025b081bb0f59449d218dfefb5b0e838193f38ea250b988')
        states = build_robot_states(runtime.config, runtime.state_factories)
        state = states[target]
        handle = state._policy
        handle.request()
        deadline = time.monotonic() + 10
        while handle.status not in ('ready', 'failed') and time.monotonic() < deadline:
            time.sleep(.01)
        assert handle.status == 'ready', handle.error
        policy = state.policy
        module = __import__(type(policy).__module__, fromlist=['ELF4_PARAMETERS'])
        params = module.ELF4_PARAMETERS
        frame = InferenceFrame(
            joints=JointStateView(params.layout, params.default_position.copy(), np.zeros(29)),
            quat_wxyz=np.array([1., 0., 0., 0.]), angular_velocity=np.zeros(3), command=np.zeros(3))
        policy.reset(frame)
        prior = policy._previous_action.copy()
        policy.step(frame, .02, advance=False)
        np.testing.assert_array_equal(policy._previous_action, prior)
        for command in ((0., 0., 0.), (.3, 0., 0.), (0., 0., .2)):
            frame.command[:] = command
            for _ in range(5):
                output = policy.step(frame, .02).joints
                assert output.position.shape == (29,)
                assert np.isfinite(output.position).all()
                np.testing.assert_array_equal(output.kp, params.kp)
                np.testing.assert_array_equal(output.kd, params.kd)
        robot_layout = ELF3_POLICY_JOINTS
        resolver = JointCommandResolver(robot_layout, JointCommandDefaults({}),
                                        warning_callback=lambda _: None)
        resolved = MotorFrame.empty(robot_layout)
        resolver.resolve_into(state.get_entry_frame(SimpleNamespace()), resolved)
        assert len(resolved.qpos) == 29
        for i, name in enumerate(robot_layout.names):
            j = params.layout.index(name)
            assert resolved.qpos[i] == output.position[j]
            assert resolved.kp[i] == params.kp[j] and resolved.kd[i] == params.kd[j]
        # Full-body input must reject missing real arm feedback.
        layout = ELF3_POLICY_JOINTS.select(ELF3_POLICY_JOINTS.names[:15])
        short = InferenceFrame(
            joints=JointStateView(layout, np.zeros(15), np.zeros(15)),
            quat_wxyz=frame.quat_wxyz, angular_velocity=frame.angular_velocity, command=frame.command)
        with pytest.raises(ValueError, match='missing joints'):
            policy.reset(short)
    finally:
        runtime.close()


def test_restored_arms_require_all_three_fresh_feedback_groups():
    merger_class = runpy.run_path(str(ROOT / 'mods/com.xmz.walk/feedback.py'))['FeedbackMerger']
    names = ELF3_POLICY_JOINTS.names
    merger = merger_class(names, arms=True)
    for group in (names[:15], names[15:22], names[22:]):
        msg = ActuatorStates()
        msg.name = list(reversed(group))
        msg.header.stamp.sec = 1
        msg.position = [float(names.index(name)) for name in msg.name]
        for field in ('velocity', 'effort', 'driver_temperature', 'motor_temperature'):
            setattr(msg, field, [0.] * len(group))
        assert merger.sample(1_000_000_000, 1.) is None
        assert merger.receive(msg, 1_000_000_000, 1.)
    result = merger.sample(1_000_000_000, 1.)
    assert result is not None and tuple(result.name) == names
    np.testing.assert_array_equal(result.position, np.arange(29))
    assert merger.sample(1_200_000_000, 1.2) is None
