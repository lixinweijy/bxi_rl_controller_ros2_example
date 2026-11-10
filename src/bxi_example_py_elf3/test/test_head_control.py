"""Head feedback and actual PD resolution, without starting hardware."""
import ast
import importlib
from pathlib import Path
import runpy
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from communication.msg import ActuatorStates

from bxi_example_py_elf3.framework.joints import (
    JointCommandDefaults, JointCommandResolver, JointDefault, JointLayout, JointStateView,
)
from bxi_example_py_elf3.framework.mod_api import MotorFrame
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS

ROOT = Path(__file__).resolve().parents[1]
HEAD = ("head_z_joint", "head_y_joint")
ADAPTER_PATH = ROOT / "bxi_example_py_elf3/bxi_example_demo.py"


def command_defaults():
    # Read the adapter's real constants without its import-time CPU scheduling setup.
    path = ADAPTER_PATH
    tree = ast.parse(path.read_text())
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == "ELF3_COMMAND_DEFAULTS"
                              for t in node.targets))
    namespace = dict(JointCommandDefaults=JointCommandDefaults, JointDefault=JointDefault)
    exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["ELF3_COMMAND_DEFAULTS"]


@pytest.mark.parametrize("arms", [False, True])
@pytest.mark.parametrize("head", [False, True])
def test_pd_holds_head_zero_and_zero_torque_releases_it(arms, head):
    package_name = "_head_control_basic_actions"
    package = ModuleType(package_name)
    package.__path__ = [str(ROOT / "mods/com.bxi.basic_actions")]
    sys.modules[package_name] = package
    try:
        definition = importlib.import_module(package_name + ".plugin").create_mod(None)
        names = (ELF3_POLICY_JOINTS.names if arms else ELF3_POLICY_JOINTS.names[:15]) + (HEAD if head else ())
        layout = JointLayout.create(names)
        ctx = SimpleNamespace(robot_layout=layout,
                              robot_joints=JointStateView(layout, np.full(len(names), .2), np.zeros(len(names))),
                              set_motor_target=lambda frame: frames.append(frame))
        resolver = JointCommandResolver(layout, command_defaults(), warning_callback=lambda _: None)
        output = MotorFrame.empty(layout)
        for kind in ("startup", "pd_brake", "zero_position", "zero_torque"):
            state = definition.state_factories[kind](SimpleNamespace(name=kind, state_id=1))
            frames = [state.get_entry_frame(ctx),
                      state.sample_running_frame(ctx, .02, advance=False),
                      state.sample_running_frame(ctx, .02, advance=True)]
            state.on_update(ctx, .02)
            for frame in frames:
                resolver.resolve_into(frame, output)
                if kind == "startup":
                    for field in ("kp", "kd", "vel", "torque"):
                        np.testing.assert_array_equal(getattr(output, field)[:len(names) - (2 if head else 0)], 0)
                for name in (HEAD if head else ()):
                    index = layout.index(name)
                    if kind == "zero_torque":
                        assert output.kp[index] == output.kd[index] == 0
                        assert output.qpos[index] == pytest.approx(.2)
                    else:
                        assert output.qpos[index] == output.vel[index] == output.torque[index] == 0
                        assert output.kp[index] == pytest.approx(16.747)
                        assert output.kd[index] == pytest.approx(1.066)
    finally:
        for name in tuple(sys.modules):
            if name == package_name or name.startswith(package_name + "."):
                sys.modules.pop(name, None)


@pytest.mark.parametrize("arms", [False, True])
@pytest.mark.parametrize("head", [False, True])
def test_merger_requires_real_head_in_body_message_and_preserves_names(arms, head):
    merger_type = runpy.run_path(str(ROOT / "mods/com.xmz.walk/feedback.py"))["FeedbackMerger"]
    names = ELF3_POLICY_JOINTS.names
    merger = merger_type(names, arms=arms, head=head)
    body_names = names[:15] + (HEAD if head else ())
    groups = [body_names] + ([names[15:22], names[22:]] if arms else [])

    def message(group, stamp=0):
        msg = ActuatorStates()
        msg.name = list(reversed(group))
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = stamp
        msg.position = [float((names + HEAD).index(name)) for name in msg.name]
        for field in ("velocity", "effort", "driver_temperature", "motor_temperature"):
            setattr(msg, field, [0.] * len(group))
        return msg

    if head:
        assert not merger.receive(message(names[:15]), 10**9, 1.)
    for group in groups:
        assert merger.sample(10**9, 1.) is None
        assert merger.receive(message(group), 10**9, 1.)
    result = merger.sample(10**9, 1.)
    expected = (names if arms else names[:15]) + (HEAD if head else ())
    assert tuple(result.name) == expected
    np.testing.assert_array_equal(result.position, [(names + HEAD).index(n) for n in expected])
    assert len(result.name) == (31 if arms else 17) - (0 if head else 2)
    for group in groups:
        assert merger.receive(message(group, 1), 10**9 + 1, 1.)
    assert merger.sample(10**9 + 101_000_000, 1.101) is None
    invalid = message(body_names, 2)
    invalid.position[-1] = float("nan")
    assert not merger.receive(invalid, 10**9 + 2, 1.)
    assert merger.sample(10**9 + 2, 1.) is None


def test_walk_head_feedback_is_excluded_from_actor_input():
    spec = importlib.util.spec_from_file_location("head_walk_policy", ROOT / "mods/com.xmz.walk/elf4_policy.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    policy = module.Elf4Policy(ROOT / "mods/com.xmz.walk/assets/config.json",
                              ROOT / "mods/com.xmz.walk/assets/policy.onnx")
    active = [i for i in range(29) if i not in policy.disabled]
    names = [policy.names[i] for i in active]
    position = policy.default[active]
    def observe(n, p, v):
        policy.reset()
        return policy.observe(n, p, v, [1., 0., 0., 0.], np.zeros(3), np.zeros(3)).copy()
    expected = observe(names, position, np.zeros(len(names)))
    with_head = names + list(HEAD)
    values = np.concatenate([position, [.3, -.2]])
    velocities = np.concatenate([np.zeros(len(names)), [.1, -.1]])
    np.testing.assert_array_equal(observe(with_head, values, velocities), expected)
    np.testing.assert_array_equal(observe(with_head[::-1], values[::-1], velocities[::-1]), expected)
    for invalid in (with_head[:-1] + ["unknown_joint"], with_head[:-1] + [with_head[0]]):
        with pytest.raises(ValueError):
            observe(invalid, values, velocities)
    with pytest.raises(ValueError):
        observe(with_head[1:], values[1:], velocities[1:])
    values[-1] = np.nan
    with pytest.raises(ValueError):
        observe(with_head, values, velocities)
