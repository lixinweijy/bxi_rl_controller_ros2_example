#!/usr/bin/env python3
"""Offline contract check. Pass the bxi_example_py_elf3 source package directory."""
import importlib
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

PACKAGE = Path(sys.argv.pop(1)).resolve()
sys.path.insert(0, str(PACKAGE))
ROOT = Path(__file__).resolve().parent.parent

from communication.msg import ActuatorStates
from bxi_example_py_elf3.framework.joints import JointLayout, JointStateView
from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS


class ModTest(unittest.TestCase):
    def setUp(self):
        config = yaml.safe_load((PACKAGE / "config/elf3_state_machine.yaml").read_text())
        self.runtime = load_mod_runtime(config, built_in_root=PACKAGE / "mods", extra_roots=[])
        self.addCleanup(self.runtime.close)
        self.runtime.resources.load_startup()
        self.states = build_robot_states(self.runtime.config, self.runtime.state_factories)
        self.state = self.states["com.xmz.walk/walk"]
        self.policy = self.state.policy.get()
        self.now = 1_000_000_000
        names = ELF3_POLICY_JOINTS.names
        defaults = dict(zip(self.policy.names, self.policy.default))
        joints = JointStateView(JointLayout.create(names), np.array([defaults[n] for n in names]),
                                np.zeros(29), timestamp_ns=self.now)
        self.ctx = SimpleNamespace(robot_joints=joints, current_quat_wxyz=np.array([1., 0, 0, 0]),
                                   current_omega=np.zeros(3), current_raw_cmd_vel=np.array([0.3, 0., 0.]),
                                   current_cmd_vel=np.zeros(3), speed_profiles=self.runtime.config["speed_profiles"],
                                   ros_node=SimpleNamespace(get_clock=lambda: SimpleNamespace(
                                       now=lambda: SimpleNamespace(nanoseconds=self.now))))
        self.state.speed_profile_name = "com.xmz.walk/walk"
        self.state.on_prepare(self.ctx, None)

    def test_preview_has_no_history_or_action_side_effect(self):
        before = self.policy.history.copy()
        previous = self.policy.previous.copy()
        frame = self.state.sample_running_frame(self.ctx, .02, advance=False)
        np.testing.assert_allclose(frame.qpos, self.policy.default, atol=1e-7)
        np.testing.assert_array_equal(self.policy.history, before)
        np.testing.assert_array_equal(self.policy.previous, previous)
        output = self.state.sample_running_frame(self.ctx, .02, advance=True)
        self.assertTrue(np.isfinite(output.qpos).all())
        self.state.on_prepare(self.ctx, None)
        self.assertTrue(self.policy.first)
        np.testing.assert_array_equal(self.policy.previous, 0)

    def test_feedback_must_be_full_fresh_and_upright(self):
        self.now += 100_000_001
        with self.assertRaisesRegex(ValueError, "fresh"):
            self.state.sample_running_frame(self.ctx, .02, advance=True)
        self.now = self.ctx.robot_joints.timestamp_ns
        self.ctx.current_quat_wxyz[:] = [0, 1, 0, 0]
        with self.assertRaisesRegex(ValueError, "tilt"):
            self.state.on_prepare(self.ctx, None)
        self.ctx.current_quat_wxyz[:] = [1, 0, 0, 0]
        joints = self.ctx.robot_joints
        self.ctx.robot_joints = JointStateView(JointLayout.create(joints.layout.names[:14]),
                                              joints.position[:14], joints.velocity[:14], self.now)
        with self.assertRaisesRegex(ValueError, "29"):
            self.state.on_prepare(self.ctx, None)

    def test_telemetry_keeps_raw_clipped_and_target_distinct(self):
        messages = {}
        self.state.trace_publishers = {
            name: SimpleNamespace(get_subscription_count=lambda: 1,
                                  publish=lambda msg, key=name: messages.__setitem__(key, msg.data))
            for name in ("observation", "action_raw", "action_clipped", "target")}
        action = np.linspace(-120., 120., 29, dtype=np.float32).reshape(1, 29)
        self.policy.session = SimpleNamespace(run=lambda *args: [action])
        frame = self.state.sample_running_frame(self.ctx, .02, advance=True)
        np.testing.assert_array_equal(messages["action_raw"], action[0])
        clipped = np.clip(action[0], -100, 100)
        clipped[self.policy.disabled] = 0
        np.testing.assert_array_equal(messages["action_clipped"], clipped)
        np.testing.assert_allclose(messages["target"], frame.qpos, atol=2e-6)
        self.assertEqual(len(messages["observation"]), 960)
        self.state.on_prepare(self.ctx, None)
        np.testing.assert_array_equal(self.policy.raw_action, 0)

    def test_adapter_preserves_real_arm_feedback(self):
        p = self.policy
        self.assertEqual(p.disabled.size, 0)
        arms = np.array([i for i, n in enumerate(p.names)
                         if any(part in n for part in ("shoulder", "elbow", "wrist"))])
        self.assertEqual(len(arms), 14)
        q, dq = p.default.copy(), np.zeros(29)
        q[arms] += .3
        dq[arms] = -.7
        original_q, original_dq = q.copy(), dq.copy()
        action = np.linspace(-2, 2, 29, dtype=np.float32).reshape(1, 29)
        p.session = SimpleNamespace(run=lambda *args: [action])
        for _ in range(12):
            p.previous[arms] = .9
            obs = p.observe(list(reversed(p.names)), q[::-1], dq[::-1],
                            [1, 0, 0, 0], [0, 0, 0], [.1, 0, 0]).reshape(10, 96)
            for start, expected in ((9, .3), (38, -.7), (67, .9)):
                np.testing.assert_allclose(obs[-1, start + arms], expected)
            target = p.infer(obs.reshape(1, 960))
            np.testing.assert_allclose(target, p.default + .23 * action[0])
        np.testing.assert_array_equal(q, original_q)
        np.testing.assert_array_equal(dq, original_dq)

    def test_no_arm_model_masks_arm_observations_and_actions(self):
        p = self.policy
        arms = np.array([i for i, n in enumerate(p.names)
                         if any(part in n for part in ("shoulder", "elbow", "wrist"))])
        self.assertEqual(len(arms), 14)
        rng = np.random.default_rng(15)
        for _ in range(32):
            obs = rng.normal(size=(1, 960)).astype(np.float32)
            changed = obs.copy().reshape(10, 96)
            for start in (9, 38, 67):
                changed[:, start + arms] = rng.normal(size=(10, 14)) * 10
            target = p.infer(obs).copy()
            np.testing.assert_array_equal(p.raw_action[arms], 0)
            np.testing.assert_array_equal(target[arms], p.default[arms])
            np.testing.assert_array_equal(p.infer(changed.reshape(1, 960)), target)

    def test_deployment_launch_defaults_enable_arms_and_head(self):
        import ast
        for path in (PACKAGE / "launch/example_demo_hw.launch.py", ROOT / "launch/xmz_walk_hw.launch.py"):
            defaults = {}
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument":
                    defaults[node.args[0].value] = next(k.value.value for k in node.keywords if k.arg == "default_value")
            self.assertEqual(defaults["arms"], "true")
            self.assertEqual(defaults["head"], "true")

    def test_direct_onnx_session_is_cpu_single_thread(self):
        import onnxruntime as ort

        self.assertIsInstance(self.policy.session, ort.InferenceSession)
        self.assertFalse(hasattr(self.policy, "backend"))
        self.assertEqual(self.policy.session.get_providers(), ["CPUExecutionProvider"])
        self.assertEqual(self.policy.session.get_session_options().intra_op_num_threads, 1)
        self.assertTrue(np.isfinite(self.policy.infer(np.zeros((1, 960), dtype=np.float32))).all())

    def test_remote_timeout_and_rb_x_routes(self):
        np.testing.assert_array_equal(self.state.get_cmd_vel(self.ctx), 0)
        self.state.on_command(None)
        np.testing.assert_allclose(self.state.get_cmd_vel(self.ctx), [0.15, 0, 0])
        manifest = yaml.safe_load((ROOT / "mod.yaml").read_text())
        self.assertEqual(manifest["events"]["walk"], dict(slot="btn_1", value=1))
        self.assertEqual(self.state.inference_hz, 50)
        self.assertTrue(any(r["to"] == "walk" for r in manifest["routes"]))
        self.assertEqual({r["to"] for r in manifest["routes"] if r["from"] == "walk"},
                         {"com.bxi.basic_actions/pd_brake", "com.bxi.basic_actions/zero_position",
                          "com.bxi.basic_actions/zero_torque"})
        spec = importlib.util.spec_from_file_location("xmz_hw_launch", ROOT / "launch/xmz_walk_hw.launch.py")
        launcher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launcher)
        self.assertEqual(launcher.required_process_exit(SimpleNamespace(
            process_name="wait_for_arm_topology-3", returncode=0), None), [])
        self.assertEqual(launcher.required_process_exit(SimpleNamespace(
            process_name="ethercat_bind-2", returncode=0), None), [])
        for name, code in (("wait_for_arm_topology-3", 1), ("hardware_elf3-2", 0), ("feedback.py-4", 1)):
            self.assertEqual(len(launcher.required_process_exit(SimpleNamespace(
                process_name=name, returncode=code), None)), 1)

    def test_arm08_imu_frame_rotation_preserves_tilt_guard(self):
        self.state.imu_roll_pi = True
        self.ctx.current_quat_wxyz[:] = [0., 1., 0., 0.]
        self.ctx.current_omega[:] = [1., 2., 3.]
        original_quat = self.ctx.current_quat_wxyz.copy()
        original_omega = self.ctx.current_omega.copy()
        obs = self.state._observe(self.ctx, np.zeros(3)).reshape(10, -1)
        np.testing.assert_allclose(obs[-1, :6], [1., -2., -3., 0., 0., -1.], atol=1e-6)
        np.testing.assert_array_equal(self.ctx.current_quat_wxyz, original_quat)
        np.testing.assert_array_equal(self.ctx.current_omega, original_omega)
        self.ctx.current_quat_wxyz[:] = [-.000334516167641, .953324511647, .301462516189, .0170745104551]
        obs = self.state._observe(self.ctx, np.zeros(3)).reshape(10, -1)
        np.testing.assert_allclose(obs[-1, 3:6], [-.0327568221, .0096568552, -.9994166978], atol=1e-6)
        self.ctx.current_quat_wxyz[:] = [1., 0., 0., 0.]
        with self.assertRaisesRegex(ValueError, "tilt"):
            self.state.on_prepare(self.ctx, None)

    def test_partial_feedback_requires_all_sources_and_expires(self):
        module = importlib.import_module(self.state.__class__.__module__.rsplit(".", 1)[0] + ".feedback")
        merger = module.FeedbackMerger(ELF3_POLICY_JOINTS.names)
        names = ELF3_POLICY_JOINTS.names
        messages = []
        for group in (names[:15], names[15:22], names[22:]):
            msg = ActuatorStates()
            msg.name = list(group)
            msg.header.stamp.sec = 1
            for field in ("position", "velocity", "effort", "driver_temperature", "motor_temperature"):
                setattr(msg, field, [0.] * len(group))
            messages.append(msg)
        for msg in messages[:2]:
            self.assertTrue(merger.receive(msg, self.now, 1.))
        self.assertIsNone(merger.sample(self.now, 1.))
        self.assertTrue(merger.receive(messages[-1], self.now, 1.))
        messages[-1].driver_temperature[0] = float("nan")
        self.assertTrue(merger.receive(messages[-1], self.now, 1.))
        self.assertEqual(len(merger.sample(self.now, 1.).name), 29)
        self.assertIsNone(merger.sample(self.now, 1.))
        for msg in messages:
            msg.header.stamp.nanosec = 1
            merger.receive(msg, self.now + 1, 1.)
        self.assertIsNone(merger.sample(self.now + 200_000_000, 1.2))
        messages[-1].position[0] = float("nan")
        self.assertFalse(merger.receive(messages[-1], self.now, 1.))
        self.assertIsNone(merger.sample(self.now + 1, 1.))

    def test_body_only_feedback_is_rejected_by_arm_policy(self):
        module = importlib.import_module(self.state.__class__.__module__.rsplit(".", 1)[0] + ".feedback")
        merger = module.FeedbackMerger(ELF3_POLICY_JOINTS.names, arms=False)
        msg = ActuatorStates()
        msg.name = list(ELF3_POLICY_JOINTS.names[:15])
        msg.header.stamp.sec = 1
        for field in ("position", "velocity", "effort", "driver_temperature", "motor_temperature"):
            setattr(msg, field, [0.] * 15)
        self.assertTrue(merger.receive(msg, self.now, 1.))
        actual = merger.sample(self.now, 1.)
        self.assertEqual(actual.name, msg.name)
        self.assertEqual(len(actual.position), 15)
        self.assertIsNone(merger.sample(self.now + 100_000_001, 1.11))
        names = self.ctx.robot_joints.layout.names
        joints = self.ctx.robot_joints
        self.ctx.robot_joints = JointStateView(JointLayout.create(names[:15]),
                                              joints.position[:15], joints.velocity[:15], self.now)
        with self.assertRaisesRegex(ValueError, "29"):
            self.state.on_prepare(self.ctx, None)
        msg.position.pop()
        self.assertFalse(merger.receive(msg, self.now + 1, 1.))
        self.assertIsNone(merger.sample(self.now + 1, 1.))


if __name__ == "__main__":
    unittest.main()
