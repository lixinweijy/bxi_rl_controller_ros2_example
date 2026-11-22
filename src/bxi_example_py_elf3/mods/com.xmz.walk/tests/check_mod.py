#!/usr/bin/env python3
"""Offline checks; arguments: package-share/source directory [source elf4_xmz]."""
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import yaml

PACKAGE = Path(sys.argv.pop(1)).resolve()
SOURCE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None
sys.path.insert(0, str(PACKAGE))
from bxi_example_py_elf3.framework.mod_api import JointLayout
from bxi_example_py_elf3.framework.joints import JointStateView
from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.policies.joints import ELF3_ISAAC_PARAMETERS

ROOT = Path(__file__).resolve().parent.parent


class XmzTest(unittest.TestCase):
    def setUp(self):
        self.runtime = load_mod_runtime(
            yaml.safe_load((PACKAGE / 'config/elf3_state_machine.yaml').read_text()),
            built_in_root=PACKAGE / 'mods', extra_roots=[])
        self.addCleanup(self.runtime.close)
        self.runtime.resources.load_startup()
        self.states = build_robot_states(self.runtime.config, self.runtime.state_factories)
        self.walk = self.states['com.xmz.walk/walk']
        self.walk._bind_logger(Mock())
        self.p = self.walk.policy.get()
        self.now = 1_000_000_000
        names = tuple(reversed(self.p.names)) + ('head_y_joint', 'head_z_joint')
        layout = JointLayout(names)
        joints = JointStateView(layout, np.r_[self.p.default[::-1], [0., 0.]],
                                np.zeros(31), timestamp_ns=self.now)
        self.ctx = SimpleNamespace(robot_joints=joints, robot_layout=layout,
            current_quat_wxyz=np.array([1., 0., 0., 0.]), current_omega=np.zeros(3),
            current_raw_cmd_vel=np.array([1., 1., 1.]), current_cmd_vel=np.zeros(3),
            speed_profiles=self.runtime.config['speed_profiles'],
            ros_node=SimpleNamespace(get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=self.now))))
        self.walk.on_prepare(self.ctx, None)

    def test_parameters_and_only_xmz_model(self):
        self.assertEqual({m.id for m in self.runtime.mods}, {'com.bxi.basic_actions', 'com.xmz.walk'})
        self.assertEqual(len(self.states), 5)
        self.assertTrue((ROOT/'assets/policy.onnx').is_file())
        self.assertEqual(len(self.p.disabled), 0)
        self.assertEqual(self.walk.inference_hz, 50)
        self.assertAlmostEqual(self.p.period, .02)
        self.assertFalse(hasattr(self.walk, 'moving_policy'))
        # Basic PD/zero keep the existing shared parameters when only policy code changes.
        for name in ('pd_brake', 'initial_pos'):
            frame = self.states['com.bxi.basic_actions/' + name].get_entry_frame(self.ctx)
            indices = [self.p.names.index(n) for n in frame.layout.names]
            np.testing.assert_allclose(frame.kp, np.asarray(ELF3_ISAAC_PARAMETERS.kp)[indices], rtol=1e-7)
            np.testing.assert_allclose(frame.kd, np.asarray(ELF3_ISAAC_PARAMETERS.kd)[indices], rtol=1e-7)
            expected = np.asarray(ELF3_ISAAC_PARAMETERS.default_position)[indices] if name == 'pd_brake' else 0
            np.testing.assert_allclose(frame.qpos, expected, atol=1e-7)
            np.testing.assert_array_equal(frame.vel, 0)
            np.testing.assert_array_equal(frame.torque, 0)
        if SOURCE:
            for name in ('config.json', 'policy.onnx'):
                self.assertEqual((ROOT/'assets'/name).read_bytes(), (SOURCE/name).read_bytes())

    def test_history_and_vector_scale_against_reference(self):
        self.check_policy_reference(self.p)

    def check_policy_reference(self, p):
        reference = np.zeros((10, 96), np.float32)
        session = p.session
        for i in range(64):
            q = p.default + np.sin(np.arange(29) + i) * .01
            dq = np.cos(np.arange(29) + i) * .1
            omega = np.array([.1, -.2, .3])
            command = np.array([(-1)**i * .8, .2, -.3])
            clipped_cmd = np.clip(command, p.ranges[:, 0], p.ranges[:, 1])
            frame = np.concatenate((omega, [0, 0, -1], clipped_cmd, q-p.default, dq, p.previous)).astype(np.float32)
            if i == 0: reference[:] = frame
            else: reference[:-1] = reference[1:]; reference[-1] = frame
            obs = p.observe(p.names[::-1], q[::-1], dq[::-1], [1,0,0,0], omega, command)
            np.testing.assert_array_equal(obs, np.clip(reference.reshape(1,-1),-100,100))
            action = session.run([p.output_name], {p.input_name: obs})[0][0]
            np.testing.assert_allclose(p.infer(obs), p.default+p.action_scale*np.clip(action,-100,100).astype(np.float64), atol=1e-7)
        action = np.linspace(-120,120,29,dtype=np.float32).reshape(1,29)
        p.session = SimpleNamespace(run=lambda *args: [action])
        np.testing.assert_allclose(p.infer(obs), p.default+p.action_scale*np.clip(action[0],-100,100).astype(np.float64))

    def test_preview_mapping_freshness_and_invalid_input(self):
        before = self.p.history.copy()
        frame = self.walk.sample_running_frame(self.ctx,.02,advance=False)
        np.testing.assert_allclose(frame.qpos,self.p.default,atol=1e-7)
        np.testing.assert_array_equal(self.p.history,before)
        self.assertTrue(np.isfinite(self.walk.sample_running_frame(self.ctx,.02,advance=True).qpos).all())
        self.now += 100_000_001
        with self.assertRaisesRegex(ValueError,'fresh'): self.walk._observe(self.ctx,np.zeros(3))
        self.now = 1_000_000_000
        self.ctx.current_quat_wxyz[:] = [0,1,0,0]
        with self.assertRaisesRegex(ValueError,'tilt'): self.walk.on_prepare(self.ctx,None)
        self.ctx.current_quat_wxyz[:] = [1,0,0,0]
        self.ctx.current_omega[0] = np.nan
        with self.assertRaisesRegex(ValueError,'Nonfinite'): self.walk.on_prepare(self.ctx,None)
        joints = self.ctx.robot_joints
        self.ctx.robot_joints = JointStateView(JointLayout(joints.layout.names[1:]),joints.position[1:],joints.velocity[1:],self.now)
        with self.assertRaisesRegex(ValueError,'29'): self.walk.on_prepare(self.ctx,None)

    def test_remote_and_safety_routes(self):
        self.walk.speed_profile_name = 'com.xmz.walk/walk'
        np.testing.assert_array_equal(self.walk.get_cmd_vel(self.ctx),0)
        self.walk.on_command(None)
        np.testing.assert_allclose(self.walk.get_cmd_vel(self.ctx),np.clip([.5,.5,1.57],self.p.ranges[:,0],self.p.ranges[:,1]))
        for raw in (-1., -.5, 0., .5, 1., 2., -2.):
            expected = np.clip(np.clip(raw*.5,-.5,.5),*self.p.ranges[0])
            self.ctx.current_raw_cmd_vel[0] = raw
            self.assertAlmostEqual(float(self.walk.get_cmd_vel(self.ctx)[0]), expected)
        self.walk.command_received = time.monotonic() - .6
        np.testing.assert_array_equal(self.walk.get_cmd_vel(self.ctx),0)
        np.testing.assert_array_equal(self.ctx.current_cmd_vel,0)
        m=yaml.safe_load((ROOT/'mod.yaml').read_text())
        self.assertEqual(m['events']['walk'],dict(slot='btn_1',value=1))
        self.assertEqual({r['to'] for r in m['routes'] if r['from']=='walk'},
            {'com.bxi.basic_actions/'+n for n in ('zero_torque','pd_brake','initial_pos')})
        zero=self.states['com.bxi.basic_actions/zero_torque'].get_entry_frame(self.ctx)
        self.assertEqual(len(zero.kp),31)
        for values in (zero.kp,zero.kd,zero.vel,zero.torque):np.testing.assert_array_equal(values,0)

    def test_single_policy_for_motion_stop_and_timeout(self):
        self.walk.speed_profile_name = 'com.xmz.walk/walk'
        original_infer = self.p.infer
        self.p.infer = Mock(wraps=original_infer)
        self.p.reset = Mock(wraps=self.p.reset)
        for raw in ([0,0,0], [.8,0,0], [0,0,0], [0,.5,0], [0,0,-.5], [-.5,0,0]):
            self.ctx.current_raw_cmd_vel[:] = raw
            self.walk.on_command(None)
            previous = self.p.previous.copy()
            history = self.p.history.copy()
            first = self.p.first
            frame = self.walk.sample_running_frame(self.ctx,.02,advance=True)
            np.testing.assert_allclose(self.p.history[-1,67:],previous)
            if not first:
                np.testing.assert_array_equal(self.p.history[:-1],history[1:])
            expected = np.clip(np.array(raw)*[.5,.5,1.57],self.p.ranges[:,0],self.p.ranges[:,1])
            np.testing.assert_allclose(self.p.history[-1,6:9],expected,atol=1e-7)
            np.testing.assert_allclose(frame.qpos,self.p.default+self.p.action_scale*self.p.previous,atol=1e-7)
            np.testing.assert_allclose(frame.kp,self.p.kp)
            np.testing.assert_allclose(frame.kd,self.p.kd)
        self.walk.command_received = time.monotonic()-.6
        self.walk.sample_running_frame(self.ctx,.02,advance=True)
        np.testing.assert_array_equal(self.p.history[-1,6:9],0)
        np.testing.assert_array_equal(self.ctx.current_cmd_vel,0)
        self.assertEqual(self.p.infer.call_count,7)
        self.p.reset.assert_not_called()

    def test_preview_and_prepare_reset(self):
        self.walk.sample_running_frame(self.ctx,.02,advance=True)
        history, previous = self.p.history.copy(),self.p.previous.copy()
        target = self.walk.target.copy()
        self.walk.on_command(None)
        frame = self.walk.sample_running_frame(self.ctx,.02,advance=False)
        np.testing.assert_array_equal(self.p.history,history)
        np.testing.assert_array_equal(self.p.previous,previous)
        np.testing.assert_allclose(frame.qpos,target)
        self.walk.on_prepare(self.ctx,None)
        self.assertTrue(self.p.first)
        np.testing.assert_array_equal(self.p.previous,0)

    def test_all_state_transitions_and_imu_lock(self):
        from unittest.mock import Mock
        from bxi_example_py_elf3.framework.mod_api import MotorFrame
        from bxi_example_py_elf3.framework.joints import JointCommandResolver, JointCommandDefaults, JointDefault
        from bxi_example_py_elf3.framework.runtime.state_machine import RobotStateMachine, RemoteEventAdapter
        resolver = JointCommandResolver(self.ctx.robot_layout, JointCommandDefaults({
            n: JointDefault(position=0.,kp=16.747,kd=1.066) for n in ('head_y_joint','head_z_joint')}))
        self.ctx.last_motor_frame = MotorFrame.empty(self.ctx.robot_layout)
        def resolve(source, output):
            resolver.resolve_into(source,output)
            return output
        self.ctx.resolve_motor_frame = resolve
        self.ctx.set_motor_target = lambda frame: resolve(frame,self.ctx.last_motor_frame)
        for state in self.states.values(): state._bind_logger(Mock())
        config = dict(self.runtime.config,graph={})
        machine = RobotStateMachine(self.ctx,config,self.states,logger=Mock())
        adapter = RemoteEventAdapter(config['remote_events'])
        for slot, target in [('btn_3','com.bxi.basic_actions/pd_brake'),
                             ('btn_1','com.xmz.walk/walk'),
                             ('btn_3','com.bxi.basic_actions/pd_brake'),
                             ('btn_1','com.xmz.walk/walk'),
                             ('btn_4','com.bxi.basic_actions/initial_pos'),
                             ('btn_3','com.bxi.basic_actions/pd_brake'),
                             ('btn_1','com.xmz.walk/walk'),
                             ('btn_2','com.bxi.basic_actions/zero_torque')]:
            adapter.extract_events(SimpleNamespace())
            events=adapter.extract_events(SimpleNamespace(**{slot:1}))
            for i in range(55):
                machine.update(.02, events if i==0 else [])
                machine.update_current_state(.02)
            self.assertEqual(machine.current_state_name,target)
            self.assertFalse(machine.in_transition)
            self.assertTrue(np.isfinite(self.ctx.last_motor_frame.qpos).all())
        np.testing.assert_array_equal(self.ctx.last_motor_frame.kp,0)
        np.testing.assert_array_equal(self.ctx.last_motor_frame.kd,0)
        self.assertTrue(machine.request_transition('com.bxi.basic_actions/imu_protection',force=True))
        machine.update_current_state(.02)
        np.testing.assert_array_equal(self.ctx.last_motor_frame.kp,20.)
        machine.update(.02,['com.bxi.basic_actions/pd_brake'])
        self.assertEqual(machine.current_state_name,'com.bxi.basic_actions/imu_protection')

    def test_relocated_mod(self):
        self.runtime.close()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'extra/private_git_mods'
            shutil.copytree(PACKAGE/'mods',root,ignore=shutil.ignore_patterns('__pycache__'))
            runtime=load_mod_runtime(yaml.safe_load((PACKAGE/'config/elf3_state_machine.yaml').read_text()),built_in_root=Path(directory)/'empty',extra_roots=[Path(directory)/'extra'])
            try:
                runtime.resources.load_startup()
                self.assertEqual(len(runtime.mods),2)
            finally:runtime.close()

    def test_unused_second_assets_are_not_required(self):
        self.runtime.close()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/'mods'
            shutil.copytree(PACKAGE/'mods',root,ignore=shutil.ignore_patterns('__pycache__'))
            for name in ('policy_1.onnx','config_1.json'):
                (root/'com.xmz.walk/assets'/name).unlink(missing_ok=True)
            runtime = load_mod_runtime(yaml.safe_load((PACKAGE/'config/elf3_state_machine.yaml').read_text()),built_in_root=root,extra_roots=[])
            try:
                runtime.resources.load_startup()
                walk = build_robot_states(runtime.config,runtime.state_factories)['com.xmz.walk/walk']
                self.assertFalse(hasattr(walk,'moving_policy'))
                walk.on_prepare(self.ctx,None)
                self.assertTrue(np.isfinite(walk.sample_running_frame(self.ctx,.02,advance=True).qpos).all())
            finally:
                runtime.close()

    def test_telemetry_and_publish_failure(self):
        self.ctx.ros_node.create_subscription = Mock()
        publishers = {}
        def create_publisher(kind, topic, qos):
            publishers[topic] = Mock()
            publishers[topic].get_subscription_count.return_value = 1
            return publishers[topic]
        self.ctx.ros_node.create_publisher = create_publisher
        self.ctx.ros_node.destroy_publisher = Mock()
        self.ctx.ros_node.destroy_subscription = Mock()
        self.walk.on_bind(self.ctx)
        self.walk.sample_running_frame(self.ctx,.02,advance=True)
        self.assertEqual(set(publishers),{'/xmz/policy/'+name for name in ('observation','action_raw','action_clipped','target')})
        for publisher in publishers.values():publisher.publish.assert_called_once()
        publishers['/xmz/policy/target'].publish.side_effect = RuntimeError('recorder unavailable')
        self.assertTrue(np.isfinite(self.walk.sample_running_frame(self.ctx,.02,advance=True).qpos).all())
        self.walk.on_unbind(self.ctx)
        self.assertEqual(self.ctx.ros_node.destroy_publisher.call_count,4)


if __name__ == '__main__':
    unittest.main()
