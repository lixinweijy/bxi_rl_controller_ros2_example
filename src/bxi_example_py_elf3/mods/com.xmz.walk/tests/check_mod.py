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
        self.assertEqual({p.name for p in (PACKAGE / 'mods').rglob('*.onnx')}, {'policy.onnx', 'policy_1.onnx'})
        self.assertEqual(len(self.p.disabled), 0)
        self.assertEqual(self.walk.inference_hz, 50)
        self.assertAlmostEqual(self.p.period, .02)
        moving = self.walk.moving_policy.get()
        self.assertEqual(moving.names, self.p.names)
        self.assertEqual(len(moving.disabled), 0)
        self.assertAlmostEqual(moving.period, .02)
        # Basic PD/zero retain the already deployed shared parameters.
        for actual, expected in ((ELF3_ISAAC_PARAMETERS.default_position, moving.default),
                                 (ELF3_ISAAC_PARAMETERS.kp, moving.kp),
                                 (ELF3_ISAAC_PARAMETERS.kd, moving.kd),
                                 (ELF3_ISAAC_PARAMETERS.action_scale, moving.action_scale)):
            np.testing.assert_allclose(actual, expected, rtol=1e-7)
        for name in ('pd_brake', 'initial_pos'):
            frame = self.states['com.bxi.basic_actions/' + name].get_entry_frame(self.ctx)
            indices = [self.p.names.index(n) for n in frame.layout.names]
            np.testing.assert_allclose(frame.kp, np.asarray(moving.kp)[indices], rtol=1e-7)
            np.testing.assert_allclose(frame.kd, np.asarray(moving.kd)[indices], rtol=1e-7)
            np.testing.assert_allclose(frame.qpos, moving.default[indices] if name == 'pd_brake' else 0, atol=1e-7)
            np.testing.assert_array_equal(frame.vel, 0)
            np.testing.assert_array_equal(frame.torque, 0)
        if SOURCE:
            for name in ('config.json', 'policy.onnx', 'config_1.json', 'policy_1.onnx'):
                self.assertEqual((ROOT/'assets'/name).read_bytes(), (SOURCE/name).read_bytes())

    def test_history_and_vector_scale_against_reference(self):
        for p in (self.p, self.walk.moving_policy.get()):
            with self.subTest(config=p.cfg['command_ranges']):
                self.check_policy_reference(p)

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
        np.testing.assert_allclose(self.walk.get_cmd_vel(self.ctx),[.3,.5,1.57])
        for raw, expected in ((-1., -.3), (-.5, -.25), (0., 0.), (.5, .25), (1., .3), (2., .3), (-2., -.3)):
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

    def test_automatic_switch_history_blend_and_timeout(self):
        self.walk.speed_profile_name = 'com.xmz.walk/walk'
        moving = self.walk.moving_policy.get()
        self.ctx.current_raw_cmd_vel[:] = 0
        self.walk.on_command(None)
        self.walk.sample_running_frame(self.ctx, .02, advance=True)
        self.assertIs(self.walk.active_policy, self.p)
        before = (self.walk.target.copy(), self.walk.kp.copy(), self.walk.kd.copy())
        moving.history.fill(999)
        self.ctx.current_raw_cmd_vel[:] = [.8, 0, 0]
        self.walk.on_command(None)
        frame = self.walk.sample_running_frame(self.ctx, .02, advance=True)
        self.assertIs(self.walk.active_policy, moving)
        for actual, old in zip((frame.qpos, frame.kp, frame.kd), before):
            np.testing.assert_allclose(actual, old)
        np.testing.assert_array_equal(moving.history, np.repeat(moving.history[-1:],10,axis=0))
        np.testing.assert_allclose(moving.history[-1, 67:], (before[0]-moving.default)/moving.action_scale, atol=1e-6)
        np.testing.assert_allclose(moving.history[-1, 6:9], [.3,0,0])
        for _ in range(16):
            self.walk.on_command(None)
            frame = self.walk.sample_running_frame(self.ctx,.02,advance=True)
        np.testing.assert_allclose(frame.qpos,moving.default+moving.action_scale*moving.previous,atol=1e-7)
        np.testing.assert_allclose(frame.kp,moving.kp)
        np.testing.assert_allclose(frame.kd,moving.kd)
        # Zero-valued messages are still arriving: select idle on content.
        self.ctx.current_raw_cmd_vel[:] = 0
        self.walk.on_command(None)
        old = frame.qpos.copy()
        self.walk.sample_running_frame(self.ctx,.02,advance=True)
        self.assertIs(self.walk.active_policy,self.p)
        np.testing.assert_allclose(self.walk.target,old)
        np.testing.assert_allclose(self.p.history[-1,67:],(old-self.p.default)/self.p.action_scale,atol=1e-6)
        for _ in range(16):
            self.walk.on_command(None)
            frame = self.walk.sample_running_frame(self.ctx,.02,advance=True)
        np.testing.assert_allclose(frame.kp,self.p.kp)
        np.testing.assert_allclose(frame.kd,self.p.kd)
        np.testing.assert_allclose(frame.qpos,self.p.default+self.p.action_scale*self.p.previous,atol=1e-7)
        # Lateral-only and yaw-only commands also count; stale nonzero input does not.
        for command in ([0,.5,0], [0,0,-.5], [-.5,0,0]):
            self.ctx.current_raw_cmd_vel[:] = command
            self.walk.on_command(None)
            self.walk.sample_running_frame(self.ctx,.02,advance=True)
            self.assertIs(self.walk.active_policy,moving)
            self.walk.command_received = time.monotonic()-.6
            self.walk.sample_running_frame(self.ctx,.02,advance=True)
            self.assertIs(self.walk.active_policy,self.p)
            np.testing.assert_array_equal(self.ctx.current_cmd_vel,0)

    def test_reversal_preview_and_prepare_reset(self):
        self.walk.speed_profile_name = 'com.xmz.walk/walk'
        for command in ([.5,0,0], [0,0,0], [0,0,.5]):
            self.ctx.current_raw_cmd_vel[:] = command
            self.walk.on_command(None)
            active, elapsed = self.walk.active_policy,self.walk.switch_elapsed
            histories = [p.history.copy() for p in (self.p,self.walk.moving_policy.get())]
            before = [v.copy() for v in (self.walk.target,self.walk.kp,self.walk.kd)]
            self.walk.sample_running_frame(self.ctx,.02,advance=False)
            self.assertIs(self.walk.active_policy,active)
            self.assertEqual(self.walk.switch_elapsed,elapsed)
            for p, h in zip((self.p,self.walk.moving_policy.get()),histories):
                np.testing.assert_array_equal(p.history,h)
            frame = self.walk.sample_running_frame(self.ctx,.02,advance=True)
            for new,old in zip((frame.qpos,frame.kp,frame.kd),before):
                np.testing.assert_allclose(new,old)
        self.walk.on_prepare(self.ctx,None)
        self.assertIs(self.walk.active_policy,self.p)
        self.assertIsNone(self.walk.switch_from)
        for p in (self.p,self.walk.moving_policy.get()):
            self.assertTrue(p.first)
            np.testing.assert_array_equal(p.previous,0)
        self.walk.moving_policy.get().names = tuple(reversed(self.p.names))
        with self.assertRaisesRegex(ValueError,'same joint order'):
            self.walk.on_prepare(self.ctx,None)

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

    def test_missing_moving_model_refuses_startup(self):
        self.runtime.close()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/'mods'
            shutil.copytree(PACKAGE/'mods',root,ignore=shutil.ignore_patterns('__pycache__'))
            (root/'com.xmz.walk/assets/policy_1.onnx').unlink()
            with self.assertRaisesRegex(RuntimeError,'policy_1.onnx'):
                load_mod_runtime(yaml.safe_load((PACKAGE/'config/elf3_state_machine.yaml').read_text()),built_in_root=root,extra_roots=[])

    def test_active_telemetry_and_publish_failure(self):
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
        active = publishers['/xmz/policy/active']
        self.assertEqual(active.publish.call_args.args[0].data,'policy.onnx')
        self.walk.speed_profile_name = 'com.xmz.walk/walk'
        self.walk.on_command(None)
        self.walk.sample_running_frame(self.ctx,.02,advance=True)
        self.assertEqual(active.publish.call_args.args[0].data,'policy_1.onnx')
        active.publish.side_effect = RuntimeError('recorder unavailable')
        self.assertTrue(np.isfinite(self.walk.sample_running_frame(self.ctx,.02,advance=True).qpos).all())
        self.walk.on_unbind(self.ctx)
        self.assertEqual(self.ctx.ros_node.destroy_publisher.call_count,5)


if __name__ == '__main__':
    unittest.main()
