#!/usr/bin/env python3
"""Offline ARM-05 integration: package-source/share path, optional remote YAML."""
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import yaml

PACKAGE = Path(sys.argv.pop(1)).resolve()
REMOTE = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None
sys.path.insert(0, str(PACKAGE))
from bxi_example_py_elf3.framework.mod_api import JointLayout, MotorFrame
from bxi_example_py_elf3.framework.joints import (
    JointStateView, JointCommandResolver, JointCommandDefaults, JointDefault,
)
from bxi_example_py_elf3.framework.runtime.mod_loader import load_mod_runtime
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.framework.runtime.state_machine import RobotStateMachine, RemoteEventAdapter

ROOT = Path(__file__).resolve().parent.parent


class StandTest(unittest.TestCase):
    def setUp(self):
        self.runtime = load_mod_runtime(
            yaml.safe_load((PACKAGE/'config/elf3_state_machine.yaml').read_text()),
            built_in_root=PACKAGE/'mods', extra_roots=[])
        self.addCleanup(self.runtime.close)
        self.runtime.resources.load_startup()
        self.states = build_robot_states(self.runtime.config, self.runtime.state_factories)
        self.stand = self.states['com.xmz.stand/stand']
        for state in self.states.values():
            state._bind_logger(Mock())
        self.assertEqual(self.stand.policy.status, 'unloaded')
        self.stand.policy.request()
        deadline = time.monotonic() + 15
        while self.stand.policy.status == 'loading' and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertEqual(self.stand.policy.status, 'ready', str(self.stand.policy.error))
        self.p = self.stand.policy.get()
        self.now = 1_000_000_000
        names = self.p.names[::-1] + ('head_y_joint', 'head_z_joint')
        layout = JointLayout(names)
        joints = JointStateView(layout, np.r_[self.p.nominal[::-1], [0., 0.]],
                                np.zeros(31), timestamp_ns=self.now)
        self.ctx = SimpleNamespace(robot_joints=joints, robot_layout=layout,
            current_quat_wxyz=np.array([1., 0., 0., 0.]), current_omega=np.zeros(3),
            current_raw_cmd_vel=np.zeros(3), current_cmd_vel=np.zeros(3),
            speed_profiles=self.runtime.config['speed_profiles'],
            ros_node=SimpleNamespace(get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=self.now))))
        self.stand.on_prepare(self.ctx, None)

    def test_export_and_routes(self):
        self.assertEqual({m.id for m in self.runtime.mods},
                         {'com.bxi.basic_actions', 'com.xmz.walk', 'com.xmz.stand'})
        info = json.loads((ROOT/'assets/model_info_tang.json').read_bytes())
        for name, key in [('policy_tang.onnx', 'onnx_sha256'),
                          ('controller_parameters.json', 'controller_parameters_sha256'),
                          ('standing_arm_motion.npz', 'arm_database_sha256')]:
            self.assertEqual(hashlib.sha256((ROOT/'assets'/name).read_bytes()).hexdigest(), info[key])
        self.assertEqual(self.stand.inference_hz, 50)
        manifest = yaml.safe_load((ROOT/'mod.yaml').read_text())
        self.assertEqual(manifest['events']['stand'], {'slot': 'btn_5', 'value': 1})
        self.assertEqual(manifest['states']['stand']['params']['arm_speed'], .5)
        self.assertIn({'from': 'com.xmz.walk/walk', 'event': 'stand',
                       'to': 'stand', 'transition': 'posture_blend'}, manifest['routes'])
        self.assertEqual(manifest['transition_profiles']['posture_blend'],
                         {'type': 'running_blend', 'duration': .6, 'curve': 'smoothstep',
                          'sample_from': False, 'sample_to': False})
        if REMOTE:
            remote = yaml.safe_load(REMOTE.read_text())
            rule = next(r for r in remote['outputs']['level'] if r['output'] == 'btn_5=1')
            self.assertEqual(rule['when']['any'][0], ['shoulder.left_event', 'button.x_event',
                {'released':'shoulder.right_event'}, {'released':'trigger.left_event'},
                {'released':'trigger.right_event'}])

    def test_history_and_two_arm_cycles(self):
        p = self.p
        config = json.loads((ROOT/'assets/controller_parameters.json').read_bytes())
        lower, upper = np.asarray(config['soft_position_limits_rad']).T
        history = np.zeros((10, 54), np.float32)
        indices = [p.names.index(n) for n in config['policy_joint_names']]
        durations = []
        for i in range(3365):
            q = p.nominal + np.sin(np.arange(29) + i)*.001
            dq = np.cos(np.arange(29) + i)*.01
            omega = np.array([.01, -.02, .03])
            frame = np.r_[omega, [0,0,-1], [0,0,0], q[indices]-p.nominal[indices],
                          dq[indices]*.1, p.previous].astype(np.float32)
            frame = np.clip(frame,-100,100)
            if i == 0:
                history[:] = frame
            else:
                history[:-1] = history[1:]
                history[-1] = frame
            started = time.monotonic()
            target = p.step(q, dq, [1,0,0,0], omega)
            durations.append(time.monotonic()-started)
            if i < 64:
                np.testing.assert_allclose(p.history, history, atol=1e-7)
                action = p._session.run(p._outputs, {p._input: history.reshape(1,540)})[0][0]
                np.testing.assert_allclose(p.previous, action, atol=1e-6)
            self.assertTrue(np.isfinite(target).all())
            self.assertTrue(np.all(target >= lower-1e-6) and np.all(target <= upper+1e-6))
        print('ONNX+arm offline tick ms p95/max:',
              np.percentile(durations,95)*1000, max(durations)*1000)
        self.assertGreater(p._arm_tick, 3360)
        with self.assertRaises(ValueError):
            p.step(np.full(29, np.nan), dq, [1,0,0,0], omega)

    def test_mapping_preview_and_fault_zero(self):
        p, state, ctx = self.p, self.stand, self.ctx
        before = p.history.copy()
        frame = state.sample_running_frame(ctx,.02,advance=False)
        np.testing.assert_array_equal(p.history,before)
        np.testing.assert_allclose(frame.qpos,p.nominal)
        self.assertEqual(frame.layout.names,p.names)
        state.sample_running_frame(ctx,.02,advance=True)
        np.testing.assert_array_equal(state._position,p.nominal)
        self.assertTrue(state.is_available(ctx))
        self.now += 100_000_001
        self.assertFalse(state.is_available(ctx))
        ctx.set_motor_target = Mock()
        ctx.request_state = Mock()
        state.on_update(ctx,.02)
        for values in ctx.set_motor_target.call_args.args[0]:
            np.testing.assert_array_equal(values,0)
        ctx.request_state.assert_called_once_with('com.bxi.basic_actions/zero_torque',
            trigger='xmz_stand_fault', transition='instant')
        self.now = 1_000_000_000
        ctx.current_quat_wxyz[:] = [0,1,0,0]
        self.assertFalse(state.is_available(ctx))
        with self.assertRaisesRegex(ValueError,'tilt'):
            state.on_prepare(ctx,None)
        ctx.current_quat_wxyz[:] = [1,0,0,0]
        ctx.robot_joints = JointStateView(JointLayout(p.names[1:]),p.nominal[1:],
                                         np.zeros(28),self.now)
        self.assertFalse(state.is_available(ctx))
        with self.assertRaisesRegex(ValueError,'requires feedback'):
            state.on_prepare(ctx,None)

    def test_state_machine_and_imu_lock(self):
        ctx = self.ctx
        resolver = JointCommandResolver(ctx.robot_layout, JointCommandDefaults({
            n:JointDefault(position=0.,kp=16.747,kd=1.066) for n in ('head_y_joint','head_z_joint')}))
        ctx.last_motor_frame = MotorFrame.empty(ctx.robot_layout)
        def resolve(frame,output):
            resolver.resolve_into(frame,output)
            return output
        ctx.resolve_motor_frame = resolve
        ctx.set_motor_target = lambda frame: resolve(frame,ctx.last_motor_frame)
        config = dict(self.runtime.config,graph={})
        machine = RobotStateMachine(ctx,config,self.states,logger=Mock())
        ctx.request_state = machine.request_transition
        adapter = RemoteEventAdapter(config['remote_events'])
        for slot,target in [('btn_3','com.bxi.basic_actions/pd_brake'),
                            ('btn_1','com.xmz.walk/walk'),
                            ('btn_5','com.xmz.stand/stand'),
                            ('btn_3','com.bxi.basic_actions/pd_brake'),
                            ('btn_5','com.xmz.stand/stand'),
                            ('btn_4','com.bxi.basic_actions/initial_pos'),
                            ('btn_3','com.bxi.basic_actions/pd_brake'),
                            ('btn_5','com.xmz.stand/stand'),
                            ('btn_2','com.bxi.basic_actions/zero_torque')]:
            adapter.extract_events(SimpleNamespace())
            events = adapter.extract_events(SimpleNamespace(**{slot:1}))
            from_walk = machine.current_state_name == 'com.xmz.walk/walk'
            for i in range(60):
                machine.update(.02,events if i==0 else [])
                if from_walk and i == 0:
                    self.assertTrue(machine.in_transition)
                machine.update_current_state(.02)
            self.assertEqual(machine.current_state_name,target)
            self.assertFalse(machine.in_transition)
            self.assertTrue(np.isfinite(ctx.last_motor_frame.qpos).all())
        np.testing.assert_array_equal(ctx.last_motor_frame.kp,0)
        self.assertTrue(machine.request_transition('com.bxi.basic_actions/imu_protection',force=True))
        machine.update_current_state(.02)
        machine.update(.02,['com.xmz.stand/stand'])
        self.assertEqual(machine.current_state_name,'com.bxi.basic_actions/imu_protection')


if __name__ == '__main__':
    unittest.main()
