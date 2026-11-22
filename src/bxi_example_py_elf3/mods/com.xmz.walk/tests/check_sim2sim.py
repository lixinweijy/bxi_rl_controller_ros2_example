#!/usr/bin/env python3
"""Compare the moving model to the supplied runner; pass its ELF4 source directory."""
import importlib.util
from pathlib import Path
import sys

import mujoco
import numpy as np

root = Path(__file__).resolve().parent.parent
source = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
from elf4_policy import Elf4Policy
spec = importlib.util.spec_from_file_location('xmz_reference', source/'amp_sim2sim_elf4_xmz.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
for command in ([0.,0.,0.], [.3,0.,0.], [0.,0.,.3]):
    sim = module.Elf4XmzSim2Sim(root/'assets/policy_1.onnx', source/'model/elf4_xmz.xml',root/'assets/config_1.json',np.array(command))
    policy = Elf4Policy(root/'assets/config_1.json',root/'assets/policy_1.onnx')
    lowest = float(sim.data.qpos[2])
    max_error = 0.
    for step in range(500):
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(sim.model,sim.data,mujoco.mjtObj.mjOBJ_BODY,sim.torso_id,velocity,1)
        obs = policy.observe(policy.names,sim.data.qpos[sim.qpos_adr],sim.data.qvel[sim.dof_adr],sim.data.qpos[3:7],velocity[:3],sim.command)
        ref = sim.observe()
        np.testing.assert_allclose(obs,ref,atol=2e-5,rtol=1e-5)
        target = policy.infer(obs)
        action = sim.infer(ref)
        expected = sim.default_pos + sim.action_scale*action.astype(np.float64)
        max_error = max(max_error,float(np.max(np.abs(target-expected))))
        np.testing.assert_allclose(target,expected,atol=2e-5,rtol=1e-5)
        assert not sim.step(action), f'fall command={command}, t={step*.02}'
        lowest=min(lowest,float(sim.data.qpos[2]))
    print(dict(command=command,frames=500,lowest_root_z=lowest,max_target_error=max_error),flush=True)
