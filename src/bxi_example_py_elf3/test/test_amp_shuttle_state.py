from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

def test_amp_shuttle_alternates_every_second():
    path = (
        Path(__file__).parents[1]
        / "mods/com.bxi.basic_actions/amp_run_state.py"
    )
    spec = spec_from_file_location("amp_run_state", path)
    assert spec and spec.loader
    module = module_from_spec(spec)
    spec.loader.exec_module(module)

    state = module.AmpShuttleState("shuttle", 1, None)
    for elapsed, expected in ((0.0, 0.5), (0.999, 0.5), (1.0, -0.5), (2.0, 0.5)):
        state._elapsed = elapsed
        assert state._target_vx() == expected
