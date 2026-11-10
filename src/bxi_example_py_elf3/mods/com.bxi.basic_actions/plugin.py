from bxi_example_py_elf3.framework.mod_api import ModDefinition, ModLoadContext

from .pd_brake_state import PdBrakeState
from .startup_state import StartupState
from .zero_torque_state import ZeroTorqueState
from .zero_position_state import ZeroPositionState


def create_mod(context: ModLoadContext) -> ModDefinition:
    return ModDefinition(
        state_factories={
            "startup": lambda state: StartupState(state.name, state.state_id),
            "zero_torque": lambda state: ZeroTorqueState(
                state.name, state.state_id
            ),
            "pd_brake": lambda state: PdBrakeState(state.name, state.state_id),
            "zero_position": lambda state: ZeroPositionState(state.name, state.state_id),
        }
    )
