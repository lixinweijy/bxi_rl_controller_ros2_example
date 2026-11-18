from bxi_example_py_elf3.framework.mod_api import ModDefinition
from .zero_torque_state import ZeroTorqueState
from .imu_protection_state import ImuProtectionState
from .pd_brake_state import PdBrakeState
from .initial_pos_state import InitialPosState


def create_mod(context):
    return ModDefinition(state_factories={
        "zero_torque": lambda state: ZeroTorqueState(state.name, state.state_id),
        "imu_protection": lambda state: ImuProtectionState(
            state.name, state.state_id, kp=state.float_param("kp", 20.0),
            kd=state.float_param("kd", 1.0)),
        "pd_brake": lambda state: PdBrakeState(state.name, state.state_id),
        "initial_pos": lambda state: InitialPosState(state.name, state.state_id),
    })
