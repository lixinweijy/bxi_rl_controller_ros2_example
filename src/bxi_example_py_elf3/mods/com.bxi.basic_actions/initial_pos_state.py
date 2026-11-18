from __future__ import annotations

from typing import TYPE_CHECKING
import numpy as np

from bxi_example_py_elf3.framework.mod_api import RobotControlState
from bxi_example_py_elf3.policies.joints import ELF3_POLICY_JOINTS, ELF3_ISAAC_PARAMETERS
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class InitialPosState(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    _TARGET_POSITION = np.zeros(29, dtype=np.float32)
    _PARAMETERS = ELF3_ISAAC_PARAMETERS.select(ELF3_POLICY_JOINTS)
    _KP = _PARAMETERS.kp
    _KD = _PARAMETERS.kd

    def _frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._motor_frame(
            ctx,
            self._TARGET_POSITION,
            self._KP,
            self._KD,
            layout=ELF3_POLICY_JOINTS,
        )

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._frame(ctx)

    def sample_running_frame(
        self, ctx: RobotControlContext, dt: float, *, advance: bool
    ) -> MotorFrame:
        return self._frame(ctx)

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        self._apply_frame(ctx, self._frame(ctx))
