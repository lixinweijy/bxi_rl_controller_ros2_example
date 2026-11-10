from __future__ import annotations
from typing import TYPE_CHECKING

import numpy as np
from bxi_example_py_elf3.framework.inference.policy import JointPolicy
from bxi_example_py_elf3.framework.mod_api import (
    ResourceHandle,
    RobotControlState,
)
from bxi_example_py_elf3.framework.mod_api.transition import (
    EntryFrameProvider,
    MotorFrame,
    RunningFrameProvider,
)

from .policy import ELF4_COMMAND_MAX, ELF4_COMMAND_MIN  # noqa: TID252

if TYPE_CHECKING:
    from bxi_example_py_elf3.framework.mod_api import RobotControlContext


class Elf4State(RobotControlState, EntryFrameProvider, RunningFrameProvider):
    """Run the ELF4 policy on the state scheduler's 50 Hz timeline."""

    def __init__(
        self,
        name: str,
        state_id: int,
        policy: ResourceHandle[JointPolicy],
    ) -> None:
        super().__init__(name, state_id, resources=(policy,))
        self._policy = policy
        self._command = np.zeros(3, dtype=np.float32)

    @property
    def policy(self) -> JointPolicy:
        return self._policy.get()

    def get_cmd_vel(self, ctx: RobotControlContext) -> np.ndarray:
        np.clip(
            self._profile_cmd_vel(ctx),
            ELF4_COMMAND_MIN,
            ELF4_COMMAND_MAX,
            out=self._command,
        )
        return self._publish_cmd_vel(ctx, self._command)

    def on_prepare(self, ctx: RobotControlContext, from_state) -> None:
        # The framework resets and previews once without advancing history
        # or the previous action before the transition.
        ctx.preheat_model(self.policy, command=self.get_cmd_vel(ctx))

    def on_enter(self, ctx: RobotControlContext) -> None:
        self._command.fill(0.0)

    def get_entry_frame(self, ctx: RobotControlContext) -> MotorFrame:
        return self._motor_frame_from_target(ctx, self.policy.output.joints)

    def sample_running_frame(
        self,
        ctx: RobotControlContext,
        dt: float,
        *,
        advance: bool,
    ) -> MotorFrame:
        self.get_cmd_vel(ctx)
        output = self.policy.step(ctx.inference_frame, dt, advance=advance)
        return self._motor_frame_from_target(ctx, output.joints)

    def on_update(self, ctx: RobotControlContext, dt: float) -> None:
        self._apply_frame(
            ctx, self.sample_running_frame(ctx, dt, advance=True)
        )
