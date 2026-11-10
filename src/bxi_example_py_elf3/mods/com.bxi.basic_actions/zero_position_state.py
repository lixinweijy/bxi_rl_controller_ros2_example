from .pd_brake_state import PdBrakeState


class ZeroPositionState(PdBrakeState):
    def _frame(self, ctx):
        frame = super()._frame(ctx)
        frame.qpos.fill(0.0)
        return frame
