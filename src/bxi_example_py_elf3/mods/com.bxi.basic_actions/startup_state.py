from .pd_brake_state import PdBrakeState


class StartupState(PdBrakeState):
    def _frame(self, ctx):
        # Omitted head joints use the adapter's zero-position PD defaults.
        frame = super()._frame(ctx)
        frame.kp.fill(0.0)
        frame.kd.fill(0.0)
        return frame
