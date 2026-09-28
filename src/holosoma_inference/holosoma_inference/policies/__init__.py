from .base import BasePolicy
from .dual_mode import DualModePolicy
from .locomotion import LocomotionPolicy


def __getattr__(name):
    if name == "WholeBodyTrackingPolicy":
        from .wbt import WholeBodyTrackingPolicy
        return WholeBodyTrackingPolicy
    raise AttributeError(name)

__all__ = ["BasePolicy", "DualModePolicy", "LocomotionPolicy", "WholeBodyTrackingPolicy"]
