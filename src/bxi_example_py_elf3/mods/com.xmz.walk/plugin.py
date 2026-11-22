from bxi_example_py_elf3.framework.mod_api import ModDefinition, ResourceKey

from .elf4_policy import Elf4Policy
from .state import XmzWalkState


POLICY = ResourceKey[Elf4Policy]("com.xmz.walk/policy")
MOVING_POLICY = ResourceKey[Elf4Policy]("com.xmz.walk/moving_policy")


def load_policy(context):
    return Elf4Policy(context.asset("assets/config.json"), context.asset("assets/policy.onnx"))


def load_moving_policy(context):
    return Elf4Policy(context.asset("assets/config_1.json"), context.asset("assets/policy_1.onnx"))


def create_mod(context):
    context.register_resource(POLICY, load_policy, policy="startup")
    context.register_resource(MOVING_POLICY, load_moving_policy, policy="startup")
    policy = context.resource(POLICY)
    moving_policy = context.resource(MOVING_POLICY)
    return ModDefinition(state_factories={
        "walk": lambda state: XmzWalkState(state.name, state.state_id, policy, moving_policy),
    })
