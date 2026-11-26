from bxi_example_py_elf3.framework.mod_api import ModDefinition, ResourceKey

from .policy import DEFAULT_ARM_CLIP, DEFAULT_ARM_SPEED, StandPolicy
from .state import XmzStandState


POLICY = ResourceKey[StandPolicy]("com.xmz.stand/policy")


def load_policy(context):
    return StandPolicy(
        context.asset("assets/controller_parameters.json"),
        context.asset("assets/policy_tang.onnx"),
        context.asset("assets/standing_arm_motion.npz"),
        clap_path=context.asset("assets/clap_motion.npz"),
        wave_path=context.asset("assets/wave_motion.npz"),
    )


def create_mod(context):
    context.register_resource(POLICY, load_policy, policy="on_demand")
    policy = context.resource(POLICY)
    return ModDefinition(state_factories={
        "stand": lambda state: XmzStandState(
            state.name, state.state_id, policy,
            play_arm_motion=state.bool_param("play_arm_motion", True),
            arm_clip=state.string_param("arm_clip", DEFAULT_ARM_CLIP),
            arm_speed=state.float_param("arm_speed", DEFAULT_ARM_SPEED),
        ),
    })
