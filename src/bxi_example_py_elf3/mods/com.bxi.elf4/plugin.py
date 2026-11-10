from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .policy import Elf4Policy  # noqa: TID252
from .state import Elf4State  # noqa: TID252

ELF4_POLICY = ResourceKey[Elf4Policy]("com.bxi.elf4/policy")


def _load_policy(resource: ResourceLoadContext) -> Elf4Policy:
    return Elf4Policy(
        resource.asset("assets/policy.onnx"),
        contract=resource.asset("assets/deploy.json"),
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(ELF4_POLICY, _load_policy, policy="on_demand")
    policy = context.resource(ELF4_POLICY)
    return ModDefinition(
        state_factories={
            "elf4": lambda state: Elf4State(
                state.name, state.state_id, policy
            ),
        }
    )


__all__ = ["ELF4_POLICY", "create_mod"]
