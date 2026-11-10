from __future__ import annotations

from bxi_example_py_elf3.framework.mod_api import (
    ModDefinition,
    ModLoadContext,
    ResourceKey,
    ResourceLoadContext,
)

from .dwaq_policy import Elf4DwaqPolicy  # noqa: TID252
from .policy import Elf4Policy  # noqa: TID252
from .state import Elf4State  # noqa: TID252

ELF4_POLICY = ResourceKey[Elf4Policy]("com.bxi.elf4/policy")
ELF4_DWAQ_POLICY = ResourceKey[Elf4DwaqPolicy]("com.bxi.elf4/dwaq_1500_policy")


def _load_policy(resource: ResourceLoadContext) -> Elf4Policy:
    return Elf4Policy(
        resource.asset("assets/policy.onnx"),
        contract=resource.asset("assets/deploy.json"),
    )


def create_mod(context: ModLoadContext) -> ModDefinition:
    context.register_resource(ELF4_POLICY, _load_policy, policy="on_demand")
    context.register_resource(
        ELF4_DWAQ_POLICY,
        lambda resource: Elf4DwaqPolicy(
            resource.asset("assets/policy_53500_dwaq.onnx")
        ),
        policy="on_demand",
    )
    policy = context.resource(ELF4_POLICY)
    dwaq_policy = context.resource(ELF4_DWAQ_POLICY)
    return ModDefinition(
        state_factories={
            "elf4": lambda state: Elf4State(
                state.name, state.state_id, policy
            ),
            "elf4_dwaq_1500": lambda state: Elf4State(
                state.name, state.state_id, dwaq_policy
            ),
        }
    )


__all__ = ["ELF4_POLICY", "ELF4_DWAQ_POLICY", "create_mod"]
