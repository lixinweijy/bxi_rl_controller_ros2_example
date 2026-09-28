"""MuJoCo entry for the unified Holosoma controller and A/B/Y test modes."""

import importlib.util

from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    path = get_package_share_path("bxi_example_py_elf3") / "launch/example_launch_vibration.py"
    spec = importlib.util.spec_from_file_location("holosoma_unified_sim_base", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(
        controller_executable_default="bxi_example_py_elf3_unified",
        controller_name_default="bxi_example_py_elf3_unified",
        joint_test_required_default="false",
        allow_hardware_without_joint_test_default="true",
        start_remote_controller_default="false",
        release_suspension_default="true",
    )
