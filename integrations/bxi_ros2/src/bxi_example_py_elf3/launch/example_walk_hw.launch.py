"""Compatibility entry: hardware walking uses the unified Holosoma controller."""

import importlib.util

from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    path = get_package_share_path("bxi_example_py_elf3") / "launch/example_launch_unified_hw.launch.py"
    spec = importlib.util.spec_from_file_location("holosoma_unified_hw", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
