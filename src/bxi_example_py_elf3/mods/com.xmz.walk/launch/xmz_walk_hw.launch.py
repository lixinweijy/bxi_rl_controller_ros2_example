"""ARM-08 deployment: the existing new body/arm drivers and xmz walking Mod."""
from pathlib import Path
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def required_process_exit(event, context):
    # The included dual_robot launcher intentionally exits its successful topology probe.
    if event.returncode == 0 and event.process_name.rsplit("-", 1)[0] in ("ethercat_bind", "wait_for_arm_topology"):
        return []
    return [EmitEvent(event=Shutdown(reason=f"xmz process exited: {event.process_name}"))]


def generate_launch_description():
    arms = LaunchConfiguration("arms")
    head = LaunchConfiguration("head")
    for entry in Path("/proc").iterdir():
        if entry.name.isdecimal():
            try:
                name = (entry / "comm").read_text().strip()
            except (OSError, ProcessLookupError):
                continue
            if name in {"hardware_elf3", "dual_kaixuan_ar", "dual_kaixuan_arm", "hw_lib_test_elf"} or name.startswith("hw_lib_test"):
                raise RuntimeError(f"Existing hardware owner pid={entry.name} ({name}); stop it before Start")
    # Contract: this launcher lives in its own portable Mod's launch/ directory.
    root = Path(__file__).resolve().parent.parent
    assert (root / "mod.yaml").is_file()
    package = Path(get_package_share_directory("bxi_example_py_elf3"))
    config = yaml.safe_load((package / "config/elf3_state_machine.yaml").read_text())
    config["mod_paths"] = []  # Both Mods are installed in the built-in directory.
    generated = Path.home() / ".ros/xmz_walk/state_machine.yaml"
    generated.parent.mkdir(parents=True, exist_ok=True)
    generated.write_text(yaml.safe_dump(config, allow_unicode=True))
    merger = ExecuteProcess(cmd=[sys.executable, str(root / "feedback.py"), "--ros-args", "-p", ["arms:=", arms], "-p", ["head:=", head]], output="screen")
    example = Node(package="bxi_example_py_elf3", executable="bxi_example_py_elf3_demo", output="screen",
                   parameters=[{"/topic_prefix": "hardware/", "/state_machine_config": str(generated),
                                "xmz_imu_roll_pi": False}],
                   remappings=[("/hardware/actuator_states", "/xmz/actuator_states")])
    hardware = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory("hardware_kaixuan_arms")) / "launch/dual_robot.launch.py")),
        launch_arguments={"arms": arms, "head": head}.items())
    imu = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory("bxi_imu")) / "launch/imu.launch.py")),
        launch_arguments={"imu_reader_priority": "20"}.items())
    return LaunchDescription([
        DeclareLaunchArgument("arms", default_value="true"),
        DeclareLaunchArgument("head", default_value="true"),
        RegisterEventHandler(OnProcessExit(on_exit=required_process_exit)),
        hardware, imu, merger, example,
    ])
