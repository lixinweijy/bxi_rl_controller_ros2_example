"""ARM-08 deployment: the existing new body/arm drivers and xmz walking Mod."""
import atexit
import fcntl
import os
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
    # Successful preflight processes finish before the long-running hardware nodes.
    if event.returncode == 0 and event.process_name.rsplit("-", 1)[0] in (
        "ethercat_bind", "wait_for_arm_topology"
    ):
        return []
    return [EmitEvent(event=Shutdown(reason=f"xmz process exited: {event.process_name}"))]


LOCK_FILE = "/tmp/bxi_example_hw.lock"
_lock_fd = None


def _release_lock():
    global _lock_fd
    if _lock_fd is None:
        return
    fd = _lock_fd
    _lock_fd = None
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        os.close(fd)
    except OSError:
        pass


def _acquire_lock():
    global _lock_fd
    if _lock_fd is not None:
        return

    # Create the lock file world-writable so a different user (or root)
    # can reuse the same system-wide lock. Force umask to 0 so the 0o666
    # mode is honored on creation regardless of the caller's umask.
    old_umask = os.umask(0)
    try:
        try:
            fd = os.open(LOCK_FILE, os.O_RDWR | os.O_CREAT, 0o666)
        except PermissionError as e:
            print(
                f"\n[ERROR] Cannot open lock file {LOCK_FILE}: {e}\n"
                f"        A stale lock file with restrictive permissions exists.\n"
                f"        Remove it and retry:  sudo rm {LOCK_FILE}\n",
                file=sys.stderr,
            )
            sys.exit(1)
    finally:
        os.umask(old_umask)

    # Repair perms on a file left behind by an older version of this
    # script (no-op unless we are the owner or root).
    try:
        os.chmod(LOCK_FILE, 0o666)
    except OSError:
        pass

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        try:
            os.lseek(fd, 0, os.SEEK_SET)
            holder = (
                os.read(fd, 64).decode("utf-8", errors="replace").strip() or "unknown"
            )
        except OSError:
            holder = "unknown"
        os.close(fd)
        print(
            f"\n[ERROR] bxi_example_hw is already running (pid={holder})! "
            f"Please stop the existing instance before starting a new one.\n",
            file=sys.stderr,
        )
        sys.exit(1)

    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode("utf-8"))
    _lock_fd = fd
    atexit.register(_release_lock)


def generate_launch_description():
    arms = LaunchConfiguration("arms")
    head = LaunchConfiguration("head")
    _acquire_lock()
    for entry in Path("/proc").iterdir():
        if entry.name.isdecimal():
            try:
                name = (entry / "comm").read_text().strip()
            except (OSError, ProcessLookupError):
                continue
            if name in {"hardware_elf3", "dual_kaixuan_ar", "dual_kaixuan_arm", "hw_lib_test_elf"} or name.startswith("hw_lib_test"):
                raise RuntimeError(f"Existing hardware owner pid={entry.name} ({name}); stop it before Start")
    # Contract: this launcher lives in its own portable Mod's launch/ directory.
    root = Path(get_package_share_directory("bxi_example_py_elf3")) / "mods/com.xmz.walk"
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
