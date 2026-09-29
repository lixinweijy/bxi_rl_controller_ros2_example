#!/usr/bin/env python3
"""Offline check: temp paths, a PTY and fake ROS; no sudo or real hardware."""

import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import tempfile
import time


def main():
    source = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix="bxi-nvidia-check-") as directory:
        root = Path(directory)
        env = dict(os.environ, PATH=f"{root}:{os.environ['PATH']}", CHECK_DIR=directory,
                   BXI_MOTOR_POWER_OPEN_DELAY="0", BXI_MOTOR_POWER_ON_DELAY="0")
        replacements = {
            "/usr/local/bin/bxi-motor-ros": str(root / "bxi-motor-ros"),
            "/usr/local/sbin/bxi-motor-power": str(root / "fake-power"),
            "/run/lock/": f"{root}/",
            "/opt/ros/jazzy/setup.bash": str(root / "setup.bash"),
            "/opt/bxi/bxi_ros2_pkg/local_setup.bash": str(root / "setup.bash"),
            "/opt/bxi/bxi_rl_controller_ros2_example/install/local_setup.bash": str(root / "setup.bash"),
        }
        for name in ("bxi-motor-power", "bxi-motor-ros", "bxi-rl-ros"):
            subprocess.run(["bash", "-n", str(source / name)], check=True)
            text = (source / name).read_text()
            for old, new in replacements.items():
                text = text.replace(old, new)
            path = root / name
            path.write_text(text)
            path.chmod(0o755)

        def run(name, *args, code=0, **variables):
            result = subprocess.run([str(root / name), *args], env=dict(env, **variables),
                                    capture_output=True, text=True, timeout=12)
            assert result.returncode == code, (name, result.returncode, result.stderr)
            return result

        # Real stty/write on a pseudo-terminal; never resolve a hardware device.
        master, slave = pty.openpty()
        try:
            for action, expected in (("on", b"o"), ("off", b"c")):
                run("bxi-motor-power", action, BXI_MOTOR_POWER_DEVICE=os.ttyname(slave))
                assert select.select([master], [], [], 1)[0], "missing serial byte"
                assert os.read(master, 16) == expected
            run("bxi-motor-power", "invalid", code=2)
            run("bxi-motor-power", "on", code=1, BXI_MOTOR_POWER_DEVICE=str(root / "missing"))
            assert not select.select([master], [], [], 0)[0], "unexpected serial data"
        finally:
            os.close(master)
            os.close(slave)

        (root / "setup.bash").write_text(":\n")
        (root / "fake-power").write_text(
            '#!/bin/bash\necho "$1" >> "$CHECK_DIR/events"\n'
            '[[ "$1" != on || "${CHECK_MODE:-}" != power-fail ]] || exit 7\n'
        )
        (root / "ros2").write_text(
            '#!/bin/bash\necho "ros2 $*" >> "$CHECK_DIR/events"\n'
            'case "${CHECK_MODE:-}" in\n'
            '  launch-fail) exit 7 ;;\n'
            '  hold) echo $$ > "$CHECK_DIR/group"; trap "" TERM; '
            'sleep 60 & echo $! > "$CHECK_DIR/child"; wait ;;\n'
            'esac\n'
        )
        for name in ("fake-power", "ros2"):
            (root / name).chmod(0o755)

        events = root / "events"
        for mode, expected_code in (("", 0), ("launch-fail", 7), ("power-fail", 7)):
            events.write_text("")
            run("bxi-rl-ros", code=expected_code, CHECK_MODE=mode)
            lines = events.read_text().splitlines()
            assert lines[0] == "on" and lines[-1] == "off", lines
            if mode == "power-fail":
                assert lines == ["on", "off"], lines
            else:
                assert lines[1] == (
                    "ros2 launch bxi_example_py_elf3 example_launch_unified_hw.launch.py "
                    "start_remote_controller:=false control_rate_hz:=200.0 motion_button_mode:=momentary"
                ), lines

        events.write_text("")
        run("bxi-motor-ros")
        assert events.read_text().splitlines() == [
            "on", "ros2 launch hardware_elf3 hardware_elf3_launch.py", "off"]
        events.write_text("")
        (root / "setup.bash").write_text("false\n")
        run("bxi-rl-ros", code=1)
        assert events.read_text() == "", "powered on despite failed ROS setup"
        (root / "setup.bash").write_text(":\n")

        proc = subprocess.Popen([str(root / "bxi-rl-ros")], env=dict(env, CHECK_MODE="hold"),
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic() + 5
            while not (root / "child").exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert (root / "child").exists(), "mock ROS did not start"
            before = events.read_text()
            run("bxi-rl-ros", code=1)
            assert events.read_text() == before, "second launcher touched power"
            run("bxi-motor-ros", "--stop")
            assert proc.wait(timeout=5) == 143
            child_pid = int((root / "child").read_text())
            stat = Path(f"/proc/{child_pid}/stat")
            assert not stat.exists() or stat.read_text().split()[2] == "Z", "orphan survived"
            assert events.read_text().splitlines()[-1] == "off"
        finally:
            if (root / "group").exists():
                try:
                    os.killpg(int((root / "group").read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if proc.poll() is None:
                proc.kill()
            proc.communicate(timeout=5)
    print("PASS: serial o/c, setup/power/launch failures, shared lock, stop/group cleanup")


if __name__ == "__main__":
    main()
