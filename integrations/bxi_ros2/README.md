# BXI ELF3 ROS 2 integration

This directory ports [`bxi_rl_controller_ros2_example:robot_test`](https://github.com/lixinweijy/bxi_rl_controller_ros2_example/tree/robot_test) into the [official Holosoma repository](https://github.com/amazon-far/holosoma). Source baseline: `robot_test@f384d29`; Holosoma baseline: `main@bccd4d7`. The BXI packages remain a separate ROS 2 workspace inside Holosoma because `communication`, `hardware_elf3`, `mujoco` and the remote controller are ROS packages.

The `bxi_example_py_elf3_unified` node remains the **sole actuator command publisher**. Its LB/RB walking modes call Holosoma's `LocomotionPolicy` in process for observation construction, inference, action scaling and PD gains. The local `elf4_preset.py` supplies the 29 joint order and observation contract because upstream Holosoma `main` has G1/T1 presets only. It uses the ELF4 values from the local `holosoma_elf4-main.zip` snapshot; model metadata must match. A/B/Y modes, PD return, reset, feedback watchdogs and motor power shutdown retain the `robot_test` implementations. The historical `mjlab`, `run` and demo entry points are still available for compatibility; do not start them alongside the unified controller.

| `robot_test` function | Ported entry |
| --- | --- |
| LB auto walk, RB remote walk, PD return | `bxi_example_py_elf3_unified` + Holosoma `LocomotionPolicy` |
| A/B ROM and Y vibration | Same unified ROS controller |
| Joint/IMU feedback, reset and single actuator publisher | `bxi_example_py_elf3` ROS package |
| Remote Start/Stop and button mapping | `remote_controller` ROS package |
| BMS | `bxi_example_bms` ROS package |
| MuJoCo simulation | `example_launch_unified.launch.py` with BXI ROS `mujoco` |
| Hardware | `example_launch_unified_hw.launch.py` |

## Build

Use Ubuntu 22.04 / ROS 2 Humble and source the BXI ROS packages that provide `communication`, `hardware_elf3` and `mujoco`. Install Holosoma inference in the same Python environment used to build the ROS workspace; this ensures the installed controller script uses that interpreter.

```bash
cd /home/lxw/holosoma/holosoma
python3 -m venv --system-site-packages /home/lxw/holosoma/.venv
source /home/lxw/holosoma/.venv/bin/activate
python -m pip install -e src/holosoma_inference
source /opt/ros/humble/setup.bash
source /path/to/bxi_ros2_pkg/install/setup.bash
cd integrations/bxi_ros2
python -m colcon build --merge-install
source install/setup.bash
```

The installed `bxi_example_py_elf3_unified` script should begin with the path to the chosen virtual environment's Python interpreter. The remote controller service must source this workspace and have `HOLOSOMA_MODEL_PATH` set to an **absolute path** before Start is pressed.

## Model and launch

An ELF4 ONNX must have one 100 dimensional `actor_obs` input, 29 actions, canonical `joint_order`, matching default pose, observation terms, action scale, policy period, velocity ranges and positive 29 joint `kp/kd`. The old ELF3 96/960 dimensional ONNX assets included for legacy entries cannot run in the unified Holosoma entry. No unverified ONNX is selected by default.

```bash
ros2 launch bxi_example_py_elf3 example_launch_unified.launch.py holosoma_model_path:=/absolute/path/to/validated-elf4.onnx
```

After simulation and suspended verification, the hardware entry is:

```bash
sudo bash -lc 'source /opt/ros/humble/setup.bash && source /path/to/bxi_ros2_pkg/install/setup.bash && source /home/lxw/holosoma/holosoma/integrations/bxi_ros2/install/setup.bash && ros2 launch bxi_example_py_elf3 example_launch_unified_hw.launch.py holosoma_model_path:=/absolute/path/to/validated-elf4.onnx start_remote_controller:=false'
```

The hardware launch rejects a missing model path before starting the hardware node. LB commands alternate ±0.5 m/s. RB uses the existing remote mapping, clipped to the model's `command_ranges`; the local ELF4 example model limits forward speed to 1 m/s. The hardware launch requires root and powers motors off on shutdown.

## Verification boundary (2026-09-28)

The official Holosoma checkout built all four imported ROS packages using a virtual environment with editable `holosoma-inference`. The 28 Python tests in `bxi_example_py_elf3/test` passed in an isolated ROS domain with a local ELF4 ONNX. The remote controller CTest run reported 26 tests, zero failures and 21 skips; its B-button assertion was updated to match the current B→`btn_8` mapping. They cover the mode transitions, one publisher, feedback and IMU faults, model velocity limits, 29/31 joint mapping, 100 dimensional observations, and rejection of an ONNX with a different joint order. A direct policy step produced 29 joint targets using the official inference source. A 15-second isolated MuJoCo startup reached the second reset acknowledgement and completed initialization; the unified node exited cleanly on SIGINT. MuJoCo gait quality and powered hardware motion have **not** been verified. The sample ELF4 model can exceed ELF3 software joint limits in some states; the controller rejects those targets.

Test command, after sourcing the environments above:

```bash
HOLOSOMA_TEST_MODEL=/absolute/path/to/elf4.onnx python -m pytest --confcutdir=src/bxi_example_py_elf3/test -q src/bxi_example_py_elf3/test
```

Historical hardware notes are retained in [README_hw.md](README_hw.md).
