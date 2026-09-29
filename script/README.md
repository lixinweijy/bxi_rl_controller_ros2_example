# NVIDIA 遥控器自启与串口电源部署

适用于 `robot_test` 分支、NVIDIA Jetson `aarch64`、Ubuntu 24.04、ROS 2 Jazzy。
这是已有 ROS/硬件运行环境的部署流程，不是 Jetson 刷机或 ROS 安装教程。
系统文件以本目录为维护入口，不再依赖某台开发机上的 `deployment/` 备份。

## 文件与启动链路

| 仓库文件 | 板端安装位置 | 用途 |
| --- | --- | --- |
| `ros_elf_launch.nvidia.service` | `/etc/systemd/system/ros_elf_launch.service` | root 启动遥控器，Domain 0 / Fast DDS / 本机发现 |
| `bxi-motor-power` | `/usr/local/sbin/bxi-motor-power` | 串口 `o` 开电、`c` 断电 |
| `bxi-motor-ros` | `/usr/local/bin/bxi-motor-ros` | 统一管理开电、ROS 进程组、退出断电；`--stop` 停止 |
| `bxi-rl-ros` | `/usr/local/bin/bxi-rl-ros` | 通过上一脚本启动当前统一 example |
| `bxi-motor-power.sudoers` | `/etc/sudoers.d/bxi-motor-power` | 可选：允许 nvidia 用户免密调用精确的 on/off 命令 |
| `bxi-dev.rules` | `/etc/udev/rules.d/bxi-dev.rules` | IMU、串口和手柄稳定设备名 |
| `bxi-battle-dragon-link` | `/usr/local/bin/bxi-battle-dragon-link` | udev 调用的 Battle Dragon 链接维护脚本 |
| `bxi-hid.conf` | `/etc/modules-load.d/bxi-hid.conf` | 开机加载 `uhid`，解决蓝牙已连但无输入设备 |
| `80-bxi-can.network` | `/etc/systemd/network/80-bxi-can.network` | can0～3，CAN-FD 仲裁 1 Mbit/s、数据 5 Mbit/s |

`ros_elf_launch.service`（无 `.nvidia` 后缀的仓库文件）是原有 Humble 模板，
包含 `ROS_DOMAIN_ID=XX` 和 `/home/bxi` 路径；**NVIDIA 请安装上表的专用版本**。

按下文配置后，链路为：

```text
开机 → ros_elf_launch.service → remote_controller（仅接收输入）
Start → xbox_default.yaml → bxi-rl-ros → bxi-motor-ros
      → bxi-motor-power on → 等待 0.5 秒 → example_launch_unified_hw.launch.py
Stop  → bxi-motor-ros --stop → 停止 ROS 进程组 → bxi-motor-power off
```

`bxi-motor-ros` 同时处理正常退出、launch 失败和 HUP/INT/TERM：先终止整个
launch 进程组，最多等待 2 秒后清理残留，再断电。两个入口共享互斥锁。
SIGKILL、系统掉电或串口硬件故障无法保证软件清理，仍需独立急停。
不要同时通过 App、其他终端或旧包装器启动另一套硬件控制程序。

## 1. 检查环境与备份

> **警告：后续安装会覆盖系统配置，停服务可能中止当前控制程序。**
> 先让机器人安全停止、扶稳或悬吊，确认急停可用并关闭电机电源。
> 部署期间不要按遥控器 Start，也不要通过 App 启动。

在 NVIDIA 的 **Bash** 终端中执行。所需依赖：`bash`、`util-linux`
（flock/setsid）、`procps`（pkill）、udev、systemd、Bluetooth、colcon，
以及与本机架构和 Jazzy ABI 匹配的 `/opt/bxi/bxi_ros2_pkg`、控制器 Python 依赖和模型。

```bash
bash
uname -m                         # aarch64
. /etc/os-release
printf '%s %s\n' "$ID" "$VERSION_ID"  # ubuntu 24.04
test -r /opt/ros/jazzy/setup.bash
test -r /opt/bxi/bxi_ros2_pkg/local_setup.bash
command -v colcon flock setsid pkill stty

mkdir -p /home/nvidia/bxi_ws
cd /home/nvidia/bxi_ws
# 首次克隆；已有目录请在该仓库内检查 git status 后再更新，勿覆盖未提交修改。
git clone --branch robot_test --single-branch \
  https://github.com/lixinweijy/bxi_rl_controller_ros2_example.git
cd bxi_rl_controller_ros2_example
git status --short --branch

backup="/var/backups/bxi-nvidia-$(date +%Y%m%d-%H%M%S)"
sudo mkdir -p "$backup"
for path in \
  /etc/systemd/system/ros_elf_launch.service \
  /etc/udev/rules.d/bxi-dev.rules \
  /etc/modules-load.d/bxi-hid.conf \
  /etc/systemd/network/80-bxi-can.network \
  /etc/sudoers.d/bxi-motor-power \
  /usr/local/sbin/bxi-motor-power \
  /usr/local/bin/bxi-motor-ros \
  /usr/local/bin/bxi-rl-ros \
  /usr/local/bin/bxi-battle-dragon-link; do
  if sudo test -e "$path"; then
    sudo cp -a --parents "$path" "$backup/"
  fi
done
cp -p src/remote_controller/config/xbox_default.yaml \
  src/remote_controller/config/xbox_default.yaml.before-nvidia-$(date +%Y%m%d-%H%M%S)
sudo systemctl stop ros_elf_launch.service
```

已有工作区固定使用 `/home/nvidia/bxi_ws/bxi_rl_controller_ros2_example`，
保留 `/opt/bxi/bxi_rl_controller_ros2_example` 兼容链接；如果目标已有真实目录或
指向其他工作区，请先核对和备份，不要强制覆盖：

```bash
sudo mkdir -p /opt/bxi
if [[ ! -e /opt/bxi/bxi_rl_controller_ros2_example && ! -L /opt/bxi/bxi_rl_controller_ros2_example ]]; then
  sudo ln -s /home/nvidia/bxi_ws/bxi_rl_controller_ros2_example /opt/bxi/bxi_rl_controller_ros2_example
fi
test "$(readlink -f /opt/bxi/bxi_rl_controller_ros2_example)" = "$PWD"
```

最后的路径检查失败就停止，先修正工作区路径。后续命令均从该仓库根目录执行。

## 2. 配置 NVIDIA 遥控启停并编译

仓库默认 `xbox_default.yaml` 直接调用统一 launch。要启用外部串口电源包装器，
将 `src/remote_controller/config/xbox_default.yaml` 中的 **`system` 段**替换如下；
保留所有输入映射、`system_mutexes` 和 `system_reset_motion_after`。
仅安装脚本不会自动改变已有的按键配置。

```yaml
system:
  start:
    - "mkdir -p /var/log/bxi_log"
    - "/usr/local/bin/bxi-rl-ros > /var/log/bxi_log/$(date +%Y-%m-%d_%H-%M-%S)_elf.log 2>&1 &"
    - "ros2 launch bxi_example_bms bms.launch.py > /var/log/bxi_log/bms_$(date +%Y-%m-%d_%H-%M-%S)_bms.log 2>&1 &"
  stop:
    - "/usr/local/bin/bxi-motor-ros --stop"
    - "killall -SIGINT hardware_elf3 bxi_example_hw bxi_example_py_elf3 bxi_example_py_elf3_demo bxi_example_py_elf3_vibration bxi_example_py_elf3_suspended_tests bxi_example_py_elf3_unified bxi_example_py_elf3_mjlab bxi_bms bxi_example_bms 2>/dev/null"
    - "sleep 1"
```

不要改回已删除的 `example_demo_hw.launch.py`。当前 `bxi-rl-ros` 启动
`example_launch_unified_hw.launch.py`，沿用 200 Hz、momentary 按键模式，
并设置 `start_remote_controller:=false` 避免重复遥控节点。

```bash
source /opt/ros/jazzy/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 pkg prefix hardware_elf3
ros2 pkg prefix remote_controller
ros2 pkg prefix bxi_example_py_elf3
ros2 pkg prefix bxi_example_bms
```

任何构建/包检查失败都先修复，不要继续启用自启动。需要重新构建遥控包后，
在安全停止状态重启遥控服务，新 YAML 才会加载。

## 3. 安装脚本与开机配置

```bash
sudo install -d /usr/local/bin /usr/local/sbin /etc/modules-load.d /etc/systemd/network
sudo install -o root -g root -m 0755 script/bxi-motor-power /usr/local/sbin/bxi-motor-power
sudo install -o root -g root -m 0755 \
  script/bxi-motor-ros script/bxi-rl-ros script/bxi-battle-dragon-link /usr/local/bin/
sudo install -o root -g root -m 0644 script/bxi-hid.conf /etc/modules-load.d/bxi-hid.conf
getent group imu >/dev/null || sudo groupadd --system imu
sudo install -o root -g root -m 0644 script/bxi-dev.rules /etc/udev/rules.d/bxi-dev.rules
sudo install -o root -g root -m 0644 script/ros_elf_launch.nvidia.service /etc/systemd/system/ros_elf_launch.service

sudo modprobe uhid
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=input
sudo udevadm trigger --action=change --subsystem-match=tty
sudo udevadm settle
sudo /usr/local/bin/bxi-battle-dragon-link
sudo systemctl daemon-reload
sudo systemctl enable ros_elf_launch.service
```

上面只启用下次开机自启，没有启动 example 或发送串口开电命令。
蓝牙手柄需要预先配对；`Connected: yes` 还必须对应有效的输入设备。

若板上 CAN 确实采用表中的速率、且由 systemd-networkd 管理，再安装 CAN 配置；
不要与 NetworkManager/其他脚本重复管理 can0～3，也不要在电机运行中重配接口：

```bash
sudo install -o root -g root -m 0644 script/80-bxi-can.network /etc/systemd/network/80-bxi-can.network
sudo systemctl enable --now systemd-networkd.service
sudo networkctl reload
sudo networkctl reconfigure can0 can1 can2 can3
ip -details link show can0
```

root 服务不需要 sudoers。仅需 nvidia 用户免密手动开关电时，才安装这一项：

```bash
sudo visudo -cf script/bxi-motor-power.sudoers
sudo install -o root -g root -m 0440 script/bxi-motor-power.sudoers /etc/sudoers.d/bxi-motor-power
sudo visudo -c
```

## 4. 验证与常用命令

本机离线检查（不会访问真实串口、启动真实 ROS 或修改系统）：

```bash
for file in script/bxi-motor-power script/bxi-motor-ros script/bxi-rl-ros; do bash -n "$file"; done
python3 script/test_nvidia_scripts.py
# 如已安装 shellcheck：
shellcheck script/bxi-motor-power script/bxi-motor-ros script/bxi-rl-ros
```

在目标板上先检查静态配置与设备，再在机器人安全停机、遥控按键松开时启动接收服务：

```bash
sudo systemd-analyze verify /etc/systemd/system/ros_elf_launch.service
ls -l /dev/uhid /dev/input/jsBattleDragon /dev/ttyMotorPower
readlink -f /dev/ttyMotorPower
sudo systemctl start ros_elf_launch.service
systemctl is-enabled ros_elf_launch.service
systemctl show ros_elf_launch.service -p ActiveState -p SubState -p NRestarts -p Environment
sudo journalctl -u ros_elf_launch.service -n 60 --no-pager
```

接收服务应为 `enabled`、`active/running`，重启次数不持续增长，等待输入时没有
`hardware_elf3` 或 example 进程。实时日志用 `sudo journalctl -fu ros_elf_launch.service`。
这只能验证接收链路；真正按 Start/Stop 的开关电和控制效果需要单独现场验收。

> **警告：以下 on 和启动命令会实际给电机上电，只有机器人已固定、急停可用时才能执行。**
> 手动断电也可能导致机器人失去支撑；应先安全停止控制程序。

```bash
sudo /usr/local/sbin/bxi-motor-power on     # 只开电
sudo /usr/local/sbin/bxi-motor-power off    # 只断电
sudo /usr/local/bin/bxi-rl-ros              # 开电并启动统一 example
sudo /usr/local/bin/bxi-motor-ros --stop    # 另一终端：停止进程组并断电
# 只调试 hardware_elf3、不启动控制器时（不能与 example 同时运行）：
sudo /usr/local/bin/bxi-motor-ros
```

串口默认 `/dev/ttyMotorPower`，不存在时才回退 `/dev/ttyACM0`；规则匹配
USB VID:PID `1a86:55d3`，多块相同设备时须用唯一的设备路径/udev 匹配。
协议为 `921600 8N1`、无流控、单字节 `o/c`，不附加换行，也没有 ACK 校验。
脚本成功表示字节已写入，不能替代供电实测。

| 环境变量 | 默认值 | 含义 |
| --- | --- | --- |
| `BXI_MOTOR_POWER_DEVICE` | `/dev/ttyMotorPower`，缺失回退 ttyACM0 | 显式指定时必须存在，错误路径不回退 |
| `BXI_MOTOR_POWER_OPEN_DELAY` | `0.5` 秒 | 打开串口并配置后、写命令前等待 |
| `BXI_MOTOR_POWER_ON_DELAY` | `0.5` 秒 | 开电完成后、启动 ROS 前等待 |

需要调整时在 `sudo systemctl edit ros_elf_launch.service` 的 `[Service]` 中添加
相应 `Environment=...`，安全停机后 reload/restart 才生效。保持延时较小：当前
launch 的额外断电回调超时为 5 秒，串口锁等待最多 3 秒，Stop 等待最多 8 秒。
终端手动调用可用 `sudo env BXI_MOTOR_POWER_DEVICE=/dev/serial/by-id/... ...`。
调试 ROS 图的终端/App 必须与服务使用相同 Domain、RMW 和本机发现设置。

启动失败后遥控器的 `robot_process` 互斥仍可能被占用；先按 Stop 清锁再 Start。
若蓝牙已连接却没有输入，先检查 `/dev/uhid`、模块加载配置、
`sudo journalctl -b -u bluetooth.service` 中的 HoG 错误及手柄稳定链接，勿默认重新配对。

## 5. 回滚与验证边界

安全停机后 `sudo systemctl stop ros_elf_launch.service`，从第 1 步备份恢复对应的
`/etc` 和 `/usr/local` 文件，并恢复原 `xbox_default.yaml`、重新构建遥控包。
执行 `sudo systemctl daemon-reload` 和 `sudo udevadm control --reload-rules` 后核对配置。
首次部署、备份中没有旧 unit 时，先 `sudo systemctl disable ros_elf_launch.service`；
不要直接覆盖其他设备的配置或批量删除备份。CAN 变更也需在停机状态恢复并重新应用。

2026-09-29：脚本来自 NVIDIA 历史部署备份，并适配 `robot_test` 当前统一 launch；
本次验证范围为 Shell 语法、伪串口及模拟 ROS 子进程。目标 NVIDIA 当前不可达，
上述安装、开关电、真实遥控按键及整机重启流程尚未在目标板重新执行。
