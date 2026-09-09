# NVIDIA 实机启动环境

## 网关启动时找不到 ROM 控制器模块

2026-09-09 在 NVIDIA 设备验证：网关启动脚本
`/opt/bxi/bxi_rc_ros2/bxi_rc_ros2_start.sh` 原先加载
`/opt/bxi/bxi_rl_controller_ros2_example/setup.bash`，但当前设备采用源码工作区布局，
正确入口是 `/opt/bxi/bxi_rl_controller_ros2_example/install/setup.bash`。
错误路径会被跳过，导致网关环境找不到
`bxi_example_py_elf3.bxi_example_suspended_tests`。

部署脚本不属于本仓库；对应的一行修复保存在
[script/fix_gateway_workspace_setup.patch](script/fix_gateway_workspace_setup.patch)。
此补丁适用于上述 NVIDIA 工作区布局，不应直接用于安装结构不同的设备。

### 应用与检查

警告：以下补丁会修改网关启动脚本。当前 NVIDIA 已应用，不要重复应用；
应用补丁不会更新已运行进程的环境，也不会自动重启服务。

从本仓库根目录执行，先备份并检查补丁：

```bash
sudo cp -pn /opt/bxi/bxi_rc_ros2/bxi_rc_ros2_start.sh /opt/bxi/bxi_rc_ros2/bxi_rc_ros2_start.sh.before-workspace-path-fix
sudo patch --batch --forward --dry-run -d /opt/bxi/bxi_rc_ros2 -p1 < script/fix_gateway_workspace_setup.patch
```

仅在检查成功且确认尚未应用时执行：

```bash
sudo patch --batch --forward -d /opt/bxi/bxi_rc_ros2 -p1 < script/fix_gateway_workspace_setup.patch
bash -n /opt/bxi/bxi_rc_ros2/bxi_rc_ros2_start.sh
```

只检查模块路径，不启动 ROS 节点或电机：

```bash
source /opt/ros/jazzy/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source /opt/bxi/bxi_rl_controller_ros2_example/install/setup.bash
source /opt/bxi/bxi_rc_ros2/setup.bash
python3 -B -c "import importlib.util; s = importlib.util.find_spec('bxi_example_py_elf3.bxi_example_suspended_tests'); assert s is not None; print(s.origin)"
```

上述语法和模块路径检查已通过；网关需要由操作人员在安全时机重新启动才能使用新环境。
OTA 覆盖部署脚本后，应检查此修复是否仍保留。

## 单线程控制与有序退出

2026-09-09 修复：实机 ROM 与振动控制器共用
`control/ros_runtime.py`。根据操作人员反馈，多线程启动出现 actuator 延迟，
切回单线程后恢复正常，因此统一恢复 `SingleThreadedExecutor`；删除线程池私有清理逻辑。
保留有序退出：SIGINT/SIGTERM 只请求停止，当前回调结束后再销毁节点及 ROS 上下文。
安全故障仍锁定指令发布并请求退出；ROM、按键、200 Hz 控制频率和 50 ms 发布间隔保护不变。

独立退出回归测试不创建机器人控制器、不发布电机话题；只向测试子进程发送信号：

```bash
source /opt/ros/jazzy/setup.bash
python3 -B src/bxi_example_py_elf3/test/test_ros_runtime.py
```

覆盖主线程执行、SIGINT、SIGTERM、安全退出、普通回调异常传播，以及停止后的控制/复位请求拦截。
2026-09-09 17:21:02 启动日志中，发布间隔 74.485 ms 超过 50 ms 保护阈值，
控制器主动退出后 launch 停止硬件；具体的多线程延迟来源尚未定位。
离线测试不等同于实机或长时间稳定性验证，也不证明 IMU 或 CAN 超时已解决。
