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

## 多线程控制器有序退出

2026-09-09 修复：实机 ROM 与振动控制器共用
`control/ros_runtime.py`，收到 SIGINT/SIGTERM 时停止调度，等待回调结束，
再销毁节点及 ROS 上下文。安全故障仍锁定指令发布，但不再从工作线程直接关闭上下文。
当前 NVIDIA 自带的 `rclpy` 未实现多线程线程池的关闭覆盖，因此在释放执行器资源前
显式等待其线程池退出；升级 ROS 后需要复核此兼容处理。

独立退出回归测试不创建机器人控制器、不发布电机话题；只向测试子进程发送信号：

```bash
source /opt/ros/jazzy/setup.bash
python3 -B src/bxi_example_py_elf3/test/test_ros_runtime.py
```

覆盖 SIGINT、SIGTERM、安全退出、普通回调异常传播，以及停止后的控制/复位请求拦截。
此修复处理退出竞态，不代表已查明最初停止信号的来源，也不处理 IMU 或 CAN 超时。
