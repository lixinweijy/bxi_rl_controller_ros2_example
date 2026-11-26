# ARM-05 ELF4 XMZ 单模型运行

RB+X 固定运行 `assets/policy.onnx`，参数来自 `assets/config.json`。有运动指令、摇杆回中或遥控失联均使用同一个模型；失联超过0.5秒仅清零速度指令。取消双模型自动选择、切换历史重建及0.10秒混合，不再加载或要求 `policy_1.onnx` / `config_1.json`；这两份文件保留用于历史备份。

29轴、10×96历史、50Hz CPU ONNX及逐关节scale保持。进入XMZ时初始化历史，持续运行时运动/停止不重置历史；`advance=False`不推进历史。四路observation/action_raw/action_clipped/target trace保留，取消双模型专用 `/xmz/policy/active`。

模型SHA256：`311add88260ad5046ffa5721a4391ea0248e05301755cc7127a12f74304be211`。参数SHA256：`7365348dbb84ddd8925fddf087a7acab6d328fc90630047633cef86fff322cfe`。ONNX接口float32[1,960]→[1,29]，无训练metadata，按用户指定沿用运动组配置。离线加载/推理通过不代表实机运动验收。

## 腕 X 角度偏置（2026-11-25）

按用户要求，在 elf4_policy.py 的 WRIST_POSITION_OFFSETS_DEG 设置左腕X −10°、右腕X +10°。observe() 在减默认角和观测缩放之前加偏置；infer() 在动作裁剪、缩放并加默认角之后再加同号偏置。两处是独立加法，不是互逆标定。原始关节反馈、速度、previous action、默认姿态和PD参数不变，进入策略前的PD姿态保持原值。内部统一换算为rad；冻结手臂时对应偏置屏蔽。

修改后需构建到正式install并正常重新Start才加载。未因此修改ONNX/JSON或底层标定。check_mod新增输入/输出单位、方向、原反馈不变与历史动作未加偏置的检查；check_sim2sim需要同步修改后的参考runner。

## 使用与换模型

正常Start后RB+B进入PD，RB+X进入单模型；RB+B回PD、RB+A零力矩、RB+Y零位。遥控profile前后满杆±0.5m/s、横移±0.5m/s、转向±1.57rad/s，最终还受当前config.json的command_ranges限制。100ms反馈校验、IMU/倾角保护和宿主失联锁定保持。

替换 `src/bxi_example_py_elf3/mods/com.xmz.walk/assets/policy.onnx` 和同目录 `config.json` 后，在机器人正常Stop且硬件/控制进程退出时执行项目 `./build.sh`，更新正式install。下次正常Start加载。不需要修改plugin.py或state.py，也不要覆盖运行中的模型文件。

本次替换主模型和主配置；备用policy_1/config_1、mod.yaml、共享policies/joints.py、硬件配置/固件/标定均保持。当前模型为用户指定的 `arm_003_60k.onnx`，安装名 `policy.onnx`；主参数按已提交源码的运动组 `config_1.json` 原字节复制为 `config.json`。29轴默认角、Kp/Kd及逐轴scale已与现有共享PD核对一致，未修改共享参数。前后指令范围±0.5m/s（不沿用旧install备用config_1的±0.3）。

config里的armature/root_pos/physics_dt是仿真参数；JSON effort_limit/velocity_limit不自动覆盖硬件/固件限制。

### 当前硬件增益范围（2026-11-21核对）

ARM-05 正式 hardware_elf3 SHA256 `28ddbfb64b92b29493e386f45a48c8d4eefc7b24423c8ec229eb06963e50f0b4` 与11月19日增益包manifest一致。BXI95的Kp/Kd编码上限800/50，BXI70/70L为500/10；其余型号保持各自量程。当前沿用运动配置的23个直驱策略关节Kp/Kd均处于上述编码范围内。

腰Y/X及四踝在主机并联环计算PD后下发扭矩，沿用既有并联限幅。编码范围核对不代表已验收固件和实机闭环；本次不修改底层通信量程或固件。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
source install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 install/share/bxi_example_py_elf3/mods/com.xmz.walk/tests/check_mod.py install/share/bxi_example_py_elf3
```

check_mod覆盖单模型运动/停止/失联且不重置历史、history/scale、预览/重进、缺少备用资产仍能启动、遥测异常、反馈/IMU校验与手柄安全路由。check_sim2sim现比较policy.onnx/config.json与参考runner，需配套XML和依赖；未运行该脚本时不代表运动稳定性验收。
