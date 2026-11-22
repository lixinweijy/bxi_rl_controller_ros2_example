# ARM-05 ELF4 XMZ 行走

2026-11-21：在用户现有两组资产上实现按遥控运动指令自动切换，四份资产保持原字节。

| 有效运动指令 | 模型/配套参数 | SHA256（模型 / JSON） |
| --- | --- | --- |
| 前后、横移、转向任一非零 | policy_1.onnx / config_1.json | b32498edbc332eff2825f3290d29fbe6c3195208ebb4894bc3b9cf597ecb602f / dd26df864ce0c7bff507d7cd7fd9faee903c04b5324299b6db75be65bd7362f8 |
| 全零或超过0.5秒无遥控消息 | policy.onnx / config.json | 6abc9f3547aaab1acd7661647cca0df9cb38f7a7f4565aceb3583b155c57fe31 / 300a98f5757b50918d39106ee2342ea47df3f7391d0a2450446179abc8efc382 |

按速度指令内容判断（绝对值≤1e−6视为零），不是按话题有无消息判断；遥控持续发送零值也使用静止模型。两个CPU session在启动时预载并预热，切换周期只推理选中模型，不从磁盘加载。RB+X初始使用静止模型。
29 个关节按 config 的名字重排，10×96 历史从旧到新，首帧铺满；CPU ONNX 50 Hz，逐关节 action_scale、默认角和 PD。双臂参与输入输出，不使用旧 0.23 标量或旧 ELF3 参数。

## 使用

正常 Start 后，RB+B 进入既有共享参数 PD 姿态，RB+X 进入 XMZ双模型自动选择；RB+B 返回 PD，RB+A 全身零力矩，RB+Y 零位。零力矩和 IMU 失联保护由精简的 `com.bxi.basic_actions` 提供。
遥控profile前后仍为满杆±0.5m/s、半杆±0.25，但当前用户config_1.json限制前后±0.3m/s，因此模型有效前后速度满杆为±0.3m/s；横移±0.5m/s、转向±1.57rad/s。状态报告的速度同步为限幅后的有效值。遥控超过0.5秒无消息则指令清零并切回静止模型。反馈超过100ms、缺少任一策略关节、非法IMU或倾角超过1rad拒绝策略输入；保留宿主IMU失联锁定保护。

每次换模型，重建所选模型的新鲜历史，按其默认角/逐轴scale把上一下发目标换算为previous_action，避免沿用另一模型的动作单位或停用前的旧历史。目标、Kp和Kd从上一下发值经0.10秒smoothstep混合到新模型实时输出；快速反向切换从当前已混合的输出继续。`advance=False`预览不切模型、不推进历史或过渡时间。

切换日志打印`xmz policy switch`；`/xmz/policy/active`（std_msgs/String）显示选中模型名。已有target trace为最终混合目标，过渡中不等于单纯default+scale×action_raw；action_raw/action_clipped仍为选中模型输出。trace只在有订阅时发送。

## 参数边界

两组模型和JSON均保留现场源字节，使用各自默认角、PD、scale及命令范围；要求29轴顺序一致。共享policies/joints.py及基础PD/零位保持改动前配置（与当前config_1.json对应），没有因加入静止模型而改全局增益。头部仍按平台配置关闭。
config 中 armature、root_pos、physics_dt 是仿真参数，真实电机不接受这些字段。effort_limit/velocity_limit 保留为训练及仿真参数；实机扭矩/速度限制由已安装硬件节点及固件执行，不能声称 JSON 会覆盖硬件限制。此次不更改底层通信量程、并联解算、零位标定和固件。

### 当前硬件增益范围（2026-11-21核对）

ARM-05 正式 hardware_elf3 SHA256 `28ddbfb64b92b29493e386f45a48c8d4eefc7b24423c8ec229eb06963e50f0b4` 与11月19日增益包manifest一致。BXI95的Kp/Kd编码上限800/50，BXI70/70L为500/10；其余型号保持各自量程。best4w的23个直驱策略关节Kp/Kd全部处于当前编码范围内。

腰Y/X及四踝在主机并联环计算PD后下发扭矩，沿用既有并联限幅。编码范围核对不代表已验收固件和实机闭环；本次不修改底层通信量程或固件。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
source install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 install/share/bxi_example_py_elf3/mods/com.xmz.walk/tests/check_mod.py install/share/bxi_example_py_elf3
# 在装有 MuJoCo 的工作站，第二个参数为原 elf4_xmz 目录：
python3 mods/com.xmz.walk/tests/check_sim2sim.py /path/to/elf4_xmz
```

check_mod检查两模型计算/历史/scale、运动与零值消息切换、失联回静止、首帧连续与PD混合、快速反向/预览/重新进入、资源缺失拒绝、遥测异常、反馈/IMU拒绝、手柄安全路由及深层外部Mod安装。可选第二参数是部署前四份资产的目录，用于逐字节核对。
check_sim2sim只比较行走模型与原参考runner，要求XML与config_1.json的armature匹配。历史单模型仿真及离线测试不能替代双模型实机运动验收，切换平滑也不保证停止时绝不抖动。
