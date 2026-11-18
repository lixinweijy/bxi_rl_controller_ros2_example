# ARM-05 ELF4 XMZ 行走

2026-11-17：使用用户提供 `elf4_xmz/elf4_2w.onnx` 和原始 `config.json`。
29 个关节按 config 的名字重排，10×96 历史从旧到新，首帧铺满；CPU ONNX 50 Hz，逐关节 action_scale、默认角和 PD。双臂参与输入输出，不使用旧 0.23 标量或旧 ELF3 参数。

## 使用

正常 Start 后，RB+B 进入新参数 PD 姿态，RB+X 进入 XMZ；RB+B 返回 PD，RB+A 全身零力矩，RB+Y 零位。零力矩和 IMU 失联保护由精简的 `com.bxi.basic_actions` 提供，旧策略资产已移除。
前进范围 [-0.5, 1.0] m/s，横移 ±0.5 m/s，转向 ±1.57 rad/s。遥控配置必须持续发布，超过 0.5 秒无消息则指令清零。反馈超过 100 ms、缺少任一策略关节、非法 IMU 或倾角超过 1 rad 拒绝策略输入；保留宿主 IMU 失联锁定保护。

## 参数边界

Mod 的 assets/config.json 和 policy.onnx 保留源字节。共享 policies/joints.py 同步 29 轴姿态、PD、动作缩放，基础 PD/零位不再加载旧模型；头部仍按平台配置关闭，已有头部补全值保持。
config 中 armature、root_pos、physics_dt 是仿真参数，真实电机不接受这些字段。effort_limit/velocity_limit 保留为训练及仿真参数；实机扭矩/速度限制由已安装硬件节点及固件执行，不能声称 JSON 会覆盖硬件限制。此次不更改底层通信量程、并联解算、零位标定和固件。

### 当前硬件增益裁剪（已确认）

ARM-05 正式 hardware_elf3 SHA256 `1851f69a9736740110a9e25ce45151ba9f84cbd231bc5e4d69921c02f3a3f652` 与 pkg 2cc8f2f 安装清单一致。配套 hw 597a4cc 的 pack_cmd 固定 kp 上限 500，kd 上限按电机表：

| 轴（左右均同） | JSON 新值 | 当前 MIT 编码结果上限 |
| --- | --- | --- |
| 髋 Y、膝 Y | kp=582.521 / kd=37.084 | kp=500 / kd=20 |
| 髋 Z、肩 Y/X | kd=7.392 | kd=5 |
| 腕 X | kd=24.183 | kd=5 |

腰 Y/X 与踝 Y/X 在主机并联环计算 PD 后下发扭矩，不按上述直驱电机增益路径裁剪。
因此 Mod 的输出与源参数一致，但当前实机有效 PD 尚未完全一致。修改通信量程必须先确认电机固件契约；本次不擅改、不做实机运动验收。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
source install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 install/share/bxi_example_py_elf3/mods/com.xmz.walk/tests/check_mod.py install/share/bxi_example_py_elf3
# 在装有 MuJoCo 的工作站，第二个参数为原 elf4_xmz 目录：
python3 mods/com.xmz.walk/tests/check_sim2sim.py /path/to/elf4_xmz
```

检查包括参数、模型、历史、逐关节缩放、乱序 31 轴反馈、预览不推进、超时/倾角拒绝、全部手柄状态切换、全身零力矩、IMU 锁定及深层外部 Mod 安装。
本次本机三组各 10 秒 MuJoCo（零指令、前进 0.3、转向 0.3）1500 帧目标与原脚本完全一致，未跌倒。未启动真实电机，不能替代吊挂/落地验收。原 README 的“尚未附带策略”为旧说明，实际交付目录已有 elf4_2w.onnx；该模型无训练 metadata，出处按用户指定。
