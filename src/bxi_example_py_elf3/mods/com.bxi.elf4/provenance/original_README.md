# ELF4 增强版 MuJoCo 部署包

模型：`model_43000`，在 2026-10-21 16:07（北京时间）锁定的最新增强版 checkpoint。
源实验为 `2026-10-21_12-06-44_upper_body_rank_from37300_seed42`，训练仍继续至 50000。
模型与源 checkpoint 的对应关系和 SHA256 记录在 `provenance/source.json`。

## 安装和运行

在解压后的本目录中执行。已在 Linux、Python 3.10 上验证；窗口模式需要可用的桌面显示。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python runtime/verify.py
bash run.sh --command 0.5 0 0 --duration 120
```

不需要 Isaac Sim、Isaac Lab、PyTorch 或训练仓库。TorchScript 文件作为备用导出提供，
若当前环境安装了 PyTorch，`verify.py` 还会自动核对 TorchScript 与 ONNX 的输出。

窗口中按 W/S 增减前进速度，A/D 增减左右侧移速度，Q/E 增减转向速度；空格将速度指令归零。
侧移速度正值向左，转向速度正值左转。关闭窗口结束运行。
`--command` 三个参数依次为前进速度（m/s）、侧移速度（m/s）、转向速度（rad/s）。

```bash
# 无窗口闭环验证
bash run.sh --headless --duration 20 --command 0.5 0 0 --report result.json
# 原地左转
bash run.sh --command 0 0 0.8
# 向右侧移
bash run.sh --command 0 -0.35 0
# 复现十方向独立测试
python validation/validate_suite.py
```

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `models/policy.onnx` | ONNX Runtime 使用的确定性 actor |
| `models/policy.pt` | 备用 TorchScript actor，同一 checkpoint |
| `config/deploy.yaml`、`deploy.json` | 等价的完整观测、控制和关节参数 |
| `config/joints.csv` | 29 关节的顺序、默认角度、动作缩放、Kp/Kd、力矩和速度限值 |
| `runtime/policy.py` | 观测构造、历史缓存、ONNX 推理、位置目标计算 |
| `runtime/run_mujoco.py` | 独立 MuJoCo 闭环控制和窗口键盘操作 |
| `assets/elf4/elf4.xml`、`meshes/` | 有碰撞及 PD 驱动的 ELF4 MJCF 和全部 OBJ 网格 |
| `validation/` | 推理参考数据、检查脚本和实际验证报告 |
| `provenance/` | 源信息、训练配置、参考 URDF、导出和转换脚本 |
| `SHA256SUMS` | 包内文件完整性校验，可运行 `sha256sum -c SHA256SUMS` |

## 策略接口

输入 `obs`，float32，固定形状 `[1, 960]`；输出 `actions`，float32，固定形状 `[1, 29]`。
ONNX opset 11，无循环网络状态，无额外的学习型观测归一化。

每帧 96 维，顺序为：躯干坐标系角速度 3、投影重力 3、速度指令 3、
关节位置相对默认角度 29、关节速度相对默认速度 29、上次原始网络输出 29。
投影重力为 `R_world_from_torso.T @ [0, 0, -1]`，不是欧拉角。
实际关节顺序从训练仿真读取，完整列在 `config/deploy.yaml`；运行时按名称映射，
不假设 MuJoCo 的关节排列与策略排列相同。

历史为 10 帧，最旧在前、最新在后；重置后的第一帧重复填满缓存。
观测裁剪至 ±100。各分量缩放本次均为 1。
历史中的上次动作使用原始网络输出，不使用位置目标或缩放后的动作。

策略周期 0.02 秒（50 Hz），物理步长 0.005 秒（200 Hz），每个策略动作保持 4 个物理步。
位置目标为 `default_joint_pos + action_scale * clip(raw_action, -100, 100)`。
动作缩放、默认角度、Kp/Kd 和力矩限值均按实际训练配置导出。
部署只运行 actor，不需要 AMP 判别器、专家动作数据、AMP 归一化器或上身排序损失。

## 已完成的验证与范围

`validation/onnx_parity.json`：ONNX 结构检查通过；120 组实际 Isaac Lab 观测下，
ONNX 与原策略的最大动作绝对误差为约 1.91e-6，观测重建误差为 0。
备用 TorchScript 与 ONNX 的最大动作误差同样为约 1.91e-6。

`validation/mujoco_forward.json`：0.5 m/s 前进闭环运行 20 秒，无倒地或 MuJoCo 数值告警，
2 秒后的平均前进速度约 0.474 m/s。
`validation/mujoco_gui.json`：带窗口闭环运行 5 秒并正常退出。
`validation/mujoco_command_suite.json`：前进、后退、原地左右转、行走左右转、左右侧移、
左右斜移各 12 秒，全部完成，无倒地或数值告警。

以上验证证明模型可加载并闭环运行，不代表每个方向的速度跟踪都达到目标。
例如左侧移指令 0.35 m/s，测试平均侧移速度约 0.023 m/s，跟踪仍不足；全部实测值已保留在报告中。
MuJoCo 与训练所用 PhysX 的接触和关节约束求解不同。

原仓库的 ELF4 XML 用于录像，机器人和地面碰撞均关闭；本包保留原几何与惯性，
增加按各连杆网格生成的凸包碰撞、训练资产的五组碰撞排除、地面摩擦和训练参数的限力位置 PD。
物理积分采用 `implicitfast`。速度限值写入配置供参考，MuJoCo 未另加关节速度硬裁剪。
`provenance/elf4_source.urdf` 和 `elf4_render_source.xml` 是源资料，不是本包运行入口；
仿真仅加载 `assets/elf4/elf4.xml`，其网格依赖已完整包含。
