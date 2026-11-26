# XMZ 站立轮播

站立策略以 50 Hz 控制腰腿。两臂按 0.5 倍速依次挥手、播放一段训练动作并鼓掌。训练段从 1 到 128 依次轮换，段间平滑过渡 1 秒。

独立仿真可从本 Mod 目录运行 `python3 tools/play_mujoco.py --viewer --loop`。在已部署本 Mod 的 ELF3 控制器中，从 PD 默认姿态或 XMZ 行走状态按 **LB+X** 进入，沿用 0.6 秒平滑过渡；在 App 中对应 **XMZ 站立轮播**。按 **RB+B** 返回 PD。

| 文件 | 用途 |
| --- | --- |
| `assets/policy_tang.onnx`、`assets/controller_parameters.json` | 站立平衡策略和配套控制参数 |
| `assets/mujoco/` | ELF4 XMZ 的 MuJoCo 模型、起始配置和网格 |
| `assets/wave_motion.npz`、`assets/standing_arm_motion.npz`、`assets/clap_motion.npz` | 挥手、训练动作库和鼓掌轨迹 |

模型来自 `stand_xmz` 第 37300 轮训练。导出和校验信息见 [model_info_tang.json](assets/model_info_tang.json)。

ARM-05：正常 Start 后先按 **RB+B** 进入 PD，再按 **LB+X**；已在 walk 中时可直接按 **LB+X** 切入站立。**RB+B** 返回 PD，**RB+A** 进入全身零力矩，**RB+Y** 进入全零目标。原 **RB+X** 行走入口保持，stand 返回 walk 仍需经 PD。修改 Mod 后需重新启动控制程序加载。原配站立参数和三份动作库不改，未额外套用行走模型的腕 X 偏置。
