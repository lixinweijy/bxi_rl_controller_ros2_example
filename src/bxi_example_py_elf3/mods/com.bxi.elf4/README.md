# ELF4 Repaired 49999 Mod

## ARM-08 RB+A：policy_12500_dwaq（2026-11-03）

RB+A改为用户提供的policy_12500_dwaq.onnx，存入既有assets/policy_53500_dwaq.onnx路径，SHA256 `3ebef3e869c3b9b8b43fcb96ccf8884485407b903cb35a69762f113b64244686`。输入history为290维（5×58）、输出15轴腰腿，使用现有DWAQ适配器和btn_10=41路由；没有附带新训练JSON，沿用现有policy_1500_dwaq.json部署契约、PD/缩放、50Hz以及双臂默认姿态。旧metadata的checkpoint哈希是历史来源，不是本次ONNX哈希。

同时关闭hardware_elf3的10Hz腰部waist_torque打印；PD、限幅、并联函数和RB+X no_arm_5k保持。旧模型/源码/安装在~/bxi_archive/dwaq12500_no_waist_log_20261103。以下带旧日期的模型和硬件说明只代表历史；本次不自动启动运动，需下次正常启动加载。

## ARM-08 当前配置（2026-10-30）

- 默认`arms=true head=false`，头部硬件已拔除，屏蔽头两轴；保留双臂，共29轴真实反馈。
- RB+A：`policy_46000_0.33.onnx`重命名覆盖`assets/policy_53500_dwaq.onnx`，SHA256 `9717d207603484d469b21ee52c2d608a7266f7e6557f7343813c973e461fd55b`。按用户提供的`elf4_waist_legs_feedback_v1`契约适配：290输入（5×58）、15腰腿动作，双臂14轴保持默认姿态，50Hz；原`policy_1500_dwaq.json`已替换为对应契约，文件名与路由保留。previous_action用未裁剪动作，额外速度滤波关闭。
- RB+X：`no_armv2_2k.onnx`重命名覆盖walk的`assets/policy.onnx`，SHA256 `76387c3cc44d23376c2c9bdb8c54f45f54dc510a4060098c9ee0e7dc612720cb`。静态文件信息为`obs [1,960]`→`actions [1,29]`；无新训练配置，沿用10×96历史、原关节顺序/PD/action_scale=0.23与14臂策略通道屏蔽。
- 旧文件在`~/bxi_ws/backups/head_off_dwaq46000_20261030_171856/`。按用户要求不跑测试、不执行推理或实机运动；下次正常Start加载。下文旧模型和开头部的描述仅为历史记录。

## ARM-08 接入（2026-10-29）

- RB+A → `activate_dwaq`（`btn_10=41`），实际加载 `assets/policy_53500_dwaq.onnx`；内部状态名沿用 `elf4_dwaq_1500`，模型输入245维、输出12个腿部动作。
- 可从 PD 或 `com.xmz.walk/walk` 切入；RB+B 返回 PD，RB+X 返回原 xmz 模型。
- 正式硬件入口默认 `arms:=false`，不启动 EtherCAT 双臂节点或双臂拓扑等待，仅使用15轴真实腰腿反馈。DWAQ 仅读取12个腿部关节；框架按实际15轴布局发布命令，不输出双臂目标。显式 `arms:=true` 可恢复原双臂链路。
- 当前框架资源按需加载并缓存，退出统一由框架关闭；移除了原包使用但本机框架尚不支持的 `ResourceHandle.release()` 调用。
- 2026-10-29 已同步 ARM-08 正式源码和安装态并重载遥控监听；本机及 ARM 离线检查通过，实体按键和带电运动未测试。备份在 ARM `~/bxi_ws/backups/elf4_dwaq_20261029_133924/`。

独立 Mod：`com.bxi.elf4`；状态：`com.bxi.elf4/elf4`。
模型和碰撞资产来自 `elf4_mujoco_model49999_repaired_20261023_150954.zip`；
2026-10-28 将电机参数覆盖为 `deploy_xmz.zip` 的配置。
ONNX 仍为同一个 `model_49999`，训练 run 为
`2026-10-22_13-25-26_elf4_motor_adapted_from_scratch_seed42`，输出 **29 个全身关节动作**。
ONNX 元数据记录 `upper_body_amp=disabled`；本次修订主要更换实际 PhysX cooked
碰撞凸包、物理积分器和速度约束。原 43000 包仅保留为历史契约与验证记录。

## 模型与契约

- `assets/policy.onnx` 是迭代 49999 的导出模型；`assets/deploy.json` 保存当前有效部署契约。
  `assets/deploy_xmz_config.json` 保留本次参数来源，`assets/motor_parameters.json`
  保留此前电机选型表。本次只导入电机控制和被动关节参数，未替换包内的 37600 模型。
- MyActuator X12 的髋俯仰和膝关节为 `Kp=150, Kd=15`；Kaixuan 肩、肘、腕为
  `Kp=50, Kd=10`。其他关节增益继承原 ELF3 参数，默认姿态及动作缩放按参数文件保持不变。
- 29 组 Kp/Kd 与此前完全相同。力矩与速度限额按来源配置的 `joint_control` 逐关节读取，
  速度单位直接为 rad/s。腰 Y/Z 力矩为 `90/150 Nm`，髋 X/Z 为 `150/45 Nm`；
  腰和髋 X/Z 速度为 `20 rad/s`，髋 Y/膝为 `15 rad/s`，上肢为 `3.14 rad/s`，
  踝 Y/X 为 `20/10 rad/s`。
- 29 个电机关节的 `armature` 均为 `0.02 kg·m²`；被动 `joint damping` 和
  `frictionloss` 均为 0。PD 的 Kd 不重复写入被动阻尼。
- 输入 `obs` 为 float32 `[1,960]`，输出 `actions` 为 `[1,29]`。
  每帧 96 维依次为角速度、投影重力、速度指令、相对默认角度的关节位置、
  相对默认速度的关节速度和上一帧原始动作；10 帧按最旧到最新堆叠，
  重置时以首帧填满，不含步态相位。
- 按关节名称映射观测和输出，并从契约读取默认姿态、动作缩放及 PD 增益。
  观测裁剪至 ±100；位置目标为 `default + scale * clip(raw_action, -100, 100)`，
  上一帧动作保存裁剪及缩放前的原始输出。
- IMU 输入使用平台发布的 **torso** 姿态与角速度；四元数输入为
  `wxyz`，不额外转换为腰部以下的 root 坐标系。
- 推理频率为 **50 Hz**，使用 CPU ONNX Runtime；过渡预览不推进观测历史和上一帧动作。
- 速度指令范围：`vx [-0.6, 1.0] m/s`、`vy [-0.5, 0.5] m/s`、
  `yaw [-1.57, 1.57] rad/s`。手柄指令经过速度配置和策略前限幅。
- Mod 输出关节位置目标和 PD 增益。ELF4 仿真从契约按名称加载物理力矩限额，
  并使用质量矩阵加权的关节速度冲量投影，保留浮动基座的耦合反应；
  不再逐个裁切关节速度。这仍是仿真限额，不表示实测力矩速度曲线。
- 模型加载时检查输入输出维度与关节配置，不做文件哈希校验；仿真启动时检查 MuJoCo 版本、积分器、步长、
  控制周期、原始 XML 的 PD、力矩上限及 armature/被动阻尼/摩擦，拒绝混用配置。

原电机选型工具 `tools/update_elf4_motor_parameters.py` 默认不会覆盖当前 XMZ 参数。
若明确要重新采用旧电机选型表，须显式传入 `--parameters`；这会替换当前力矩和速度限额。
参数改变后须同步契约与 ELF4 XML 并重新构建，ROS 才会使用安装目录中的新配置。
原包运行器通过 `qfrc_applied` 显式裁剪力矩；本 mod 保留 repaired 的隐式 PD 和
质量矩阵速度投影。原包读入速度上限但没有实际限速，因此这里不是完整复制其物理运行方式。

## 进入和退出

在仿真启动命令后选择新机器人：

ELF4 使用独立的 ROS Python 3.10 / MuJoCo 3.17.0 环境。当前机器已配置；
新环境首次使用时，在 `bxi_rl_controller` 工作区运行 `bash tools/setup_elf4_sim.bash`。
该环境与 ELF3 陪练使用的 `.venv/sparring_sim` 独立。

```bash
ros2 launch bxi_example_py_elf3 example_launch_demo.py robot:=elf4
```

遥控节点也选择相同机器人（另一个已加载工作区环境的终端）：

```bash
ros2 launch remote_controller remote_controller.launch.py DEBUG:=true robot:=elf4
```

该遥控配置保留零力矩和 PD 刹车，并提供两个独立的模型状态：

| 模型 | 键盘 | 手柄 | 事件 |
| --- | --- | --- | --- |
| 原全身 49999：`elf4` | `1` | RB+X；或 LB+RB+RT+B（松开 LT） | `btn_10=40` |
| 双腿 DWAQ 53500：`elf4_dwaq_1500` | `2` | LB+A；或 LB+RB+RT+X（松开 LT） | `btn_10=41` |

两个状态可直接互切，使用框架的 `soft_switch` 过渡。启动仍默认进入原 49999 状态。
零力矩后先进入 PD 刹车，再选择任一模型。ELF3 的按键映射不变。

当前 DWAQ 模型文件由 `plugin.py` 中的路径指定，为 `assets/policy_53500_dwaq.onnx`。
以后替换同结构模型，修改这处文件名并重新构建即可，不需要更新哈希。
内部状态名 `elf4_dwaq_1500` 保留以兼容已有切换路由，界面显示当前的 53500 模型。
`assets/policy_1500_dwaq.json` 是此前提供的观测/控制契约，53500 模型沿用该配置；
`assets/policy_1500_dwaq_deploy.json` 保存命令范围和契约路径。新模型使用 torso IMU、5×49 按时间
顺序堆叠的观测、12 个腿部动作、0.8 s 步态周期及 50 Hz 控制；重置时首帧填满历史。
原始上一帧动作与预览状态独立保存，预览不推进相位和历史。
腰部和上肢共 17 个关节保持新契约的默认姿态与 PD，不使用原 49999 的默认腿姿态。
部署声明 `extra_velocity_filter=false`，适配器不另加速度滤波。
共享仿真继续使用现有 XMZ 物理限额；新训练契约的踝 Y/X 速度为 15/5 rad/s，
共享仿真为 20/10 rad/s。该处差异保留在部署记录中，新增切换键不会改写共享物理参数。

该参数以当前默认 `data/mujoco_simulation/elf3.xml` 为场景模板，仅将机器人部分
替换为 `elf4/elf4.xml` 中的 ELF4 资产，保留默认地形几何、视觉、相机和灯光。
ELF4 使用修复包的 `discrete` 积分器、5 ms 物理步长和 50 Hz 控制周期；
生成场景中的公共主平面摩擦设为配套的 `0.8`，其它特殊地形参数保持原样。
位置 PD 与接触约束在同一有效惯量中求解。
生成的临时 XML 在 launch 退出时清理，原始默认 XML 不会被改写。
ELF4 使用 XML 中的 29 关节机器人及 1.35 m 初始根节点高度，沿用默认 ELF3 的
两步复位流程；第一步恢复训练默认关节姿态，释放后自动进入本策略。
模型通过框架的默认过渡准备和预热。
ELF4 仿真子进程单独限制 BLAS 为单线程：速度约束只求解最多 35×35 的小矩阵，
默认多线程会在运动时引入严重调度开销。GUI 使用独立状态副本和同步线程，
不占用物理步进/指令接收的锁；RGB/depth 队列只保留最新一帧。
鼠标拖拽力保留，超过 200 ms 的旧 GUI 扰动力会丢弃；GUI Physics 面板不修改
物理副本中已校验的部署参数。机器人复位继续使用 ROS 复位流程。
默认不带 `robot` 参数仍使用 ELF3 的原 XML 和初始化流程；ELF4 暂不支持
`hands:=true` 或 `sparring:=true`。

复用已有遥控器预留组合 **LB+RB+RT+B（松开 LT）**，发送
`btn_10=40`；无需修改或新增遥控键位。从 Normal 或 PD Brake 进入。
使用现有 Normal、Zero Torque 或 PD Brake 控制退出。

策略资源 `com.bxi.elf4/policy` 使用 `on_demand`：请求进入此状态时才创建
ONNX 推理资源。退出过渡完成、已准备的进入过渡被取消、预热失败或状态解绑时，
状态向框架请求异步释放；退出过渡仍在采样时保留策略资源。
若资源尚未完成加载就取消进入，当前框架不会调用状态的取消钩子，资源会缓存到
下次进入后退出或控制器关闭；此时不会执行 ELF4 策略控制。
`setup.py` 会自动将完整 Mod 目录安装到包的 `share/.../mods/`，
安装后由框架发现 `mod.yaml` 并注册。

部署时须同步本次 `framework/runtime/state_machine.py` 的资源请求修复及支持
`ResourceHandle.release()` 的框架：策略尚在卸载时再次进入，会排队重新加载，
避免状态一直等待一个已经卸载的资源。

## 验证边界

`validation/active_model_report.json` 记录当前 ONNX 的 SHA256、元数据、输入输出、
CPU 推理及电机参数检查。修复包新增的 11 个工况、2200 帧原生 Isaac 参考已单独回放：
观测最大误差 `0`，原始动作最大误差 `3.93e-6`，关节目标最大误差 `1.19e-6 rad`。
参考为 `validation/isaac_reference_49999.npz/json`，不复用旧模型动作作为新模型基准。
原生参考中的力矩和速度限额属于 49999 的原训练配置；XMZ 覆盖后的限额不再宣称与该
训练配置相同。变更前契约见 `provenance/deploy_before_xmz_20261028.json`，
本次按关节记录的差异见 `provenance/deploy_xmz_parameters_20261028.json`。
此前 2026-10-23 的闭环与延迟报告仍是覆盖前的历史结果。

`validation/bxi_adapter_report.json` 是 **43000 旧模型** 的历史验证：120 组 Isaac
参考数据的观测重建最大误差为 0，原始动作最大误差为 `1.91e-6`，
关节目标最大误差为 `4.77e-7 rad`。参考数据仅包含 `vx=0.5, vy=0, yaw=0`；
关节乱序、重置、多轴重力投影、异常输入、预览和资源生命周期另有单元测试。
`provenance/original_*` 为原部署包的资料或报告；`provenance/deploy_43000.json`
保留旧契约，`provenance/active_model.json` 记录当前替换来源。旧动作参考数据不能验证新模型动作，
测试仅用它重建继承的观测，并独立比较当前 ONNX 与适配器输出。

在 HoloMotion 仓库根目录运行离线测试（关闭无关 ROS pytest 插件的自动加载）：

```bash
source train.env
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/bxi_rl_controller/src/bxi_example_py_elf3${PYTHONPATH:+:$PYTHONPATH}" \
  "$Train_CONDA_PREFIX/bin/python" -m pytest -q \
  bxi_rl_controller/src/bxi_example_py_elf3/test/test_elf4_mod.py \
  bxi_rl_controller/src/bxi_example_py_elf3/test/test_resource_manager_release.py
```

当前修复版的静止、前进和左右侧移各 60 秒闭环见 `validation/repaired_closed_loop_report.json`，
分别记录直立、脚间距和速度跟踪指标；不以“未倒地”代替侧移跟踪通过。
静止末 10 秒脚间距均值 `0.2507 m`；左/右 `vy=±0.35` 时末 10 秒实际横移速度
约为 `+0.286 / -0.168 m/s`，右移仍偏慢，前进速度与 Isaac 仍有差异。
`validation/repaired_physics_parity_60s.json` 记录原包物理与适配器 60 秒
逐帧 `qpos/qvel/target` 零误差对照；`validation/repaired_ros_runtime_report.json`
记录完整 ROS 图形仿真验证。
可用 `tools/validate_elf4_closed_loop.py --duration 60 --command 0 0 0`
（相对 BXI 工作区，使用 `OPENBLAS_NUM_THREADS=1 .venv/elf4_sim/bin/python`）复现。
`validation/motor_model_closed_loop_report.json` 是此次修复之前的历史报告。
仿真资产目录的 `closed_loop_report.json` 是 2026-10-21 的 **43000 旧模型** 历史报告。
尚未进行真机动作验证。


## 2026-10-29：RB+A 使用53500

模型assets/policy_53500_dwaq.onnx，SHA256 1afa1dce15af3f15e19ebb12bbc240cb643c42bfb90312d8baae6ae77616d61e，245输入/12腿动作，沿用现有DWAQ契约；未提供53500独立训练配置。原事件/路由/内部状态名及双臂关闭默认保持，下次启动控制程序加载。旧37500模型保留，本轮未启动控制。


## 2026-10-29：ARM-08 LB+B与完整双臂控制

用户确认恢复双臂。正式example_demo_hw和Mod xmz_walk_hw入口的arms默认true，启动双臂拓扑等待/EtherCAT节点，反馈合并必须取得15腰腿+7左臂+7右臂真实反馈。AMP不补双臂假数据，动作和双臂目标按原完整29轴契约执行。

| 按键 | 功能 |
| --- | --- |
| LB+B | AMP49999：assets/policy.onnx，btn_10=40 |
| RB+A | DWAQ53500，btn_10=41 |
| RB+B | 默认PD初始姿态 |
| RB+X | no_arm_12k，策略屏蔽双臂、手臂PD保持默认姿态 |
| RB+Y | 零位PD（当前全部29轴） |

AMP模型SHA256：83ab3bd76a55af008025b081bb0f59449d218dfefb5b0e838193f38ea250b988；输入obs[1,960]、输出actions[1,29]，配套assets/deploy.json。不复用xmz根目录的其他同名policy.onnx。

启动方式：按Start启动整套控制（含双臂），等待两步复位和真实29轴反馈，再RB+B进入PD，LB+B切入AMP；RB+B返回PD。双臂开启是全局启动配置，RB+A/RB+X同样会输出原手臂保持目标；使用arms:=false仅腰腿时不能进入需要完整29轴反馈的AMP。

本机/ARM源码各7项相关检查，ARM安装态7项检查以及两端真实InputMapper的512种手柄/CRSF组合检查通过。验证包含完整29轴命令映射、不完整/过期反馈拒绝、策略预览和返回路由。当前框架没有ResourceHandle.release，保留设备已有兼容Elf4State，未直接覆盖原包含该新API的state.py；模型/契约/观测/动作保持原样。只部署并重载遥控监听，未启动或验证带电动作。


## RB+A 更新 policy_54000_waist（2026-10-30）

用户新模型`policy_54000_waist.onnx`重命名覆盖原`assets/policy_53500_dwaq.onnx`，SHA256 `26db95d132718d8ae6439d347e53993d0b10d2c0b74cf471afdb401de736fd2f`，静态声明仍history290→actions15；未提供新JSON，沿用用户已确认腰腿契约、5×58历史、默认姿态/增益/缩放及双臂默认PD。按键/内部状态名不变，未运行模型测试或推理，下次启动加载。


## ARM-08 BXI85替换（2026-11-01）

四个原脉塔髋Y/膝轴现为BXI85。AMP部署配置与DWAQ运行增益均使用176.421/11.231，部署限值150Nm/20rad/s；DWAQ原训练契约保留，运行增益引用基础PD参数表。`motor_parameters.json`与备用XMZ部署配置同步85选型。模型文件及按键不变；RB+X的新模型在com.xmz.walk中。
