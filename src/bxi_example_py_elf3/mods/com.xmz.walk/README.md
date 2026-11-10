# xmz ELF4 行走 Mod

## ARM-08当前：no_arm_30k（2026-11-10）

RB+X模型已更新为用户提供的no_arm_30k.onnx，SHA256 3915a33f1dacc56246b6b6fb998e1880f927ab258a94bdda5d56350ca752bb68，输入960、输出29。保留用户新PD参考姿态：两髋y=-0.3，左髋x/z=-0.017/-0.072、右髋x/z=0.0017/0.072 rad，其余关节及原kp/kd、0.23动作缩放、50Hz和按键配置保持。未取得新的训练JSON，部署继续使用已同步的参考配置；未做实机动作验收。下方切模型条目为历史。

## ARM-08：PD与模型参考姿态同步（2026-11-10）

按用户要求统一PD目标、共享ELF3_ISAAC_PARAMETERS及本Mod的default_joint_pos：两髋y=-0.3；左髋x/z=-0.017/-0.072，右髋x/z=0.0017/0.072 rad，其余关节按给定姿态。PD直接复用共享默认值；模型位置观测为q-default，目标为default+0.23*action。保留原Kp/Kd、50Hz、模型文件和按键配置；这是用户指定的部署参考，未取得新的训练JSON，未做实机动作验收。下方切模型说明保留为当时的历史记录。

## ARM-08历史：no_arm_10k（20261110_113725）

RB+X已换为用户提供的 `no_arm_10k.onnx`，SHA256 `0b992c15413b2ad7b35c111a866a595db8e7fca14ee9f5e92c5f691073b3abd1`。960→29，内部屏蔽14臂动作及420项历史臂观测。保留RT+X全身零力矩、head/arms=true、默认姿态/PD、0.23/50Hz、RB+A和ros_pkg；仅模型、source_run及本说明变更。无新训练JSON，未实机运动验收；下方均为历史切换记录。

## ARM-08历史：no_arm_27k（20261109_120820）

RB+X已换为用户提供的 `no_arm_27k.onnx`，SHA256 `5ad6112c848a1dd5c3c25b1f392eb8b3f9e77a6d272a001ae8d36360be38fda1`。960→29，内部屏蔽14臂动作及420项历史臂观测。保留RT+X全身零力矩、head/arms=true、默认姿态/PD、0.23/50Hz、RB+A和ros_pkg；仅模型、source_run及本说明变更。无新训练JSON，未实机运动验收；下方均为历史切换记录。

## ARM-08历史：no_arm_43k（20261106_150241）

RB+X已换为用户提供的 `no_arm_43k.onnx`，SHA256 `083256ef1128feb1fe8d95ce60322359e901ae69c7661f31608a9e50ca45d4d8`。960→29，内部屏蔽14臂动作及420项历史臂观测。保留RT+X全身零力矩、head/arms=true、默认姿态/PD、0.23/50Hz、RB+A和ros_pkg；仅模型、source_run及本说明变更。无新训练JSON，未实机运动验收；下方均为历史切换记录。

## ARM-08历史：head_no_arm_3w（20261105_131939）

RB+X已换为用户提供的 `head_no_arm_3w.onnx`，SHA256 `f488aa71333261e5a5f6846c5008ad113024b09a7f457d4c3626a2ef40b70418`。960→29，内部屏蔽14臂动作及420项历史臂观测。保留RT+X全身零力矩、head/arms=true、默认姿态/PD、0.23/50Hz、RB+A和ros_pkg；仅模型、source_run及本说明变更。无新训练JSON，未实机运动验收；下方均为历史切换记录。

## ARM-08历史：head_no_arm_5k（20261104_182448）

RB+X已换为用户提供的 `head_no_arm_5k.onnx`，SHA256 `9634a2dc80cc29a3c3a46aad2759032f68fa136902e729b8eba85423ec008006`。960→29，ONNX内部屏蔽14臂输出及相应观测；现有适配器和真实反馈保持。保留RT+X全身零力矩、头部/双臂开启、全部PD/默认姿态、0.23/50Hz和RB+A。没有新训练JSON，仅模型、source_run及本说明变更；下方切换记录为历史，未实机运动验收。

## ARM-08历史：再次切回no_arm_38k（20261104_150243）

RB+X为 `no_arm_38k.onnx`，SHA256 `adb1421579b5b60b1494c579205263cd7420b9c736094d28fcdb7fe9efdc4099`。保留RT+X全身零力矩、头部/双臂开启、全部PD/默认姿态、0.23/50Hz和RB+A。仅模型、source_run及本说明变更；下方切换记录均为历史，未实机运动验收。

## ARM-08历史：RB+X再次切回head_no_arm_15k（2026-11-04）

按用户要求再次切回 `head_no_arm_15k.onnx`，SHA256 `12c17e4a794cf09f7f8fd127b4af7179bac66e4c5070c5d9a68774831685dcf7`。保留RT+X全身零力矩、头部/双臂开启、全部PD/默认姿态、0.23/50Hz和RB+A；仅模型、source_run及本说明变更。下方no_arm_38k为历史。模型内部仍屏蔽双臂，硬件层双臂继续开启；未实机运动验收。

## ARM-08：RB+X切回no_arm_38k，RT+X零力矩（2026-11-04）

当前RB+X模型为 `no_arm_38k.onnx`，SHA256 `adb1421579b5b60b1494c579205263cd7420b9c736094d28fcdb7fe9efdc4099`。960→29，模型内部屏蔽双臂输入及输出，双臂保持既有默认姿态/PD；头与双臂继续开启（arms=true/head=true），0.23/50Hz、增益、RB+A和ros_pkg保持。没有新训练JSON。

新增RT+X→btn_2=1→基础动作zero_torque，从启动/PD/零位及三个模型状态以instant切换，31轴Kp/Kd/前馈力矩均为0，包含头和双臂。此模式释放支撑，不是下电；仅在有可靠支撑时使用。LB/RB/LT及其他面键必须松开。RB+B返回PD，RB+Y返回零位PD。下方head_no_arm_15k条目为历史，零力矩及新模型均未做实机运动验收。

## ARM-08 RB+X：head_no_arm_15k，恢复头部（2026-11-04）

当前模型为用户提供的 `head_no_arm_15k.onnx`，SHA256 `12c17e4a794cf09f7f8fd127b4af7179bac66e4c5070c5d9a68774831685dcf7`。接口仍为960→29，ONNX内部屏蔽14轴双臂的位置差、速度、上一动作输入及动作输出；部署侧保留真实反馈与现有PD，手臂目标保持默认姿态。`policy_disabled_joints=[]`、action_scale=0.23、50Hz和所有增益保持。

用户确认头部已装回，example及walk入口默认 `arms=true/head=true`，传入既有ros_pkg启动参数；反馈要求31轴（腰腿15＋臂14＋头2）。ros_pkg文件不改，RB+A及遥控路由不改。未提供训练JSON，沿用现有姿态、关节顺序和归一化配置；下方旧模型记录为历史。本次仅离线验证，不代表实机运动验收。

## ARM-08 RB+X：切回arm_5k（2026-11-03）

按用户要求，模型及config恢复为`404edc2`的arm_5k版本，SHA256 `64afa4c7eea9a40d1b6d92690769341ed3a475775fc32c0fc6275dad53a15a3f`。双臂开启、头部关闭、29轴通道不屏蔽、50Hz、action_scale=0.23及既有PD保持。保留禁止root服务写Python字节码缓存的配置。下方arm_8k条目为历史。

## ARM-08 RB+X：arm_8k（2026-11-03）

当前`assets/policy.onnx`更新为用户提供的`arm_8k.onnx`，SHA256 `4a040e2f3a8e950f2b2807bf0c8ae224d859fb9ba36df13231618ef1f9abdb47`。保持960→29、50Hz、action_scale=0.23、现有默认姿态与PD、`policy_disabled_joints=[]`，双臂开启、已拆头部关闭。没有新训练JSON；仅模型及source_run来源标记改变，RB+X路由与RB+A模型保留。下方arm_5k及更早条目为历史，不是当前模型的仿真验证结果。

## ARM-08 RB+X：arm_5k，恢复双臂（2026-11-03）

当前 `assets/policy.onnx` 为用户提供的 `arm_5k.onnx`，SHA256 `64afa4c7eea9a40d1b6d92690769341ed3a475775fc32c0fc6275dad53a15a3f`。`policy_disabled_joints=[]`，29轴全部使用真实反馈与策略动作；头部已拆除，默认关闭并屏蔽本体索引15/16（0x18000），不要求头部反馈。

example及独立walk默认`arms=true head=false`，通过EtherCAT绑定、本体健康状态检查和两路各7从站拓扑检查后启动双臂，复位步骤由本体广播同步。RB+X按键路由保持不变。未提供新训练JSON，沿用50Hz、10×96观测、默认姿态、PD与action_scale=0.23；实机效果待验收。恢复前配置已提交到设备example的`71e0dde`。下文旧部署仅为历史。

## ARM-08 RB+X：no_arm_38k（2026-11-03）

RB+X更新为用户提供的no_arm_38k.onnx，SHA256 `adb1421579b5b60b1494c579205263cd7420b9c736094d28fcdb7fe9efdc4099`。仍为960输入、29输出，模型内部屏蔽14臂输入/输出；保留部署侧臂屏蔽、action_scale0.23、现有默认姿态/PD和50Hz策略。腰pitch/roll/yaw Kd为6.904/10.356/11.231，Kp不变。没有新的训练配置；下面日期更早的模型/硬件描述仅为历史。

仅模型与source_run来源标记变更，沿用现有硬件并联公式与主机完整PD。新模型未附训练配置，离线检查不能作为实机验收。旧模型/config/安装备份位于 `~/bxi_archive/arm08_no_arm_38k_20261103/`；构建后需正常重新启动控制器才会加载，不自动启动运动。

## ARM-08 RB+X：no_arm_5k（2026-11-02）

RB+X更新为用户提供的no_arm_5k.onnx，SHA256 `7fc6b3f8bae7416dd9504b36efc2544b68e907e5240248f2f0425205399716c0`。仍为960输入、29输出，模型内部屏蔽14臂输入/输出；保留部署侧臂屏蔽、action_scale0.23、现有默认姿态/PD和50Hz策略。腰pitch/roll/yaw Kd为6.904/10.356/11.231，Kp不变。没有新的训练配置；下面日期更早的模型/硬件描述仅为历史。

仅模型与source_run来源标记变更，沿用现有硬件并联公式与主机完整PD。该模型本地短测无跌倒但部分膝踝目标超仿真XML限位，不能作为实机验收。旧模型/config/安装备份位于 `~/bxi_archive/arm08_no_arm_5k_20261102/`；构建后需正常重新启动控制器才会加载，不自动启动运动。

## ARM-08 BXI85当前配置（2026-11-01）

- RB+X：`no_arm_85_9w.onnx`覆盖`assets/policy.onnx`，SHA256 `f74ea67e5d83b733dff164fdc16e7ddcf5bf22abb476143ebc29e57f1670c7fc`；RB+B仍为基础PD。
- 左右髋Y、膝Y均换为BXI85，walk和基础PD/零位Kp/Kd为176.421/11.231，walk力矩/速度限值150Nm/20rad/s，与既有髋X的85配置一致。
- 无新训练元数据，保留10×96观测、29轴顺序、默认姿态、action_scale=0.23及14臂策略通道屏蔽；头/双臂默认关闭。
- 底层保留ELF3解算和电机侧阻尼，本体统一1000Hz，采用0x7FF、64字节CAN-FD+BRS按总线打包；物理方向与零位仍需现场验证，未自动写零或使能。下面旧配置为历史记录。

## ARM-08 当前配置（2026-10-30）

- 默认`arms=true head=false`，头部硬件已拔除，屏蔽头两轴；保留双臂，共29轴真实反馈。
- RB+A：`policy_46000_0.33.onnx`重命名覆盖`assets/policy_53500_dwaq.onnx`，SHA256 `9717d207603484d469b21ee52c2d608a7266f7e6557f7343813c973e461fd55b`。按用户提供的`elf4_waist_legs_feedback_v1`契约适配：290输入（5×58）、15腰腿动作，双臂14轴保持默认姿态，50Hz；原`policy_1500_dwaq.json`已替换为对应契约，文件名与路由保留。previous_action用未裁剪动作，额外速度滤波关闭。
- RB+X：`no_armv2_2k.onnx`重命名覆盖walk的`assets/policy.onnx`，SHA256 `76387c3cc44d23376c2c9bdb8c54f45f54dc510a4060098c9ee0e7dc612720cb`。静态文件信息为`obs [1,960]`→`actions [1,29]`；无新训练配置，沿用10×96历史、原关节顺序/PD/action_scale=0.23与14臂策略通道屏蔽。
- 旧文件在`~/bxi_ws/backups/head_off_dwaq46000_20261030_171856/`。按用户要求不跑测试、不执行推理或实机运动；下次正常Start加载。下文旧模型和开头部的描述仅为历史记录。

## 当前整机功能与合并约定（2026-10-29）

ARM-08 的 `simon_dev` 合并 `recover-branch`，同时保留基础动作、悬空跑步、XMZ walk 和 ELF4 AMP/DWAQ 四个 Mod。下文带日期的旧部署限制只描述当时状态；当前按键如下：

| 按键 | 功能 |
|---|---|
| Start / Stop | 原整机启动 / 停止，Stop及launch退出清理Voice |
| RB+B | 默认PD；悬空跑步中先平滑停止再回PD |
| RB+Y | 所有受控关节目标0rad的PD零位，不写编码器零点 |
| 单X | 仅从PD进入悬空跑步；再次按X平滑停止 |
| RB+X | no_arm_46k walk，双臂策略输入/动作屏蔽、关节保持各自默认姿态 |
| RB+A | ELF4 DWAQ 53500，12腿动作，其他身体关节保持默认姿态 |
| LB+B | ELF4 AMP 49999，完整29轴策略，要求双臂真实反馈 |

- 默认 `arms:=true head:=true`，反馈为17腰腿头+14臂共31轴；关闭头分别为29/15轴，关闭双臂保留17轴。仅允许缺少已屏蔽的双臂作为walk输入，AMP仍拒绝缺失双臂。
- 头部 `head_z_joint/head_y_joint` 由adapter的默认命令保持0rad、Kp16.747/Kd1.066；零力矩状态释放头部。walk按关节名提取模型需要的29/15轴，不将头两轴加入960维观测或29维动作。
- 普通PD/零位复用 `ELF3_ISAAC_PARAMETERS` 的默认Kp/Kd，合并不恢复旧腰/膝Kp300覆盖；模型自己的增益不改。零位是现有标定坐标的0，不等于重新标定。
- 悬空跑步仍0.5倍速、50Hz，保留ELF4倾斜髋轴补偿、URDF位置限位、反馈新鲜度及控制间隔保护；不恢复旧测试速度上限/超速切电。它没有落地平衡控制，必须双脚离地并可靠悬挂。
- 保留IMU reader priority20透传、HiPNUC原帧及walk倾角保护，XMZ继续直接CPU单计算线程ONNX；不切换RKNN或更换模型。保留硬件owner互斥、复位握手、正常preflight退出放行和状态图导出容错。
- 合并检查仅执行离线测试与构建；不代表合并后的实机带电或行走验收。遥控监听不自动热加载新配置，需正常停止后再加载；不要同时启动手动硬件工具与正式控制。

## 当前模型：no_arm_46k（2026-10-29）

`policy.onnx` 已替换为用户提供的 `no_arm_46k.onnx`，SHA256 `d5d0317c1f0f61aec124f370a605926a0e1ef23994cc32d93b38a620998f3fb2`。仍为float32 `[1,960]` 输入、`[1,29]` 输出、10帧历史；未提供新训练配置/检查点，保留现有关节顺序、默认姿态、PD和缩放参数，不把旧source_run当作本模型来源。

配置 `policy_disabled_joints` 按名称列出双臂14轴：每帧位置差、速度、上一帧action的对应项均置0；推理后的有效手臂action也置0，使目标保持各轴 `q_default`（肩/肘默认值并非全0）。腰腿仍使用全局 `action_scale=0.23`。完整29轴真实反馈、有限数/新鲜度/倾角保护及手臂PD增益保留；禁止将真实反馈清零或取消手臂反馈需求。正式Mod的action_raw保留模型原输出，action_clipped为限幅并屏蔽后的有效action。

此变更不使能或试走；离线检查不能替代物理方向/限位、IMU校准和有支撑的实机验收。上一版no_arm_12k模型、配置、说明和本次检查记录保存在ARM example的 `diagnostics/rbx_no_arm_46k_20261029_222123/`。早期测试结果对应当时旧模型，不是本模型的验收结果。


用于当前官方 `bxi_example_py_elf3` API 4 框架，复用原 ELF4 ONNX、29 关节、10×96 历史、50 Hz 策略和原 PD。RB+X 映射 `btn_1=1`，仅从原基础PD状态进入walk，RB+B返回原PD；启动零力矩仅保留作内部安全状态。

**2026-10-26 ARM-08部署副本：RB+X接入本Mod，RB+B保留原PD，其他动作绑定已移除。** 仿真发现起步偏航、目标超模型范围、高速踝电机饱和；真实凯旋双臂几何和限位尚未核实。检查详情见 [仿真与驱动审计](../../diagnostics/arm08_xmz_20261025/README.md)。

## 接口与边界

- `plugin.py` 启动加载资源，`state.py` 只通过框架反馈/目标接口运行，`advance=False` 不修改推理历史。
- 必须有腰腿15 + 左臂7 + 右臂7 共29个真实具名关节；不填充缺失手臂。反馈100 ms 内有效，倾斜/非有限数据拒绝。
- `feedback.py` 把 `/hardware/actuator_states` 的三组部分帧合并到 `/xmz/actuator_states`；组间时间差不超过30 ms，过期不输出。温度未知允许 NaN，位置/速度/力矩必须有限。
- `/motion_commands` 由框架解释，Mod 单独检查接收心跳；0.5秒停发将速度清零。清零不等于停止策略或站定，不能替代急停。
- `launch/xmz_walk_hw.launch.py` 组合新版 body、凯旋双臂、IMU、合并器和框架，配置写入 `~/.ros/xmz_walk/state_machine.yaml`。这是**真实硬件入口，会启动带电源/使能配置的驱动，不要用它做离线检查**；目前只做静态检查，未运行。
- 常驻进程退出会关闭整套；官方双臂拓扑检查器正常退出用于继续启动双臂，允许该正常退出，失败则关闭。离线检查覆盖这一分支。
- PD 及过渡阶段需外部支撑，不能依赖其自主平衡。驱动量程不等于机械限位，双臂直接 MIT 路径绕过配置的数值限制。
- `elf4_policy.py` 本地软链接到原策略；复制独立 Mod 时需展开链接，例如 `tar -h`。assets 与原 `elf4/config.json`、`policy.onnx` 相同。

## 离线验证

本机在项目根目录执行：

```bash
source scripts/env.sh
python3 mods/com.xmz.walk/tests/check_mod.py \
  diagnostics/arm08_xmz_20261025/reference/src/bxi_example_py_elf3
python3 mods/com.xmz.walk/tools/check_framework_sim.py elf4 \
  diagnostics/arm08_xmz_20261025/reference/src/bxi_example_py_elf3 \
  diagnostics/arm08_xmz_20261025/output_audit
```

第一项验证资源/状态、无副作用预览、完整反馈、超时和事件路由；第二项使用真实新框架驱动本地 MuJoCo，PD阶段模拟吊挂，行走阶段释放。无硬件节点，固定 ROS Domain187/localhost。数学与电机表审计命令见详细报告。

ARM-08 上只运行契约检查（无需电机）：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/local_setup.bash
source /opt/bxi/bxi_rl_controller_ros2_example/local_setup.bash
source /home/bxi/bxi_ws/bxi_ros2_pkg_src/install/local_setup.bash
source /home/bxi/bxi_ws/xmz/deployment/install/local_setup.bash
python3 /home/bxi/bxi_ws/xmz/mods/com.xmz.walk/tests/check_mod.py \
  /home/bxi/bxi_ws/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3
```

2026-10-25已完成独立 ARM 原生构建，安装于 `deployment/install`；没有改官方 example 源码、原遥控配置或 service，也没有上电试走。当前来源为设备官方 example `f2bbe1f` 及设备新版 ROS 工作区，运行时需后者覆盖 `/opt` 的旧驱动。

## 仿真视频回放

`tools/render_framework_video.py` 直接渲染上述检查保存的29关节和机身位姿，不重新推理、不改变轨迹、不连接硬件。并排显示直行和转向，包含时间、支撑状态及轨迹小图，镜头方位固定于世界坐标，便于观察偏航。

```bash
python3 mods/com.xmz.walk/tools/render_framework_video.py elf4 \
  diagnostics/arm08_xmz_20261025/output_audit \
  diagnostics/arm08_xmz_20261025/video/framework_walk.mp4
```

本机依赖已有 MuJoCo/OSMesa、imageio/ffmpeg、Pillow、Noto CJK 字体，无需ROS环境。视频25fps、正常速度，包含3秒PD/过渡支撑和20秒无支撑行走；结束于恢复吊挂之前，避免退出测试的根位置复位造成画面跳变。使用原ELF4视觉几何，不代表真实凯旋臂外形。


## RB+X 更新 no_armv2_6k（2026-10-30）

按用户要求将`no_armv2_6k.onnx`重命名覆盖`assets/policy.onnx`，静态输入`obs [1,960]`/输出`actions [1,29]`，SHA256 `eef2d1041438d46c73c8f3ce707fc2b3361ccea41e3856923e7877af608e6ea1`。未附新训练契约，沿用原配置及14臂策略屏蔽，未运行测试/推理或实机运动；下次启动加载。
