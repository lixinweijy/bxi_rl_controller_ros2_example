# ARM-05 必要基础状态

仅保留框架固定引用的零力矩、IMU 失联锁定、PD 姿态和零位；没有旧动作模型。
PD 使用共享 ELF3_ISAAC_PARAMETERS 的最新 XMZ 默认角及 kp/kd，零位使用同一套 kp/kd、目标角为零。
RB+A 零力矩、RB+B PD、RB+Y 零位。XMZ 的按键、部署和验证见 com.xmz.walk/README.md。
