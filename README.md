# Hexapod Sim2Real

本工程由 Unitree 实机部署框架改造而来，用于将 PyTorch/TorchScript 策略部署到纯六足机器人和绳驱六足机器人。工程包含策略推理、手柄状态机、IMU/张力/绳方向传感器、EtherCAT 电机通信、绳轮张力控制、数据采集和离线建模工具。

> 安全提示：本工程会向实机电机下发位置、速度或力矩指令。运行前必须架起机器人、确认电机映射/方向/零位、传感器和手柄按键，并确保 LB 阻尼退出可用。不要在电机未卸载或人员位于机器人运动范围内时直接运行。

## 1. 控制流程

当前重点入口 `deploy_real_hexapod_estimate.py` 的基本状态机是：

1. 启动后进入零力矩状态。
2. 按 `X` 运动到默认姿态。
3. 按 `A` 进入策略控制。
4. 策略运行或默认姿态等待期间按 `LB` 进入阻尼状态。
5. 策略运行时按 `X` 返回默认姿态，之后可按 `A` 继续策略，或按 `LB` 进入阻尼。

## 2. 目录结构

| 路径 | 用途 |
| --- | --- |
| `deploy/deploy_real/` | 实机部署主程序、配置读取、数据记录和离线工具。 |
| `deploy/deploy_real/configs/` | YAML 运行参数：策略路径、控制周期、观测/动作维度、电机映射、增益和绳控制参数。 |
| `deploy/deploy_real/convert_to_jit/` | 将 RSL-RL/MJLab checkpoint 转换并校验为实机部署用 TorchScript 策略。 |
| `deploy/deploy_real/data_logs/` | 绳驱整机实验的分程序、分时间戳 CSV 日志，以及离线生成的曲线图。 |
| `deploy/deploy_real/deploy_only_cable_data/` | 单独调试绳轮时保存的速度/力矩控制 CSV 数据。 |
| `deploy/deploy_real/pre_data/` | 张力预测训练使用的原始轨迹、合并数据和快速检查图。 |
| `deploy/pre_train/` | 部署用 checkpoint 和 TorchScript 策略；可按训练批次建立日期/编号子目录。 |
| `deploy/deploy_real/common/` | 安全指令、Unitree 遥控器解析和坐标旋转工具。 |
| `deploy/deploy_real/motor_igh_sdk/` | EL4090 + PySOEM 电机通信及 PD/位置/速度/力矩控制封装。 |
| `deploy/deploy_real/imu_sdk_deta40/` | 当前 DETA40/FDILink IMU 读取、坐标系转换和速度估计。 |
| `deploy/deploy_real/hexapod_tethered_utils/` | 绳驱传感器、手柄、张力控制、运动学和 MoE/预测模型部署工具。 |
| `deploy/deploy_real/unitree_sdk2py/` | Unitree SDK2 Python/IDL 支持文件，主要服务于原 G1/H1/Go2 部署框架。 |

## 3. `deploy/deploy_real/` 脚本说明

### 3.1 实机部署入口

| 脚本 | 作用 |
| --- | --- |
| `deploy_real_hexapod.py` | 纯六足主程序；组装 66 维观测，调用 18 维策略，下发 18 个关节 PD 指令，并保存每个电机的跟踪日志。 |
| `deploy_real_hexapod_estimate.py` | **纯六足 Estimate 主程序（当前重点入口）**；运行带 GRU 状态估计器的 MJLab 策略，以 63 维可观测状态在线估计机体线速度和 6 个足端高度，再输出 18 维关节动作。 |
| `deploy_real_hexapod_tethered.py` | 基础绳驱入口；在腿部策略之外读取张力、绳端俯仰角和绳臂航向角，通过转速模式控制绳轮。 |
| `deploy_real_hexapod_tethered_speed.py` | 带实时曲线和 CSV 日志的绳轮速度控制实验版。 |
| `deploy_real_hexapod_tethered_plot.py` | 基础速度式绳驱程序的绘图/日志版。 |
| `deploy_real_hexapod_tethered_plot_torque.py` | 将绳轮改为力矩模式，记录并绘制张力、航向、速度和绳轮力矩。 |
| `deploy_real_hexapod_tethered_plot_torque_data.py` | 力矩式部署加训练数据采集；以因果时序记录状态、上一步/当前力矩和绳张力，供 `pre_tension.py` 使用。 |
| `deploy_real_hexapod_tethered_plot_torque_data_estimate.py` | 带 GRU 状态估计器的绳驱力矩部署/采集版；使用 estimate 策略并记录完整轨迹数据。 |
| `deploy_real_hexapod_tethered_plot_torque_data_constant.py` | 恒定绳轮力矩对照版；当前代码把绳轮指令固定为 13.14 Nm，用于基线比较。 |
| `deploy_real_hexapod_tethered_plot_torque_data_trex.py` | TReX 文献方法对照版；根据斜坡、摩擦和机器人速度生成确定性目标张力。 |
| `deploy_real_hexapod_tethered_plot_torque_data_nagatani.py` | Nagatani 方法对照版；利用机器人姿态、绳方向和 MJCF 运动学计算目标张力。 |
| `deploy_real_hexapod_tethered_plot_torque_data_polzin.py` | Polzin–Hughes 方法对照版；基于六足接触/支撑几何搜索可用拉力方向和大小。 |
| `deploy_real_hexapod_tethered_plot_torque_data_moe.py` | 递归 MoE 策略部署版；使用 68 维观测、GRU 隐状态和多专家 actor，同时记录绳控制数据。 |
| `deploy_real_hexapod_tethered_plot_torque_pre.py` | 短时域张力预测模型部署版；在候选绳轮力矩中选择预测张力更合适的指令，并保留 shadow/debug 数据。 |
| `deploy_only_cable.py` | 不运行腿部策略，单独调试张力跟踪和绳轮速度控制，数据保存到 `deploy_only_cable_data/speed/`。 |
| `deploy_only_cable_torque.py` | 不运行腿部策略，单独调试张力跟踪和绳轮力矩控制，数据保存到 `deploy_only_cable_data/torque/`。 |
| `deploy_real.py` | 原 Unitree G1/H1/H1_2 通用部署入口，通过 DDS/SDK2 与原厂机器人通信。 |
| `deploy_real_go2.py` | 原 Unitree Go2 部署入口，主要用于保留原工程参考。 |

### 3.2 配置读取

| 脚本 | 作用 |
| --- | --- |
| `config_hexapod.py` | 读取 `configs/hexapod.yaml`，提供纯六足策略、观测、动作和电机映射参数。 |
| `config_hexapod_estimate.py` | 读取 `configs/hexapod_estimate.yaml`，提供 Estimate 策略路径、63 维观测、18 维动作、缩放系数和电机映射参数。 |
| `config_hexapod_tethered.py` | 读取绳驱 YAML，除腿部参数外还包含绳轮速度/力矩控制器增益和限幅。 |
| `config_hexapod_tethered_moe.py` | 在绳驱配置上增加 MoE/GRU 策略路径、观测合同和部署检查。 |
| `config.py` | 原 G1/H1/H1_2 通用 YAML 读取器。 |
| `config_go2.py` | 原 Go2 YAML 读取器。 |

`configs/hexapod.yaml` 对应普通纯六足策略；`hexapod_estimate.yaml` 对应纯六足 Estimate 策略；`hexapod_tethered.yaml` 对应普通绳驱策略；`hexapod_tethered_estimate.yaml` 对应绳驱 Estimate 策略；`hexapod_tethered_moe.yaml` 对应 MoE 策略。`g1.yaml`、`go2.yaml`、`h1.yaml` 和 `h1_2.yaml` 为原 Unitree 机型配置。

### 3.3 纯六足 Estimate 实机部署（重点）

`deploy_real_hexapod_estimate.py` 用于部署带在线状态估计器的纯六足策略。它不再把 IMU 积分线速度直接放入 Actor 观测，而是让 GRU 根据连续的本体感知历史估计不可直接可靠测量的状态，因此该入口的模型结构、观测排列和 GRU 历史都必须与 MJLab 训练时完全一致。普通 66 维纯六足 MLP 策略不能用于这个入口。

#### 策略数据流

每个 20 ms 控制周期按以下顺序运行：

1. 从 EtherCAT 读取 18 个关节的位置和速度，从 `imu_sdk_new` 读取机体角速度与单位重力方向，从 Xbox 手柄读取 `vx/vy/wz` 指令。
2. 按训练顺序组装 63 维观测：`角速度(3) + 重力方向(3) + 相对关节位置(18) + 关节速度(18) + 上一步动作(18) + 速度指令(3)`。
3. 观测先使用 checkpoint 中的均值和标准差归一化。GRU 使用前 60 维历史，输出 9 维估计量：`机体线速度(3) + 六个足端高度(6)`；速度指令的最后 3 维不进入 GRU。
4. Actor 使用 `归一化后的 63 维观测 + 9 维估计量` 输出 18 维动作。主程序将动作限制到 `[-10, 10]`，再按 `目标关节角 = default_angles + action × action_scale` 下发关节 PD 指令。
5. 当前配置为 50 Hz（`control_dt: 0.02`）、`action_scale: 0.25`；策略阶段关节增益为 `kp=100`、`kd=3`，移动/保持默认姿态时为 `kp=150`、`kd=11`。

#### 63 维观测输入

以下切片采用 Python 左闭右开规则，例如 `obs[0:3]` 表示索引 0、1、2。

| Obs 范围 | 维度 | 输入内容 |
| --- | ---: | --- |
| `obs[0:3]` | 3 | IMU 机体角速度 `[ωx, ωy, ωz] × ang_vel_scale` |
| `obs[3:6]` | 3 | IMU 计算的机体系单位重力方向 `[gx, gy, gz]` |
| `obs[6:24]` | 18 | 相对关节位置 `(q - default_angles) × dof_pos_scale` |
| `obs[24:42]` | 18 | 关节速度 `dq × dof_vel_scale` |
| `obs[42:60]` | 18 | 上一个控制周期经 `[-10, 10]` 限幅后的策略动作 |
| `obs[60:63]` | 3 | 手柄速度指令 `[vx, vy, wz] × command_scale` |

18 个关节数据的固定顺序为 `RF → RM → RB → LF → LM → LB`，每条腿内部依次为 `HAA → HFE → KFE`。`obs[0:60]` 送入 GRU，用于估计机体线速度和六个足端高度；`obs[60:63]` 的速度指令不进入 GRU。完整 63 维观测与 GRU 输出的 9 维估计量拼接后送入 Actor。策略内部还会使用训练时保存的 observation mean/std 对观测归一化，因此不能调整这些输入的顺序。

策略封装会根据文件内容自动识别两种模型：

- MJLab 原始 `model_*.pt`：直接读取 `actor_state_dict`，并在部署端重建归一化器、GRU、估计头和 MLP Actor。
- 转换后的 `policy_*.pt`：加载带 `reset()` 的递归 TorchScript。当前 `configs/hexapod_estimate.yaml` 默认使用 `deploy/pre_train/hexapod_estimate/1020_flat/policy_24500.pt`。

网络启动时会预热 10 步并立即清空 GRU 历史；每次从默认姿态按 `A` 开始策略前，也会同时清空 GRU 历史和上一步动作，避免把预热或上一段运行的时序状态带入新一轮控制。

#### 运行前配置

重点检查 `configs/hexapod_estimate.yaml` 中的以下字段：

| 字段 | 当前值/含义 |
| --- | --- |
| `policy_path` | Estimate 原始 checkpoint 或递归 TorchScript；不能填普通 Hexapod MLP 策略。 |
| `control_dt` | `0.02` s，即 50 Hz。 |
| `num_obs` / `num_actions` | 必须分别为 `63` / `18`。 |
| `command_scale` | `[0.8, 0.5, 0.8]`，依次缩放 `vx/vy/wz`。 |
| `action_scale` | `0.25`，将策略动作换算为目标关节角增量。 |
| `joint2motor_idx` | 训练关节顺序到实机 motor ID 的映射。 |
| `motor_directions` / `motor_offsets` | 电机安装方向和实机零位；必须使用本机标定结果。 |
| `default_angles` | 进入策略前的默认关节姿态，当前 18 个值均为 0。 |

> **重要：当前存在两处硬件配置来源。** Estimate 主程序从 `hexapod_estimate.yaml` 读取策略参数和 `joint2motor_idx`；但 `motor_igh_sdk/deploy_real_el4090_pysoem.py` 内部仍从 `hexapod_tethered.yaml` 读取实际收发使用的 `joint2motor_idx`、`motor_directions` 和 `motor_offsets`。运行前必须确认这两份 YAML 的三组电机参数完全一致，否则策略索引、反馈位置和下发目标可能错位。

当前网卡名和 IMU 串口也不是命令行参数：

- EtherCAT 网卡在 `deploy_real_hexapod_estimate.py` 中创建 `RL_Real_PySOEM('enp86s0')` 时固定；换机器后需先改为实际网卡名。
- IMU 固定使用 `/dev/ttyUSB0`、`921600` baud。必须确认设备名、安装方向和角速度/重力坐标系与训练一致。
- 脚本当前不解析 `{net_interface}` 命令行参数，不要照搬 `deploy_real.py enp3s0 config.yaml` 的调用方式。

#### 启动命令与手柄状态机

在仓库根目录运行：

```bash
sudo /home/lgl/anaconda3/envs/mujoco_rl/bin/python deploy/deploy_real/deploy_real_hexapod_estimate.py
```

启动顺序如下：

1. 架起机器人并连接 EtherCAT、DETA40 IMU 和 Xbox 手柄；启动后检查终端中的策略预热、18 个日志文件、EtherCAT OP 状态、IMU 数据和初始关节位置。
2. 零力矩状态按 `X`，程序用 2 s 从当前关节位置插值到 `default_angles`。
3. 默认姿态保持状态按 `A`，清空递归状态后进入 Estimate 策略控制。
4. 策略运行时按 `X` 返回默认姿态；随后可按 `A` 重新开始一段策略控制。
5. 默认姿态等待或策略运行时按 `LB` 进入持续阻尼模式；最后按 `Ctrl+C` 完全退出。

手柄按键编号取决于 Linux/SDL 驱动。默认映射和 `HEXAPOD_GAMEPAD_PRESET`、`HEXAPOD_GAMEPAD_BTN_*` 覆盖方式见 `hexapod_tethered_utils/joystick_reader.py`，上电前应先确认 `X/A/LB` 与摇杆方向。

> **安全注意：** 当前程序在 EtherCAT 或 IMU 启动失败时只打印警告，并可能继续使用零反馈或安全默认 IMU 值，不会自动禁止进入策略。看到 `Robot start failed`、`IMU start failed`、EtherCAT 未进入 OP、IMU 数据不更新或关节初始位置异常时，不要按 `X/A`，应直接退出并排查。`LB` 进入阻尼后程序仍会持续下发阻尼指令，需要再次 `Ctrl+C` 才会结束进程。

#### 日志与模型转换

每次运行会在 `deploy/deploy_real/motor_logs/hexapod/motor_logs_<时间戳>/` 下创建 18 个 CSV。文件名同时包含 motor ID 和 policy index，每行记录时间、目标位置、实际位置、误差和策略动作，可使用 `plot_motor_curves.py` 绘制跟踪曲线。

若要把新的 MJLab Estimate checkpoint 转为递归 TorchScript，使用 `deploy/deploy_real/convert_to_jit/convert_mjlab_hexapod_estimate_checkpoint_to_jit.py`。转换前检查脚本顶部的输入/输出路径；转换器会自动推断网络维度并用多步时序逐项比较原模型和导出模型。主入口也能直接加载结构匹配的原始 `model_*.pt`，但 TorchScript 更适合固定部署环境。

### 3.4 数据、模型与调试工具

| 脚本 | 作用 |
| --- | --- |
| `convert_checkpoint_to_jit.py` | 从 RSL-RL checkpoint 重建 actor/观测归一化器，导出实机用 TorchScript policy。脚本内含固定输入/输出路径，使用前需检查。 |
| `convert_mjlab_checkpoint_to_jit.py` | 从 MJLab 的 MLP actor checkpoint 重建归一化器和策略，导出并数值校验 TorchScript；当前指向 `hexapod_tethered/1015_1103`。 |
| `convert_mjlab_hexapod_estimate_checkpoint_to_jit.py` | 从 MJLab GRU-estimator checkpoint 自动推断观测、隐状态、估计量和动作维度，导出带 `reset()` 的递归 TorchScript 并逐时序校验。 |
| `convert_moe_checkpoint_to_jit.py` | 将 68 维双 GRU、6-expert MoE checkpoint 导出为实机部署用 recurrent TorchScript，并逐帧校验全部输出；输入和输出路径在脚本顶部配置，运行时无需传参。 |
| `check_keys.py` | 打印 PyTorch checkpoint 的 key，检查 actor 和 observation normalizer 是否存在。当前模型路径在脚本内硬编码。 |
| `test_load_model.py` | 加载/检查部署模型的小型测试脚本。 |
| `plot_motor_curves.py` | 读取 `motor_logs_*` 中的 18 个电机 CSV，按 HAA/HFE/KFE 绘制目标位置和实际位置曲线。 |
| `plot_tethered_data.py` | 读取一条绳驱综合 CSV，绘制实测/目标张力、前向速度指令和绳轮指令/实测力矩；在脚本顶部设置 `CSV_PATH`，图片保存到 CSV 同目录。 |
| `plot_tethered_speed_data.py` | 读取绳轮速度模式综合 CSV，绘制张力、前向速度指令和绳轮指令/实测转速；在脚本顶部设置 `CSV_PATH`。 |
| `replay_rear_leg_trajectory.py` | 在右后腿或左后腿重放已记录轨迹；其他关节保持阻尼，用于单腿跟踪试验。 |
| `measure_cable_k.py` | 用逐步增加的绳轮力矩测量张力响应，保存到 `cable_data/K/`，用于识别绳系统参数。 |
| `measure_cable_kc.py` | 通过绳轮位置模式采集张力—位置数据，保存到 `cable_data/K_C/`。 |
| `generate_synthetic_pre_tension_data.py` | 生成确定性 50 Hz 合成 CSV，用于测试张力数据和训练流程；不能当作实机质量评估数据。 |
| `merge_pre_tension_csv.py` | 校验并合并多条张力轨迹 CSV，保留 `trajectory_id`、时间戳和力矩因果顺序。 |
| `pre_tension.py` | 检查张力 CSV，构造严格时序窗口，训练动作条件短时域张力预测模型。 |

当前普通绳驱配置 `configs/hexapod_tethered.yaml` 使用 71 维观测，并加载 `deploy/pre_train/hexapod_tethered/1015_1103/policy_9500.pt`。同目录的 `model_9500.pt` 是原始 checkpoint，`policy_9500.pt` 是转换后的部署策略。

运行数据按用途分开保存：整机绳驱脚本写入 `data_logs/<脚本名>/<脚本名_时间戳>/`，单绳脚本写入 `deploy_only_cable_data/speed/` 或 `deploy_only_cable_data/torque/`，张力预测训练轨迹集中在 `pre_data/`。这些目录中的时间戳 CSV/PNG 是实验产物，不需要逐个写入脚本索引。

## 4. 支持模块说明

### 4.1 `common/`

| 脚本 | 作用 |
| --- | --- |
| `command_helper_hexapod.py` | 为 EL4090 六足封装生成零力矩、阻尼、零速度等安全指令。 |
| `command_helper.py` | 原 Unitree SDK2 机型的命令初始化/安全指令辅助函数。 |
| `remote_controller.py` | 解析 Unitree 遥控器原始字节、按键和摇杆。 |
| `rotation_helper.py` | 四元数、重力方向和机体坐标变换工具。 |

### 4.2 `hexapod_tethered_utils/`

| 脚本 | 作用 |
| --- | --- |
| `joystick_reader.py` | 通过 Pygame 读取 Xbox 手柄，生成 `vx/vy/wz` 指令并提供 A/X/LB 等按键状态。 |
| `cable_tension_sensor.py` | 读取绳张力传感器，输出标定后的张力值。 |
| `cable_tension_sensor_zero.py` | 张力传感器清零/零点标定脚本。 |
| `cable_end_pitch_sensor.py` | 读取绳端俯仰角传感器。 |
| `cable_arm_yaw_sensor.py` | 读取绳臂/挂点航向角传感器。 |
| `cable_arm_yaw_differ_test.py` | 联合驱动绳轮并读取 yaw 传感器，用于方向差和电机方向调试。 |
| `cable_arm_yaw_differ_test_1.py` | yaw 差测试的另一个实验版本，使用前应与主版对比。 |
| `tension_speed_controller.py` | 张力误差 + 速度/绳方向前馈的绳轮转速控制器，包含 PID、滤波、死区和限幅。 |
| `tension_torque_controller.py` | 对应的绳轮力矩控制器。 |
| `learned_tension_torque_selector.py` | 加载 `pre_tension.py` 模型，维护历史窗口，批量评估候选力矩并选择指令。 |
| `moe_policy.py` | 只包含部署所需的递归 MoE actor、观测构造和 checkpoint 加载，不依赖 MuJoCo/RSL-RL。 |
| `hexapod_mjcf_kinematics.py` | 从六足 MJCF 参数构建轻量正运动学，供 Polzin/Nagatani 对照控制使用。 |
| `test_unit.py` | 将 yaw/pitch 转为机体坐标系单位绳方向的简单测试。 |

历史/试验文件：

- `cable_tension_sensor copy.py`：张力传感器旧副本。
- `tension_speed_controller copy.py` 和 `tension_speed_controller copy 2.py`：张力—转速控制器的旧参数/算法版本。
- `hexapod_tethered_utils_gkq/diff1.py`、`test.py`、`test1.py`、`test_01.py`：个人/阶段性差分和数值试验，运行前需阅读源码确认输入。

### 4.3 `motor_igh_sdk/`

| 脚本 | 作用 |
| --- | --- |
| `el4090_motor_sdk.py` | EL4090 EtherCAT/CAN 数据结构、帧打包/解包和位置、速度、力矩指令底层函数。 |
| `deploy_real_el4090_pysoem.py` | 18 个六足关节的 PySOEM 主站封装，负责 policy index↔motor ID、方向/零位转换和 PD 收发循环。 |
| `deploy_real_el4090_pysoem_spool_speed.py` | 在 18 关节封装上增加 motor 19 绳轮速度模式。 |
| `deploy_real_el4090_pysoem_spool_torque.py` | 增加 motor 19 绳轮力矩模式。 |
| `deploy_real_el4090_pysoem_spool_position.py` | 增加 motor 19 绳轮位置模式。 |
| `deploy_real_el4090_speed_pysoem.py` | 单电机速度模式测试封装。 |
| `deploy_real_el4090_torque_pysoem.py` | 单电机力矩模式测试封装。 |
| `deploy_real_el4090_position_pysoem.py` | 单电机位置模式测试封装。 |
| `rl_real_el4090_demo.py` | 不连真实硬件的映射/缓冲区演示。 |

`motor_igh_sdk_hexapod/` 是较早的六足 EL4090 封装副本：`el4090_motor_sdk.py` 是底层帧编解码，`deploy_real_el4090_pysoem.py` 是关节通信封装，`rl_real_el4090_demo.py` 是映射演示。当前纯六足主程序导入的是 `motor_igh_sdk/deploy_real_el4090_pysoem.py`。

### 4.4 IMU

| 脚本 | 作用 |
| --- | --- |
| `imu_sdk_deta40/imu_sdk.py` | 当前主程序使用的 IMU SDK；解析串口数据，提供角速度、重力、线加速度和积分线速度。 |
| `imu_sdk_deta40/test_imu.py` | 实时打印 IMU 姿态/惯性数据的命令行测试。 |
| `imu_sdk_deta40/test_vel.py` | 测试速度积分、静止检测和 ZUPT 参数。 |
| `imu_sdk_deta40/demo.py` | FDILink 原始串口示例。 |
| `imu_sdk/imu_sdk.py` | 旧版 IMU 实现，主入口已改用 `imu_sdk_deta40/`。 |
| `imu_sdk/test_imu.py` | 旧版 IMU 数据打印测试。 |
| `imu_sdk/test_vel.py` | 旧版 IMU 线速度测试。 |

## 5. 根目录脚本

| 脚本 | 作用 |
| --- | --- |
| `startup.sh` | 先对张力传感器清零，然后启动指定的绳驱部署脚本。当前含绝对路径，需与实机用户名/环境保持一致。 |
| `env.sh` | 激活 `hexapod_lgl` Conda 环境并设置 HTTP/HTTPS 代理。 |
| `setup.py` | Python 包安装信息和基础依赖。 |
| `test_pack.py` | 检查单条电机 ctypes 结构的字节对齐/大小。 |
| `test_full_pack.py` | 检查完整 EtherCAT 消息 ctypes 结构的字节大小。 |

## 6. 其他文档与自动生成代码

- `README_LEGACY.md`：从根 README 迁出的旧版命令和注释。
- `deploy/deploy_real/MOTOR_DEBUG_LOGGING_GUIDE.md`：电机 CSV 日志和曲线绘制说明。
- `deploy/deploy_real/pre_data/README.md`：张力预测数据集说明。
- `deploy/deploy_real/pre_data/plot.py`：预张力 CSV 数据的快速绘图检查脚本。
- `deploy/deploy_real/README.md` 和 `README.zh.md`：原 Unitree 实机部署文档。
- `deploy/deploy_real/SIM2REAL_file_roles.md`：`common/` 和 `unitree_sdk2py/` 的更细文件索引。
- `unitree_sdk2py/`：大量 IDL/RPC/DDS 文件为 SDK 自动生成或原工程支持代码，不建议为六足功能直接修改。

## 7. 运行前检查

- 核对 YAML 中的 `policy_path`、观测维度、动作维度、`joint2motor_idx`、`motor_directions` 和 `motor_offsets`。
- 核对网卡名、IMU 串口、张力/yaw/pitch 传感器串口以及 udev 别名。部分脚本仍使用硬编码网卡或绝对路径，命令行参数未必会覆盖它们。
- 先分别测试手柄、IMU、传感器和单电机，再启动整机策略。
- 绳轮的位置、速度和力矩测试脚本会直接动作 motor 19，运行前必须卸载或限定张力。
- 带 `copy`、`test`、人名或文献名后缀的脚本主要用于对照实验，不要在未检查配置的情况下当作默认入口。

---
