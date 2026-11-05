# Gazebo Estimate 控制

本包对照 `deploy/deploy_real/deploy_real_hexapod_estimate.py` 实现 Gazebo 适配。
直接复用真机 `HexapodEstimatePolicy`、`build_observation`、配置文件和手柄读取器，
不导入真机主脚本，不打开 EtherCAT 或 IMU 串口。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/ch/Hexapod_Sim2Real/ros2_ws
colcon build --symlink-install
./scripts/launch_estimate.sh
```

也可 `source install/setup.bash` 后使用
`ros2 launch hexapod_control estimate.launch.py`。
先关闭之前的 Gazebo；控制入口负责生成机器人、配置 PD、预热网络、解除暂停。
默认用 2 秒进入默认姿态保持，等待 A 启动策略。

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | `false` 无窗口 |
| `gamepad` | `true` | 有手柄时优先使用摇杆；无手柄可用键盘或话题 |
| `initial_mode` | `default` | `zero` 可复现真机零力矩等待 Y；仿真没有吊架，零力矩会下落 |
| `real_config` | 空 | 空值读取项目 `deploy/deploy_real/configs/hexapod_estimate.yaml` |
| `policy_path` | 空 | 空值沿用真机 YAML，非空则用指定的策略绝对路径 |
| `python_executable` | 自动 | 有 PyTorch 和 ROS Humble 的 Python 3.10 |
| `log_csv` | `true` | 保存至 `ros2_ws/log/control/` |
| `project_root` | 自动 | 当前项目根目录 |

其他状态机和增益参数位于 `config/estimate.yaml`。
真机 YAML 默认加载 `deploy/pre_train/hexapod_estimate/1020_flat/policy_24500.pt`。
支持原始 MJLab Estimate checkpoint 或带 `reset()` 的 TorchScript；
普通 66 维 Hexapod 策略不适用于该入口。

## 手柄与键盘状态机

指定的真机脚本实际使用 Y，而非旧文档中的 X。沿用同一手柄读取器及
`HEXAPOD_GAMEPAD_PRESET`、`HEXAPOD_GAMEPAD_BTN_*` 环境变量，
如果设备的 SDL 按键编号不同，可用这些环境变量覆盖。

| 动作 | 手柄 | 键盘 |
| --- | --- | --- |
| 2 秒插值回默认姿态并保持 | Y | `y` |
| 默认姿态完成后进入策略 | A | `a` |
| 停止策略并锁定阻尼 | LB | 空格 |
| 零力矩复位，允许重新 Y→A | 话题 `zero` | `z` |
| 前后速度 | 左杆纵向 | `w` / `s` |
| 左右速度 | 左杆横向 | `j` / `l` |
| 转向 | 右杆横向 | `q` / `e` |
| 速度归零，策略继续运行 | 松开摇杆 | `x` |

```bash
# 另一终端，必须是交互终端
source /opt/ros/humble/setup.bash
source /home/ch/Hexapod_Sim2Real/ros2_ws/install/setup.bash
ros2 run hexapod_control keyboard_control.py
```

键盘每次增加/减少 0.1 m/s 或 rad/s，并以 10 Hz 持续发送当前速度；
松键不自动停车。`x` 清零，空格阻尼，Ctrl+C/ESC 阻尼并退出键盘。
连接手柄时摇杆优先；键盘/话题控制请用 `gamepad:=false` 启动。
控制节点或整个 launch 退出后，C++ 电机层的命令看门狗仍负责去掉旧位置目标。

每次进入或退出策略都会清空 GRU 和上一步动作；按住 A 不会每周期重置网络。
LB 优先于其他按键。阻尼状态需要先显式 `zero` 复位，才允许再次默认姿态→策略。

## ROS 控制接口

```bash
# 完成默认姿态后进入策略
ros2 topic pub --once /hexapod/mode std_msgs/msg/String "{data: policy}"

# 速度必须持续发送；单次命令会超时归零
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --once /hexapod/mode std_msgs/msg/String "{data: default}"
ros2 topic pub --once /hexapod/mode std_msgs/msg/String "{data: damping}"
# 阻尼后复位：zero → default（等待两秒）→ policy
ros2 topic pub --once /hexapod/mode std_msgs/msg/String "{data: zero}"
```

速度采用机体坐标，`linear.x/y` 单位 m/s，`angular.z` 单位 rad/s，
按真机配置 `[0.8,0.5,0.8]` 限幅。摇杆原始值乘一次 `command_scale`；
ROS 速度已是物理单位，不会再乘第二次。速度超过 0.5 秒墙钟无更新就归零，
策略继续运行。状态输出为 `starting/zero/moving_to_default/default/policy/damping`。

| 输出话题 | 类型 | 用途 |
| --- | --- | --- |
| `/hexapod/state` | `std_msgs/String` | 当前状态，支持后加入订阅者 |
| `/hexapod/observation` | `Float64MultiArray` | 策略运行时的 63 维输入 |
| `/hexapod/policy_action` | `Float64MultiArray` | 策略运行时的 18 维裁剪后动作 |
| `/hexapod_pd_controller/state` | `sensor_msgs/JointState` | 100 Hz 的 q/dq 和实际限幅力矩 |

`/hexapod_pd_controller/commands` 是内部 5×18 行优先数组，依次为
`q_target/dq_target/kp/kd/tau_ff`。正常使用只操作状态和速度接口。
不要与原始 `effort_controller` 同时占用同一组关节。

## 观测与控制周期

| 下标 | 观测 |
| --- | --- |
| 0–2 | 机体角速度 × `ang_vel_scale` |
| 3–5 | IMU 四元数计算的机体系单位重力 |
| 6–23 | `(q-default_angles)*dof_pos_scale` |
| 24–41 | `dq*dof_vel_scale` |
| 42–59 | 上一步经过 ±10 裁剪的原始动作 |
| 60–62 | 物理单位速度指令 |

GRU 每 20 ms 使用前 60 维；完整 63 维观测送入 Actor。
机体线速度由网络内部估计，未使用 Gazebo 真值或 IMU 积分线速度。
角速度来自 `trunk` IMU，单位重力为 `R_body_to_world.T @ [0,0,-1]`，
不是含运动加速度的 accelerometer 归一化值。

关节反馈按名字重排为 `RF/RM/RB/LF/LM/LB`，每腿 `HAA/HFE/KFE`。
Gazebo 已使用模型坐标，不能再应用实机 `motor_directions/motor_offsets`。
目标位置为 `default_angles + clip(action,-10,10)*0.25`，不额外裁剪目标位置。

默认姿态增益 `150/11`、策略增益 `100/3`、阻尼增益 `0/11` 与真机一致。
策略节点以 50 Hz 更新目标，C++ PD 插件以 1000 Hz 从硬件接口读取反馈：

```text
tau = clip(kp*(q_target-q) + kd*(dq_target-dq) + tau_ff, -30, 30)
```

目标速度和前馈力矩默认均为零。PD 环不依赖 `/joint_states` 消息频率。
实时缓冲区交接命令，超出 0.25 秒仿真时间无更新时，独立切入 `-11*dq` 阻尼。
实现参考 [ROS 2 Humble 控制器文档](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)。

策略使用仿真时钟，暂停时不推进 GRU；Gazebo `/clock` 已从默认 10 Hz 改为
1000 Hz，避免同一时刻重复推理。传感器过期、无效数据、推理异常或手柄断开
会进入锁定阻尼。

**重置场景：请关闭并重新启动本 launch。** 本机 `gazebo_ros2_control 0.4.10`
未实现清除内部更新时间的 Reset，直接 `/reset_simulation` 或 Gazebo 的 Reset Time
会让它停止更新，直到时间追上重置前的数值。控制节点检测到时间倒退时清空 GRU
与命令、锁定停止并暂停场景，日志明确要求重启；此时不能靠 Y/A 恢复。
正常暂停/继续不受这个限制影响。

## Python 环境

本机已验证解释器为 `/home/ch/El4090_sim2real-master/.venv/bin/python`，
Python 3.10，支持 ROS Humble 和 PyTorch。launch 优先本项目 `.venv/bin/python`，
其次该已有环境，最后尝试系统解释器。策略显式在 CPU 上运行，不需要 CUDA 或 MJLab。
若另建环境，使用 Python 3.10 并保留系统 ROS 包：

```bash
cd /home/ch/Hexapod_Sim2Real
python3 -m venv --system-site-packages .venv
./.venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
./.venv/bin/python -m pip install 'numpy>=1.24,<2' PyYAML pygame
```

可传 `python_executable:=/绝对路径/python`。
`No module named torch` 表示当前解释器缺少 PyTorch；
`_rclpy_pybind11` 不匹配应换回 Python 3.10，不能用 Python 3.12 的环境运行 Humble。

## 文件、日志与测试

- `hexapod_control/core.py`：与 ROS 无关的状态机、重力转换、观测、动作映射。
- `hexapod_control/node.py`：反馈、启动握手、手柄/ROS 输入、仿真定时器和 CSV。
- `src/pd_controller.cpp`：实时 PD、接口重排、力矩限幅、看门狗与遥测。
- `config/estimate.yaml`：仿真状态机增益和超时；`config/controllers.yaml`：电机环配置。

CSV 在 `ros2_ws/log/control/estimate_时间.csv`，每周期记录仿真时间、状态、
推理次数、命令、18 关节 q/dq/目标、动作和完整观测，列名标明关节顺序。
原有真机 `motor_logs/` 和配置均未修改。

```bash
cd /home/ch/Hexapod_Sim2Real
# 包括真实策略多步对照、GRU 清除、重力旋转和状态转换
/home/ch/El4090_sim2real-master/.venv/bin/python \
  ros2_ws/src/hexapod_control/test/test_core.py

# 终端一：专用测试场景
cd ros2_ws
./scripts/launch_estimate.sh gui:=false gamepad:=false
# 终端二：场景进入 default 后执行；会实际行走，最后进入阻尼
python3 scripts/check_estimate_control.py
```

已完成的 8 秒前进测试：指令 0.2 m/s，实际前进约 0.61 m，
策略约 49.9 Hz，最低机体高度约 0.294 m；命令超时、暂停和状态切换通过。
另已验证 Python 策略进程暂停后，C++ 看门狗独立输出限幅后的 `-11*dq`；
键盘连续按键和退出已实测。本机未检测到手柄，手柄分支复用真机读取器，
没有声称已完成实体手柄测试。
当前速度跟踪仍有偏差。模型的 ODE 接触、电机 armature 与训练环境存在差异，
详见 [模型转换说明](../hexapod_description/SOURCE.md)，未声称仿真动力学与真机完全一致。
