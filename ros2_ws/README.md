# 六足机器人 ROS 2 / Gazebo 工作空间

适用于本机 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。
已完成模型导入和纯六足 Estimate 策略控制。模型查看入口默认暂停；
控制入口会配置 PD、预热策略、解除暂停，并进入默认姿态保持。

当前使用你提供的 `../hexapod/` 模型（MJCF + MJLab 常量 + 25 个 STL），
已替换临时 EL4090 模型。总质量约 39.94 kg，初始高度为 0.313 m，
18 个关节的轴向、限位和 30 Nm 力矩上限已按新模型更新。

已验证：三个包成功构建，模型和 18 路控制接口正常。
发送 0.2 m/s 前进指令的 8 秒仿真测试中，实际前进约 0.61 m，
策略频率约 49.9 Hz，机体最低高度约 0.294 m；暂停、速度超时和状态切换检查通过。
当前速度跟踪仍有偏差，尚未标定为精确速度跟踪。

## 启动 Estimate 控制

先关闭已有 Gazebo 实例。终端一：

```bash
source /opt/ros/humble/setup.bash
cd /home/ch/Hexapod_Sim2Real/ros2_ws
colcon build --symlink-install
./scripts/launch_estimate.sh
```

等到 `READY`，再等待约 2 秒进入 `default`。终端二：

```bash
source /opt/ros/humble/setup.bash
source /home/ch/Hexapod_Sim2Real/ros2_ws/install/setup.bash
ros2 run hexapod_control keyboard_control.py
```

按 `a` 启动策略，再按 `w` 增加前进速度；`x` 速度归零，空格进入阻尼。
键盘每次加减 0.1，当前速度会持续发送，松键不自动停车。
手柄与指定真机脚本一致：**Y** 回默认姿态、A 启动策略、LB 阻尼，摇杆控制速度。
无手柄可用键盘；连接手柄时摇杆优先，键盘/话题控制可用 `gamepad:=false` 禁用手柄。
阻尼状态锁定后，先按 `z` 零力矩复位，再 `y` 回默认，最后 `a` 恢复策略。

标准入口：`ros2 launch hexapod_control estimate.launch.py`。
`gui:=false` 无窗口；`initial_mode:=zero` 完整复现真机零力矩等待 Y 的启动流程。
仿真没有真机吊架，因此默认自动站立，**不会自动启动行走策略**。

完整的按键、ROS 话题、Python 环境、观测合同、配置、日志和验证说明见
[Estimate 控制 README](src/hexapod_control/README.md)。

## 构建与打开

```bash
source /opt/ros/humble/setup.bash
cd /home/ch/Hexapod_Sim2Real/ros2_ws
colcon build --symlink-install
./scripts/launch_gazebo.sh
```

也可以使用 ROS 2 标准入口：

```bash
source /opt/ros/humble/setup.bash
source /home/ch/Hexapod_Sim2Real/ros2_ws/install/setup.bash
ros2 launch hexapod_gazebo gazebo.launch.py
```

参数：`gui:=false` 无窗口运行；`paused:=false` 启动物理仿真；
`x:=0.0 y:=0.0 z:=0.313` 设置出生位置；`world:=/绝对路径/场景.world` 替换场景。
此入口仅查看模型，不加载站立控制；点击播放后模型会受重力下落、关节自由运动。
需要行走时使用上面的 `launch_estimate.sh` 控制入口。

## RoboSense Airy 三维激光雷达

模型默认带一个 Airy 雷达，配置位于
[`airy_lidar.xacro`](src/hexapod_description/urdf/airy_lidar.xacro)。
位置按**机身前沿再向前约 25 cm**设置：机身 STL 前沿约为 `x=0.5025 m`，
雷达底座中心相对 `trunk/base_link` 为 `(0.75, 0, 0) m`。
高度暂取与机体原点等高；站立初始状态约距地面 0.313 m。
这是固定安装关系，暂未建模实体安装支架及其质量。

安装关节为 `airy_mount_joint`，`rpy=(0, pi/2, 0)`：

| 雷达坐标轴 | 对应机体方向 |
| --- | --- |
| `+X` | 下方（机体 `-Z`） |
| `+Y` | 左方（机体 `+Y`） |
| `+Z` | 正前方（机体 `+X`） |

按 [RoboSense Airy 官方手册](https://robosense-robotics.github.io/product-manual/en/Airy/)
的 96 线版本设置：本地水平方位 360°、仰角 0～90°、10 Hz、量程 0.1～60 m，
本体直径约 60 mm、高 63 mm、质量 0.23 kg。
该安装姿态使扫描半球**朝向机器人前方**，不是围绕机器人水平扫描一整圈。
外观为圆柱和球面近似，惯量为均匀圆柱估算；不是官方 CAD 或实测惯量。
添加雷达后总质量约为 40.17 kg，固定支架质量未计入。

Gazebo 使用 CPU `ray` 和 `libgazebo_ros_ray_sensor.so` 发布
`/airy/points`（`sensor_msgs/msg/PointCloud2`）。点云坐标系为 `airy_scan`，
相对底座坐标系 `airy_link` 沿本地 Z 偏移 0.033 m，轴向一致。
这个偏移是简化扫描中心，**不是实机标定外参**。
点云是理想均匀采样模型，包含 XYZ/intensity，不提供实机的逐点时间、ring、
UDP 数据包、真实反射率、帧缺口和多回波；未添加 Airy 内置 IMU。
现有策略仍使用机体 `/imu/data`。

重新构建并重启 Gazebo 后生效：

```bash
source /opt/ros/humble/setup.bash
cd /home/ch/Hexapod_Sim2Real/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch hexapod_control estimate.launch.py
```

查看接口与安装 TF（仿真播放时才持续发布点云）：

```bash
ros2 topic info /airy/points
ros2 topic hz /airy/points
ros2 run tf2_ros tf2_echo base_link airy_scan
```

RViz 中将 Fixed Frame 设为 `base_link`，添加 `PointCloud2`，
选择 `/airy/points`，Reliability 可设为 Best Effort。
默认每帧采样 `900 × 96` 条射线，CPU 负载较高时可降低采样数：

```bash
ros2 launch hexapod_control estimate.launch.py airy_horizontal_samples:=360 airy_vertical_samples:=32
```

导入入口 `hexapod_gazebo gazebo.launch.py` 支持相同参数。
两个入口均可传 `use_airy:=false` 恢复不带雷达的基础模型。
雷达配置独立于 MJCF 生成文件，重新运行导入脚本后会继续保留。

## 文件与数据接口

- `src/hexapod_description/`：完整的 25 个 STL、Xacro、质量和惯性、碰撞体。
- `src/hexapod_gazebo/`：启动文件、离线可用的地面与光照、控制器配置。
- `/joint_states`：18 个关节的位置/速度；顺序为
  `RF/RM/RB/LF/LM/LB`，每腿 `HAA/HFE/KFE`。
- `/imu/data`：200 Hz 机体 IMU，坐标系 `trunk`，与源 MJCF 的 `imu` site 重合。
- `/airy/points`：10 Hz Airy 简化三维点云，坐标系 `airy_scan`。
- `/gazebo/model_states`、`/gazebo/link_states`：模型和连杆世界位姿。
- `/robot_description`、`/tf`、`/tf_static`、`/clock`：描述、坐标变换和仿真时间。
- 控制入口另外发布 `/hexapod/state`、`/hexapod/policy_action`、`/hexapod/observation`。
- `/hexapod_pd_controller/state`：100 Hz 电机遥测，`effort` 为限幅后实际下发的 Nm。

暂停时周期数据停止更新。默认出生高度为 0.313 m，对应提供的 MJLab 初始配置；
足端球半径为 0.023 m，零位姿态下足底约在地面上方 0.3 mm。

## 验证导入

Gazebo 已按默认参数启动、机器人仍在初始位置时，在另一终端运行：

```bash
source /opt/ros/humble/setup.bash
python3 /home/ch/Hexapod_Sim2Real/ros2_ws/scripts/check_gazebo.py
```

检查脚本短暂播放物理仿真，验证实体、18 个关节顺序和 IMU 有效数据，
最后重新暂停并重置仿真与机器人姿态。此操作会重置当前场景全部模型。
如果启动时设置了 `use_ros2_control:=true`，可在检查命令后加
`--check-control`，同时验证 18 路 effort 接口。

## 仅导入模式的原始力矩接口

在**仅模型导入**入口增加 `use_ros2_control:=true` 可加载 Gazebo effort 硬件接口。
此选项只提供接口，不会自动运行策略或使机器人站立。
需要使用力矩控制时，先播放仿真，再加载控制器：

```bash
source /opt/ros/humble/setup.bash
source /home/ch/Hexapod_Sim2Real/ros2_ws/install/setup.bash
ros2 run controller_manager spawner effort_controller
```

`/effort_controller/commands` 使用 `std_msgs/msg/Float64MultiArray`，单位 Nm，
顺序见 `src/hexapod_gazebo/config/controllers.yaml`。关节状态已有 Gazebo 插件发布，
无需同时启用 `joint_state_broadcaster`。
Estimate 控制入口使用 `hexapod_pd_controller`，不要同时启动原始
`effort_controller`，两者会争用相同的 18 个力矩接口。

## 修改源模型后重新生成

Gazebo 启动只依赖已生成的描述，无需安装 MuJoCo 或 MJLab。
修改 `hexapod/` 中的模型后，用有 `mujoco` 和 `numpy` 的 Python 环境重新转换：

```bash
cd /home/ch/Hexapod_Sim2Real
/home/ch/mujoco_ws/mjlab/.venv/bin/python ros2_ws/scripts/import_mjcf.py
xacro ros2_ws/src/hexapod_description/urdf/hexapod.urdf.xacro use_airy:=false > /tmp/hexapod.urdf
/home/ch/mujoco_ws/mjlab/.venv/bin/python ros2_ws/scripts/check_model_conversion.py /tmp/hexapod.urdf
cd ros2_ws
colcon build --symlink-install
```

转换检查临时关闭附加雷达，只比较源 MJCF 的机器人本体。
它会对比 MuJoCo 和 URDF 在零位及 20 组非零关节位姿下的连杆、
视觉网格与碰撞体，核对各刚体质量/惯性、关节方向/限位和网格哈希。
初始位置、初始关节值或关节集合变化后，还应同步更新启动参数和导入检查脚本。

该模型是纯六足版本，当前控制入口已选择对应的纯六足 Estimate 策略。
ODE 接触与电机转子惯量等跨仿真器差异见
`src/hexapod_description/SOURCE.md`；不能直接沿用实机电机方向或零位补偿。
