# 模型来源与转换

当前模型来自本仓库用户提供的 `hexapod/xmls/hexapod.xml`、
`hexapod/xmls/assets/*.STL` 和 `hexapod/hexapod_constants.py`。
已替换初次导入时临时使用的相邻工程 EL4090 描述和网格。
来源文件摘要、MuJoCo 编译器版本和模型参数见 `model_info.json`。

主要参数：总质量 39.9407774168 kg，18 个腿部关节，25 个网格；
HAA 限位 ±0.8 rad、HFE/KFE 限位 ±0.6 rad，速度上限 16.44 rad/s，
力矩上限 30 Nm；关节初始值为 0，根位置为 `(0, 0, 0.313)`。
执行器限值和初始位置读取自提供的 MJLab 常量文件，不导入 MJLab。

`ros2_ws/scripts/import_mjcf.py` 负责生成 `urdf/hexapod.urdf.xacro` 和复制网格。
它使用 MuJoCo 编译后参数解析 MJCF 继承、四元数、关节轴和胶囊端点，
采用实际编译后的质量、质心、主惯量及惯性坐标系。
原始 MJCF 使用 `balanceinertia="true"`，部分大腿惯量会在编译时自动修正，
因此不能仅复制原始 `fullinertia` 数字。
参见 [MuJoCo 编译与惯性定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler)。

网格保持字节不变；视觉位姿读取原始 MJCF，避免重复应用 MuJoCo 内部的
网格重心/主轴变换。胶囊碰撞体转为同位置圆柱和两端球，保持几何外形；
足端球放在已有的固定 FOOT 连杆坐标系中，保持世界位姿，方便独立设置摩擦。
机体坐标系为 `trunk`，`base_link` 是与机体重合的无质量 ROS 根坐标系。
IMU 位于 `trunk` 原点，与源模型 `imu` site 一致。

Gazebo 适配边界：采用 ODE 接触参数，足端摩擦系数为 1，其他碰撞体摩擦为 0，
对应 MJLab `FULL_COLLISION` 的足端摩擦和非足端仅法向接触意图；
自碰撞关闭。胶囊拆分的接触求解、滚动/扭转摩擦、接触软化和自碰撞行为
与 MuJoCo 不完全等价。MJLab 设置的电机 armature（0.0486 kg·m²）
没有直接等价的标准 URDF 参数，本次没有把它加到连杆惯量中。
PD 增益（100/3）由 `hexapod_control` 的电机控制器实现，
没有作为被动关节阻尼加入模型。已完成策略行走验证，
跨仿真器的速度跟踪与动力学差异仍需继续校准。

这是纯六足版本，没有绳臂、绳索和张力执行器。原始 `hexapod/` 文件保持不变。
本包保留初次适配的 BSD 许可；提供的模型目录未附独立许可证。
