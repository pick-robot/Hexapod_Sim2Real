import numpy as np
import time
import torch
import csv
import os
from datetime import datetime

from imu_sdk_deta40.imu_sdk import IMUSDK

from common.command_helper_hexapod import create_zero_cmd,create_damping_cmd,create_zero_torque_cmd

from hexapod_tethered_utils.tension_torque_controller import TensionTorqueController,TensionTorqueControllerConfig

from motor_igh_sdk.deploy_real_el4090_pysoem_spool_torque import RL_Real_PySOEM_WithSpoolTorque


from hexapod_tethered_utils.joystick_reader import Gamepad

from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.cable_end_pitch_sensor import CableEndPitchSensor
from hexapod_tethered_utils.cable_arm_yaw_sensor import CableArmYawSensor
from hexapod_tethered_estimate_utils.observation import (
    tether_direction_from_yaw_pitch,
)


import config_hexapod_tethered

from config_hexapod_tethered import Config


class TensionTrainingCsvLogger:
    """Write the complete deployment state into one CSV row per control step."""

    def __init__(self, directory: str, trajectory_id: str, control_dt: float,
                 flush_every_n: int = 50) -> None:
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, f"{trajectory_id}.csv")
        self.trajectory_id = trajectory_id
        self.control_dt = float(control_dt)
        self.flush_every_n = max(int(flush_every_n), 1)
        self._rows = 0
        self._file = open(self.path, "w", newline="", buffering=1024 * 1024)
        sudo_uid = os.environ.get("SUDO_UID")
        sudo_gid = os.environ.get("SUDO_GID")
        if sudo_uid is not None and sudo_gid is not None:
            try:
                os.chown(directory, int(sudo_uid), int(sudo_gid))
                os.chown(self.path, int(sudo_uid), int(sudo_gid))
            except (OSError, ValueError) as exc:
                print(f"[PreTensionCSV][WARN] cannot restore log ownership: {exc}")
        self._fieldnames = self._make_fieldnames()
        self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames)
        self._writer.writeheader()
        print(f"[PreTensionCSV] logging enabled: {self.path}")

    @staticmethod
    def _names(prefix: str, size: int):
        return [f"{prefix}_{i}" for i in range(size)]

    @classmethod
    def _make_fieldnames(cls):
        scalar = [
            "timestamp_ns", "trajectory_id", "sample_index", "control_dt_s",
            "force_raw_n", "torque_actual_nm",
            "torque_command_prev_nm", "torque_command_issued_nm",
            "motor_position_rad", "force_reference_n",
            "imu_valid", "tension_sensor_valid",
            "saturation_flag", "emergency_flag", "data_valid_flag",
        ]
        vectors = []
        for name, size in (
            ("velocity_command", 3), ("policy_action", 18),
            ("joint_position", 18), ("joint_velocity", 18),
            ("body_gravity_vector", 3), ("body_angular_velocity", 3),
            ("body_linear_velocity", 3),
            ("body_linear_acceleration", 3),
        ):
            vectors.extend(cls._names(name, size))
        return scalar + vectors

    @staticmethod
    def _put_vector(row, prefix: str, value, size: int) -> None:
        array = np.asarray(value, dtype=np.float64).reshape(-1)
        if array.size != size:
            raise ValueError(f"{prefix} must contain {size} values, got {array.size}")
        for i, item in enumerate(array):
            row[f"{prefix}_{i}"] = float(item)

    def write(self, *, timestamp_ns: int, force_raw_n: float,
              torque_actual_nm: float, torque_command_prev_nm: float,
              torque_command_issued_nm: float, motor_position_rad: float,
              force_reference_n: float, velocity_command, policy_action,
              joint_position, joint_velocity, body_gravity_vector,
              body_angular_velocity, body_linear_velocity,
              body_linear_acceleration,
              imu_valid: bool, tension_sensor_valid: bool,
              saturation_flag: bool, emergency_flag: bool = False) -> None:
        row = {
            "timestamp_ns": int(timestamp_ns),
            "trajectory_id": self.trajectory_id,
            "sample_index": self._rows,
            "control_dt_s": self.control_dt,
            "force_raw_n": float(force_raw_n),
            "torque_actual_nm": float(torque_actual_nm),
            "torque_command_prev_nm": float(torque_command_prev_nm),
            "torque_command_issued_nm": float(torque_command_issued_nm),
            "motor_position_rad": float(motor_position_rad),
            "force_reference_n": float(force_reference_n),
            "imu_valid": int(imu_valid),
            "tension_sensor_valid": int(tension_sensor_valid),
            "saturation_flag": int(saturation_flag),
            "emergency_flag": int(emergency_flag),
            "data_valid_flag": int(imu_valid and tension_sensor_valid),
        }
        self._put_vector(row, "velocity_command", velocity_command, 3)
        self._put_vector(row, "policy_action", policy_action, 18)
        self._put_vector(row, "joint_position", joint_position, 18)
        self._put_vector(row, "joint_velocity", joint_velocity, 18)
        self._put_vector(row, "body_gravity_vector", body_gravity_vector, 3)
        self._put_vector(row, "body_angular_velocity", body_angular_velocity, 3)
        self._put_vector(row, "body_linear_velocity", body_linear_velocity, 3)
        self._put_vector(
            row, "body_linear_acceleration", body_linear_acceleration, 3
        )
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % self.flush_every_n == 0:
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None


class Controller:
    def __init__(self,config:Config) -> None:
        self.config = config
        #self.remote_controller = RemoteController()

        # 手柄（pygame）用于生成 cmd；遥控器仍用于按钮状态机（start/A/select）
        self.gamepad = None
        if Gamepad is not None:
            try:
                self.gamepad = Gamepad()
            except Exception as e:
                self.gamepad = None
                print(f"[gamepad] disabled due to error: {e}")

        self.ang_vel_scale = config.ang_vel
        self.dof_pos_scale = config.dof_pos
        self.dof_vel_scale = config.dof_vel
        self.lin_vel_scale = config.lin_vel

        # Policy-side per-joint gains (indexed by policy_idx 0..17)
        self._kp_policy, self._kd_policy = self._build_policy_joint_gains()

        self.policy = torch.jit.load(config.policy_path)
        # 预热网络，减少第一次推理的延迟
        self._warm_up()
        
        # 初始化状态和命令
        self.qj = np.zeros(config.num_leggeds_actions,dtype=np.float32)
        self.dqj = np.zeros(config.num_leggeds_actions,dtype=np.float32)
        self.spool_q = np.zeros(1, dtype=np.float32)
        self.action = np.zeros(config.num_actions,dtype=np.float32)
        self.target_dof_pos = config.default_angles.copy()
        self.obs = np.zeros(config.num_obs,dtype=np.float32)
        self.cmd = np.array([0, 0, 0],dtype=np.float32)
        self.counter = 0

        # Keep each script and each process invocation in its own log directory.
        self.script_name = os.path.splitext(os.path.basename(__file__))[0]
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.join(
            script_dir,
            "data_logs",
            self.script_name,
            f"{self.script_name}_{timestamp}",
        )
        os.makedirs(self.log_dir, exist_ok=True)
        self.trajectory_id = f"{self.script_name}_trajectory_{timestamp}"
        self.pre_tension_logger = TensionTrainingCsvLogger(
            directory=self.log_dir,
            trajectory_id=self.trajectory_id,
            control_dt=float(self.config.control_dt),
            flush_every_n=50,
        )
        self._previous_spool_torque_command_nm = 0.0

        # 电机初始化 EtherCAT（单 Master）：18 关节 PD + 额外 spool 速度电机 motor_id=19 (slave_idx=3, passage=1)
        # 这里的 'enp86s0' 就是网口名；如果你要换网口，改成你的实际 NIC。
        # Spool (cable) motor is fixed in this project:
        # motor_id=19 on slave_idx=3 passage=1, direction=+1.
        self.robot = RL_Real_PySOEM_WithSpoolTorque('enp86s0')
        self.robot_start = self.robot.start()
        if not self.robot_start:
            print("[WARNING] Robot start failed. Will use zero data.")

        # Tension -> spool speed controller (pure numeric inputs, no sensor IO inside)
        self.tension_torque_controller = TensionTorqueController(
            TensionTorqueControllerConfig(
			speed_sign=float(self.config.tsc_speed_sign),
			k_p_forward_rpm_per_unit=float(self.config.tsc_k_p_forward_rpm_per_unit_torque),
			k_p_backward_rpm_per_unit=float(self.config.tsc_k_p_backward_rpm_per_unit_torque),
			k_d_forward_rpm_per_unit=float(self.config.tsc_k_d_forward_rpm_per_unit_torque),
			k_d_backward_rpm_per_unit=float(self.config.tsc_k_d_backward_rpm_per_unit_torque),
			k_i_forward_rpm_per_unit=float(self.config.tsc_k_i_forward_rpm_per_unit_torque),
			k_i_backward_rpm_per_unit=float(self.config.tsc_k_i_backward_rpm_per_unit_torque),
			ff_enabled=bool(self.config.tsc_ff_enabled),
			ff_radius_m=float(self.config.tsc_ff_radius_m),
			tension_deadband=float(self.config.tsc_tension_deadband),
			tension_lpf_alpha=float(self.config.tsc_tension_lpf_alpha),
            Kff_forward=float(self.config.tsc_Kff_forward_torque),
            Kff_backward=float(self.config.tsc_Kff_backward_torque),
			torque_limit=float(self.config.tsc_torque_limit),
			torque_deadband=float(self.config.tsc_torque_deadband),
            )
        )

        # imu init
        self.imu = IMUSDK(port='/dev/ttyUSB_imu', baudrate=921600)
        self.imu_started = self.imu.start()
        if not self.imu_started:
            print("[WARNING] IMU start failed. Will use zero data.")

        # 六位力传感器初始化 RS232
        self.tension_sensor = CableTensionSensor(port='/dev/ttyUSB_cable_tension', baudrate=115200)
        self.tension_started = self.tension_sensor.start()
        if not self.tension_started:
            print("[WARNING] Tension sensor start failed. Will use zero data.")
        
        # 末端绝对角度传感器初始化 RS485
        self.pitch_sensor = CableEndPitchSensor(port='/dev/ttyUSB_pitch', baudrate=115200)
        self.pitch_started = self.pitch_sensor.start()
        if not self.pitch_started:
            print("[WARNING] Pitch sensor start failed. Will use zero data.")

        # 磁栅编码器初始化 RS485
        self.yaw_sensor = CableArmYawSensor(port='/dev/ttyUSB_yaw', baudrate=115200)
        self.yaw_started = self.yaw_sensor.start()
        if not self.yaw_started:
            print("[WARNING] Yaw sensor start failed. Will use zero data.")

    def __del__(self):
        csv_logger = getattr(self, "pre_tension_logger", None)
        if csv_logger is not None:
            try:
                csv_logger.close()
            except Exception:
                pass
        log_dir = getattr(self, "log_dir", None)
        if log_dir is not None:
            print(f'[Logging] CSV file saved to {log_dir}')

    def _build_policy_joint_gains(self):
        """Build kp/kd arrays aligned with policy joint indices.

        User-defined groups are specified in motor_id space, but the runtime
        buffers are indexed by policy_idx (0..17). We map via
        config.joint2motor_idx.
        """

        group1 = {13, 7, 1, 16, 10, 4}
        group2 = {18, 9, 3, 15, 11, 6}
        group3 = {17, 8, 2, 14, 12, 5}

        kp = np.zeros(18, dtype=np.float32)
        kd = np.zeros(18, dtype=np.float32)

        unknown_motor_ids = []
        for policy_idx in range(18):
            motor_id = int(self.config.joint2motor_idx[policy_idx])
            if motor_id in group1:
                kp[policy_idx] = 100.0
                kd[policy_idx] = 3.0
            elif motor_id in group2:
                kp[policy_idx] = 100.0
                kd[policy_idx] = 3.0
            elif motor_id in group3:
                kp[policy_idx] = 100.0
                kd[policy_idx] = 3.0
            else:
                unknown_motor_ids.append(motor_id)

        if unknown_motor_ids:
            # 不阻塞运行，但提示配置/分组可能不完整
            uniq = sorted(set(unknown_motor_ids))
            print(f"[WARNING] Some motor_ids are not in any gain group: {uniq}")

        return kp, kd

    def _warm_up(self):
        obs = torch.ones((1, int(self.config.num_obs)))
        for _ in range(10):
            _ = self.policy(obs)
        print('Network has been warmed up.')


    # 零力矩模式
    def zero_torque_state(self):
        print("Enter zero torque state.")
        print("Waiting for the Y signal to default pos...")
        if self.gamepad is None:
            raise RuntimeError("Gamepad is not available; cannot enter zero_torque_state")

        while self.gamepad.get_button_y() != 1:
            create_zero_cmd(self.robot)
            # 零速度
            create_zero_torque_cmd(self.robot)
            time.sleep(self.config.control_dt)

    # 移动到默认位置
    def move_to_default_pos(self):
        print('Moving to default pos.')
        # Safety: ensure spool speed motor (id19) is stopped while repositioning joints.
        create_zero_torque_cmd(self.robot)
        total_time = 2
        num_step = int(total_time / self.config.control_dt)
        default_pos = self.config.default_angles


        init_dof_pos = np.zeros(18,dtype=np.float32)
        # 读取当前 DOF 位置作为起点
        # NOTE: RL_Real_PySOEM buffers are indexed by policy_idx (0..17).
        # SDK internally maps policy_idx -> motor_id and applies direction/offset.
        for i in range(18):
            init_dof_pos[i] = self.robot.motor_state_buffer.position[i]

        for i in range(num_step):
            if self.gamepad.get_button_lb() == 1:
                return False

            alpha = i / num_step
            
            for j in range(18):
                target_pos = default_pos[j]
                q = init_dof_pos[j] * (1 - alpha) + target_pos * alpha
                self.robot.motor_command_buffer.kp[j] = 150.0
                self.robot.motor_command_buffer.kd[j] = 11.0
                self.robot.motor_command_buffer.target_position[j] = q
                self.robot.motor_command_buffer.target_velocity[j] = 0.0
                self.robot.motor_command_buffer.feedforward_torque[j] = 0.0

            # Keep stopping spool continuously (avoid stale speed cmd).
            create_zero_torque_cmd(self.robot)

            time.sleep(self.config.control_dt)

        return True

    # 在默认位置等待，直到按下按钮A，或按下LB进入阻尼
    def default_pos_state(self):
        print("Enter default pos state.")
        print("Waiting for Button A to start policy, or LB for damping...")

        if self.gamepad is None:
            raise RuntimeError("Gamepad is not available; cannot enter default_pos_state")

        #init_dof_pos = np.zeros(18, dtype=np.float32)
        while True:
            if self.gamepad.get_button_lb() == 1:
                return False
            if self.gamepad.get_button_a() == 1:
                return True

            for i in range(18):
                default_pos = self.config.default_angles[i]
                self.robot.motor_command_buffer.kp[i] = 150.0
                self.robot.motor_command_buffer.kd[i] = 11.0
                self.robot.motor_command_buffer.target_position[i] = default_pos
                self.robot.motor_command_buffer.target_velocity[i] = 0.0
                self.robot.motor_command_buffer.feedforward_torque[i] = 0.0
                # init_dof_pos[i] = self.robot.motor_state_buffer.position[i]
                # print(init_dof_pos[i])

            # Safety: keep spool speed at 0 while holding default pose.
            create_zero_torque_cmd(self.robot)
            time.sleep(self.config.control_dt)

    # 打印关节初始位置
    def print_initial_joint_positions(self):
        print("Initial joint positions:")
        for i in range(18):
            motor_id = self.config.joint2motor_idx[i]
            pos = self.robot.motor_state_buffer.position[i]
            print(f"policy_idx {i:02d} (motor_id {motor_id:02d}), Position: {pos:.3f}")

    # 打印imu数据
    def print_imu_data(self):
        for _ in range(10):
            vel = self.imu.get_linear_velocity()
            grav = self.imu.get_gravity_acceleration()
            print(
                f"Vel: [{vel[0]:+7.3f}, {vel[1]:+7.3f}, {vel[2]:+7.3f}] | "
                f"Grav: [{grav[0]:+7.3f}, {grav[1]:+7.3f}, {grav[2]:+7.3f}]",
                flush=True,
            )
            time.sleep(0.1)

    def print_pitch_angle(self):
        pitch_value = self.pitch_sensor.get_angle()
        if pitch_value is not None:
            print(f"Pitch Angle: {pitch_value:.3f} rad")
        else:
            print("No pitch angle data available.")

    def print_yaw_angle(self):
        yaw_value = self.yaw_sensor.get_angle()
        tether_yaw = self.yaw_sensor.get_body_fixed_yaw(
            yaw_value=yaw_value,
            offset_rad=self.config.offset_deg,
        )
        if yaw_value is not None and tether_yaw is not None:
            print(f"Yaw Sensor: {float(yaw_value):.3f} deg")
            print(f"Policy Tether Yaw: {tether_yaw:.3f} rad")
        else:
            print("No yaw angle data available.")

    def run(self):
        if self.gamepad is None:
            raise RuntimeError("Gamepad is not available; cannot run control loop")

        observation_timestamp_ns = time.monotonic_ns()
        self.counter += 1
        for i in range(18):
            self.qj[i] = self.robot.motor_state_buffer.position[i]
            self.dqj[i] = self.robot.motor_state_buffer.velocity[i]

        self.spool_q[0] = self.robot.spool_state_buffer.position[0]
        spool_torque_actual_pre_action = float(self.robot.spool_state_buffer.torque[0])
        vel = self.imu.get_linear_velocity()
        imu_data = self.imu.get_imu_data()
        grav = self.imu.get_gravity_acceleration()

        tension_value = self.tension_sensor.get_cable_tension()
        tension_sensor_valid = tension_value is not None and np.isfinite(tension_value)
        if not tension_sensor_valid:
            tension_value = 0.0
            
        pitch_radians = self.pitch_sensor.get_angle()
        if pitch_radians is None or not np.isfinite(pitch_radians):
            pitch_radians = 0.0
            
        yaw_value = self.yaw_sensor.get_angle()
        tether_yaw = self.yaw_sensor.get_body_fixed_yaw(
            yaw_value=yaw_value,
            offset_rad=self.config.offset_deg,
        )
        if tether_yaw is None or not np.isfinite(tether_yaw):
            tether_yaw = 0.0
        tether_direction_body = tether_direction_from_yaw_pitch(
            float(0.0), float(0.0)
        )
        # tether_direction_body = tether_direction_from_yaw_pitch(
        #     float(tether_yaw), float(pitch_radians)
        # )
        if imu_data is None or vel is None or grav is None:
            # 如果没有 IMU 数据，使用全 0
            linvel = np.zeros(3, dtype=np.float32)
            ang_vel = np.zeros(3, dtype=np.float32)
            gravity_orientation = np.array([0.0, 0.0, -1.0], dtype=np.float32)  # 假设重力向下
            body_linear_acceleration = np.zeros(3, dtype=np.float32)
            imu_valid = False
        else:
            linvel = np.asarray(vel, dtype=np.float32)
            ang_vel = np.asarray(
                [imu_data['gyro_x'], imu_data['gyro_y'], imu_data['gyro_z']], dtype=np.float32
            )
            gravity_orientation = np.asarray(grav, dtype=np.float32)
            body_linear_acceleration = np.asarray(
                [imu_data['acc_x'], imu_data['acc_y'], imu_data['acc_z']],
                dtype=np.float32,
            )
            imu_valid = bool(
                np.all(np.isfinite(linvel))
                and np.all(np.isfinite(ang_vel))
                and np.all(np.isfinite(gravity_orientation))
                and np.all(np.isfinite(body_linear_acceleration))
            )

            # Match sim2sim convention: gravity is a unit vector in body frame.
            g_norm = float(np.linalg.norm(gravity_orientation))
            if g_norm > 1e-6:
                gravity_orientation = gravity_orientation / g_norm


        cmd = self.gamepad.get_command()
        
        self.cmd[0] = np.float32(cmd[0])
        self.cmd[1] = np.float32(cmd[1])
        self.cmd[2] = np.float32(cmd[2])

        qj_obs = self.qj.copy()
        qj_obs = qj_obs - self.config.default_angles
        dqj_obs = self.dqj.copy()
        dqj_obs = dqj_obs 

        self.obs[:3] = linvel * self.lin_vel_scale
        self.obs[3:6] = ang_vel * self.ang_vel_scale
        self.obs[6:9] = gravity_orientation
        self.obs[9:27] = qj_obs * self.dof_pos_scale
        self.obs[27:45] = dqj_obs * self.dof_vel_scale
        self.obs[45:64] = self.action

        # Command is stored in obs with scaling (match sim2sim layout).
        # Avoid printing at control rate (50Hz) since it can disturb timing.
        self.obs[64:67] = self.cmd * self.config.command_scale
        
        self.obs[67] = np.float32(max(float(tension_value), 0.0))
        self.obs[68:71] = tether_direction_body
        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        with torch.no_grad():
            action_tensor = self.policy(obs_tensor)
        self.action = action_tensor.detach().cpu().numpy().squeeze().copy()

        # Keep the raw output for the next last_action observation; only the
        # hardware commands use the safety/actuator limits.
        joint_action = np.clip(self.action[0:18], -10.0, 10.0)
        cable_action = np.clip(self.action[18], 0.0, 2.0)

        target_dof_pos = self.config.default_angles + joint_action * self.config.action_scale
        # target_dof_pos = self.config.default_angles
        
        target_tension = cable_action * self.config.tension_action_scale

        for i in range(18):
            q = target_dof_pos[i]
            #motor_id = int(self.config.joint2motor_idx[i])
            
            # Read actual motor position
            actual_pos = self.robot.motor_state_buffer.position[i]
            error = actual_pos - q

            # Add feedforward torque for Group2 joints with correct sign
            # Sign depends on motor_direction to ensure torque assists motion
            # group2_motor_ids = {18, 9, 3, 15, 11, 6}
            # if motor_id in group2_motor_ids:
            #     # Get motor direction (+1 or -1)
            #     motor_dir = self.config.motor_directions[i]
            #     # Apply feedforward in the direction that assists gravity compensation
            #     # If motor_dir is -1, we need to flip the sign
            #     feedforward_torque = 1.5 * motor_dir
            # else:
            #     feedforward_torque = 0.0

            self.robot.motor_command_buffer.kp[i] = float(self._kp_policy[i])
            self.robot.motor_command_buffer.kd[i] = float(self._kd_policy[i])
            self.robot.motor_command_buffer.target_position[i] = q
            self.robot.motor_command_buffer.target_velocity[i] = 0.0
            self.robot.motor_command_buffer.feedforward_torque[i] = 0.0


            # motor_id 仅用于对照打印
            # motor_id = self.config.joint2motor_idx[i]
            # print(f"policy_idx: [{i}] | motor_id: [{motor_id}] | q: [{q}]")
            #print(f"Vel: [{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}] | Grav: [{grav[0]:6.3f}, {grav[1]:6.3f}, {grav[2]:6.3f}]")
            

        # Spool speed command from reference/actual tension alignment.
        # `speed_input` is a user-chosen scalar (here: commanded forward speed from gamepad).
        # Paper-style: feedforward uses IMU speed (already read above).
        # Here we use forward velocity component; adjust to norm(linvel[:2]) if needed.
        spool_torque_cmd,feedback_torque, rff_torque,error_tension = self.tension_torque_controller.step(
            speed_input=float(cmd[0]),
            #speed_input=float(linvel[0]),
            yaw=float(tether_yaw),
            tension_ref=float(target_tension),
            tension_meas=float(tension_value),
        )
        self.robot.spool_command_buffer.target_torque_nm[0] = float(spool_torque_cmd)
        self.robot.spool_state_buffer.torque[0] = float(self.robot.spool_state_buffer.torque[0])  # Ensure torque is updated for next control step
        torque_limit = float(self.config.tsc_torque_limit)
        saturation_flag = abs(float(spool_torque_cmd)) >= max(
            torque_limit - 1e-6, 0.0
        )
        self.pre_tension_logger.write(
            timestamp_ns=observation_timestamp_ns,
            force_raw_n=float(tension_value),
            torque_actual_nm=spool_torque_actual_pre_action,
            torque_command_prev_nm=float(self._previous_spool_torque_command_nm),
            torque_command_issued_nm=float(spool_torque_cmd),
            motor_position_rad=float(self.spool_q[0]),
            force_reference_n=float(target_tension),
            velocity_command=self.cmd * self.config.command_scale,
            policy_action=self.action[:18] * float(self.config.action_scale),
            joint_position=self.qj,
            joint_velocity=self.dqj,
            body_gravity_vector=gravity_orientation,
            body_angular_velocity=ang_vel,
            body_linear_velocity=linvel,
            body_linear_acceleration=body_linear_acceleration,
            imu_valid=imu_valid,
            tension_sensor_valid=tension_sensor_valid,
            saturation_flag=saturation_flag,
            emergency_flag=False,
        )
        self._previous_spool_torque_command_nm = float(spool_torque_cmd)
        time.sleep(self.config.control_dt)


if __name__ == "__main__":

    config_path = f"{config_hexapod_tethered.ROOT_DIR}/deploy/deploy_real/configs/hexapod_tethered.yaml"
    config = Config(config_path)

    # ChannelFactoryInitialize(0, args.net)

    controller = Controller(config)

    time.sleep(1) # 等待系统稳定
    controller.print_initial_joint_positions()
    controller.print_imu_data()
    controller.print_pitch_angle()

    controller.print_yaw_angle()

    controller.zero_torque_state()
    enter_policy = controller.move_to_default_pos()
    if enter_policy:
        enter_policy = controller.default_pos_state()

    if enter_policy:
        print("Start main control loop. Press LB to exit.")

    while enter_policy:
        try:
            # 先处理安全按键，避免按下 LB/Y 后再多执行一次策略。
            if controller.gamepad.get_button_lb() == 1:
                break

            if controller.gamepad.get_button_y() == 1:
                # Safety: stop spool speed motor immediately on Y.
                create_zero_torque_cmd(controller.robot)
                if not controller.move_to_default_pos():
                    break
                # 进入默认位置保持状态，A 恢复策略，LB 进入阻尼。
                if not controller.default_pos_state():
                    break
                create_zero_torque_cmd(controller.robot)
                continue

            controller.run()
            
        except KeyboardInterrupt:
            break


    if getattr(controller, "gamepad", None) is not None:
        try:
            controller.gamepad.stop()
        except Exception:
            pass

    print("Entering damping mode to lower the robot safely. Press Ctrl+C to exit completely.")
    # Keep sending damping command continuously to keep the robot in damping mode
    try:
        while True:
            create_damping_cmd(controller.robot)
            create_zero_torque_cmd(controller.robot)
            time.sleep(config.control_dt)
    except KeyboardInterrupt:
        pass
        
    print('Exit')
