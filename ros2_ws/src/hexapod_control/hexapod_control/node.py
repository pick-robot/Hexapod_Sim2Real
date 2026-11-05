"""把 Gazebo 反馈转换成真机 Estimate 策略需要的输入；本文件不访问实机设备。"""

import csv
from datetime import datetime
from pathlib import Path
import sys
import time
import signal
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Empty


class EstimateController(Node):
    def __init__(self):
        super().__init__('hexapod_estimate_controller')
        defaults = {
            'project_root': '', 'real_config': '', 'policy_path': '',
            'initial_mode': 'default', 'gamepad': True, 'stand_duration': 2.0,
            'stand_kp': 150.0, 'stand_kd': 11.0, 'policy_kp': 100.0, 'policy_kd': 3.0,
            'damping_kd': 11.0, 'command_timeout': 0.5, 'sensor_timeout': 0.1, 'log_csv': True,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        self.root = Path(self.settings['project_root']).expanduser().resolve()
        deployment = self.root / 'deploy' / 'deploy_real'
        if not (deployment / 'hexapod_estimate_utils').is_dir():
            raise ValueError(f'Invalid project_root: {self.root}')
        # 仅导入配置与策略工具；不导入真机主脚本，避免启动 EtherCAT、串口或电机。
        sys.path.insert(0, str(deployment))
        from config_hexapod_estimate import Config
        from hexapod_estimate_utils import HexapodEstimatePolicy
        from hexapod_estimate_utils.observation import JOINT_NAMES
        from .core import EstimateCore
        import torch
        torch.set_num_threads(1)  # 小网络使用单线程，减少 ROS 回调中的线程调度抖动。
        config_file = self.settings['real_config'] or str(deployment / 'configs' / 'hexapod_estimate.yaml')
        self.config = Config(config_file)
        if self.settings['policy_path']:
            self.config.policy_path = str(Path(self.settings['policy_path']).expanduser().resolve())
        for key in ('stand_duration', 'command_timeout', 'sensor_timeout'):
            if not np.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('stand_kp', 'stand_kd', 'policy_kp', 'policy_kd', 'damping_kd'):
            maximum = 300 if key.endswith('kp') else 30
            if not np.isfinite(self.settings[key]) or not 0 <= self.settings[key] <= maximum:
                raise ValueError(f'Invalid gain {key}')
        self.command_scale = np.asarray(self.config.command_scale, dtype=float)
        if self.command_scale.shape != (3,) or not np.isfinite(self.command_scale).all() or np.any(self.command_scale <= 0):
            raise ValueError('command_scale must contain three positive values')
        policy = HexapodEstimatePolicy(self.config.policy_path, device='cpu')
        policy.warm_up()
        self.core = EstimateCore(self.config, policy, self.settings)
        if self.settings['initial_mode'] not in ('zero', 'default'):
            raise ValueError('initial_mode must be zero or default; policy always requires an explicit start')
        self.joint_names = tuple(JOINT_NAMES)
        self.positions = self.velocities = self.imu = None
        self.joint_stamp = self.imu_stamp = None
        self.command = np.zeros(3)
        self.command_received = -float('inf')
        self.ready = False
        self.last_tick = None
        self.time_reset = False
        self.reset_pause = None
        self._boot_switch = self._boot_unpause = None
        self._boot_started = time.monotonic()
        self._last_status = ''
        self._last_status_time = 0.0
        self.publisher = self.create_publisher(Float64MultiArray, '/hexapod_pd_controller/commands', 1)
        self.action_publisher = self.create_publisher(Float64MultiArray, '/hexapod/policy_action', 1)
        self.observation_publisher = self.create_publisher(Float64MultiArray, '/hexapod/observation', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, '/hexapod/state', latched)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        self.create_subscription(String, '/hexapod/mode', self.on_mode, 10)
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        self.pause_client = self.create_client(Empty, '/pause_physics')
        self.gamepad = None
        self.previous_buttons = {'lb': 0, 'y': 0, 'a': 0}
        self.setup_gamepad()
        self.log_file = self.log_writer = None
        self.last_log_flush = time.monotonic()
        if self.settings['log_csv']:
            directory = self.root / 'ros2_ws' / 'log' / 'control'
            directory.mkdir(parents=True, exist_ok=True)
            name = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
            self.log_file = (directory / f'estimate_{name}.csv').open('w', newline='')
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(['sim_time', 'state', 'inference_count', 'cmd_vx', 'cmd_vy', 'cmd_wz',
                                      *[f'q_{n}' for n in self.joint_names],
                                      *[f'dq_{n}' for n in self.joint_names],
                                      *[f'target_{n}' for n in self.joint_names],
                                      *[f'action_{n}' for n in self.joint_names],
                                      *[f'obs_{i}' for i in range(63)]])
            self.get_logger().info(f'CSV: {self.log_file.name}')
        # 策略使用 ROS 仿真时间；暂停后不推进 GRU，也不计算新的动作。
        self.policy_timer = self.create_timer(self.config.control_dt, self.tick)
        # 启动和手柄必须使用墙钟，否则暂停状态下无法完成激活/解除暂停握手。
        self.wall_timer = self.create_timer(0.02, self.wall_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Loaded real Estimate policy: {self.config.policy_path}; dt={self.config.control_dt}s')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def setup_gamepad(self):
        if not self.settings['gamepad']:
            return
        try:
            import os
            os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
            os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
            import pygame
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                self.get_logger().info('No gamepad; use keyboard_control.py or /cmd_vel + /hexapod/mode.')
                return
            from hexapod_tethered_utils.joystick_reader import Gamepad
            self.gamepad = Gamepad()  # 与真机使用同一个读取器、SDL 映射和环境变量覆盖。
        except Exception as exc:
            self.get_logger().warning(f'Gamepad unavailable: {exc}; keyboard/topic input remains available.')

    def on_joints(self, message):
        # ROS JointState 可被重新排序；绝不能把数组下标直接当作策略关节下标。
        index = {name: i for i, name in enumerate(message.name)}
        if not all(name in index for name in self.joint_names):
            return
        try:
            q = np.array([message.position[index[name]] for name in self.joint_names], dtype=np.float32)
            dq = np.array([message.velocity[index[name]] for name in self.joint_names], dtype=np.float32)
        except IndexError:
            return
        if not np.isfinite([q, dq]).all():
            self.fault('Non-finite joint feedback')
            return
        self.positions, self.velocities = q, dq
        self.joint_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def on_imu(self, message):
        if message.header.frame_id != 'trunk':
            self.fault(f'Expected IMU frame trunk, got {message.header.frame_id}')
            return
        self.imu = message
        self.imu_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def on_command(self, message):
        command = np.array([message.linear.x, message.linear.y, message.angular.z])
        if not np.isfinite(command).all():
            self.fault('Non-finite velocity command')
            return
        # ROS 接口使用物理单位；摇杆归一化数值的缩放只在 gamepad 分支做一次。
        self.command = np.clip(command, -self.command_scale, self.command_scale)
        self.command_received = time.monotonic()

    def on_mode(self, message):
        self.set_mode(message.data.strip().lower())

    def set_mode(self, mode):
        if self.time_reset:
            self.get_logger().warning('Simulation time was reset; restart estimate.launch.py before controlling again.')
            return
        if not self.ready and mode not in ('zero', 'damping'):
            self.get_logger().warning('Controller is still starting; wait for READY.')
            return
        try:
            self.core.request_mode(mode)
            # 手柄/ROS 输入可以在暂停时变更模式，但不会推进策略。
            self.publish_status(force=True)
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def fault(self, reason):
        if self.core.mode != 'damping':
            self.get_logger().error(f'{reason}; entering latched damping mode')
            self.core.request_mode('damping')
            self.command.fill(0)
            self.publish_status(force=True)

    def publish_status(self, force=False):
        state = self.core.mode if self.ready else 'starting'
        wall_now = time.monotonic()
        if force or state != self._last_status or wall_now-self._last_status_time > 0.2:
            self.status_publisher.publish(String(data=state))
            if state != self._last_status:
                self.get_logger().info(f'State: {state}')
            self._last_status, self._last_status_time = state, wall_now

    def wall_tick(self):
        if self.time_reset and self.reset_pause is None and self.pause_client.service_is_ready():
            self.reset_pause = self.pause_client.call_async(Empty.Request())
        if not self.ready:
            # spawner 已把 PD 控制器配置为 inactive。先预热网络，再请求激活并
            # 解除暂停，让 controller_manager 有仿真步来完成 switch。
            if self._boot_switch is None and self.switch_client.service_is_ready() and self.unpause_client.service_is_ready():
                request = SwitchController.Request()
                request.activate_controllers = ['hexapod_pd_controller']
                request.strictness = SwitchController.Request.STRICT
                request.activate_asap = True
                request.timeout.sec = 10
                self._boot_switch = self.switch_client.call_async(request)
                self._boot_unpause = self.unpause_client.call_async(Empty.Request())
            if self._boot_switch is not None and self._boot_switch.done() and self._boot_unpause.done():
                if not self._boot_switch.result().ok:
                    raise RuntimeError('Could not activate hexapod_pd_controller')
                if self.positions is not None and self.imu is not None:
                    self.ready = True
                    # 初始 zero/default 是显式配置；不会自动启动行走策略。
                    if self.core.mode != 'damping':
                        self.core.request_mode(self.settings['initial_mode'])
                    self.get_logger().info('READY: Y=default, A=policy, LB=damping. Keyboard: y/a/space.')
            if time.monotonic()-self._boot_started > 60:
                raise RuntimeError('Startup timed out waiting for controller and Gazebo sensors')
        if self.gamepad is not None:
            if not self.gamepad.is_running:
                self.fault('Gamepad disconnected')
                self.gamepad = None
            else:
                buttons = {key: int(getattr(self.gamepad, f'get_button_{key}')()) for key in self.previous_buttons}
                # LB 始终优先；用上升沿避免按住 A 每周期重置 GRU。
                if buttons['lb']:
                    if self.core.mode != 'damping':
                        self.set_mode('damping')
                else:
                    for key, mode in [('y', 'default'), ('a', 'policy')]:
                        if buttons[key] and not self.previous_buttons[key]:
                            self.set_mode(mode)
                            break
                self.previous_buttons = buttons
                self.command = np.clip(self.gamepad.get_command(), -1, 1)*self.command_scale
                self.command_received = time.monotonic()
        self.publish_status()

    def tick(self):
        if not self.ready or self.time_reset:
            return
        now = self.now_seconds()
        if self.last_tick is not None and 0 <= now-self.last_tick < self.config.control_dt*0.5:
            return  # /clock 突跳或回调积压时，不能对同一仿真时刻重复推进 GRU。
        # 在读取旧时间戳的传感器之前识别 /clock 倒退，禁止跨回合沿用命令/GRU。
        if self.last_tick is not None and now < self.last_tick-1e-6:
            self.core.request_mode('zero')
            self.command.fill(0)
            self.command_received = -float('inf')
            # 本机 gazebo_ros2_control 0.4.10 没有重置内部更新时间的 Reset()
            # 实现。时间倒退后 PD 不会继续更新，不能仅重置 Python GRU 后声称恢复。
            # 清空历史并暂停场景，要求重新启动整个 launch 来重建插件。
            self.time_reset = True
            self.get_logger().error('Gazebo time reset: installed gazebo_ros2_control cannot resume its update loop. Pausing simulation; restart estimate.launch.py.')
            self.fault('Simulation time reset')
            return
        self.last_tick = now
        stamps = (self.joint_stamp, self.imu_stamp)
        fresh = all(stamp is not None and -0.02 <= now-stamp <= self.settings['sensor_timeout'] for stamp in stamps)
        if not fresh:
            self.fault('Joint/IMU feedback stale or simulation reset')
            # 停止发送正常目标；独立 C++ watchdog 会保持阻尼。
            return
        command = self.command.copy() if time.monotonic()-self.command_received <= self.settings['command_timeout'] else np.zeros(3)
        imu = self.imu
        angular = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        try:
            packet = self.core.step(now, self.positions, self.velocities, angular, quat, command)
        except Exception as exc:
            self.fault(f'Policy/control failure: {exc}')
            packet = np.concatenate((np.zeros(54), np.full(18, self.settings['damping_kd']), np.zeros(18)))
        self.publisher.publish(Float64MultiArray(data=packet.tolist()))
        if self.core.mode == 'policy':
            self.action_publisher.publish(Float64MultiArray(data=self.core.action.tolist()))
            self.observation_publisher.publish(Float64MultiArray(data=self.core.observation.tolist()))
        if self.log_writer is not None:
            self.log_writer.writerow([now, self.core.mode, self.core.inference_count, *command,
                                      *self.positions, *self.velocities, *packet[:18],
                                      *self.core.action, *self.core.observation])
            if time.monotonic()-self.last_log_flush >= 1.0:
                self.log_file.flush()
                self.last_log_flush = time.monotonic()
        self.publish_status()

    def close(self):
        # SIGINT 时尽量先发送阻尼；若 DDS 来不及送达，C++ watchdog 仍独立生效。
        try:
            packet = np.concatenate((np.zeros(54), np.full(18, self.settings['damping_kd']), np.zeros(18)))
            self.publisher.publish(Float64MultiArray(data=packet.tolist()))
        except Exception:
            pass
        if self.gamepad is not None:
            self.gamepad.stop()
        if self.log_file is not None:
            self.log_file.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = EstimateController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
