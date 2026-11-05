"""与 ROS 无关的状态机；观测构造和 GRU 推理复用真机代码。"""

import numpy as np

from hexapod_estimate_utils import build_observation
from hexapod_estimate_utils.observation import JOINT_NAMES


def projected_gravity(quaternion):
    """Gazebo IMU 给出 body→world 四元数 xyzw，返回机体坐标中的单位重力。

    使用姿态而非 accelerometer：足端碰撞与机体运动会污染比力，不能直接
    把 IMU linear_acceleration 归一化当作 projected_gravity。
    """
    quat = np.asarray(quaternion, dtype=np.float64)
    if quat.shape != (4,) or not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-8:
        raise ValueError('IMU orientation is invalid')
    x, y, z, w = quat / np.linalg.norm(quat)
    return np.array([-2*(x*z-w*y), -2*(y*z+w*x), -(1-2*(x*x+y*y))], dtype=np.float32)


class EstimateCore:
    """Y→2 秒默认姿态、A→策略、LB→阻尼；每次进出策略清除 GRU 历史。"""

    def __init__(self, config, policy, gains):
        self.config, self.policy, self.gains = config, policy, gains
        if config.num_actions != 18 or config.num_obs != 63 or config.control_dt <= 0:
            raise ValueError('Estimate requires 18 actions, 63 observations and positive control_dt')
        if not np.isfinite(config.default_angles).all() or len(config.default_angles) != 18:
            raise ValueError('Expected 18 finite default joint angles')
        self.mode = 'zero'
        self.action = np.zeros(18, dtype=np.float32)
        self.observation = np.zeros(63, dtype=np.float32)
        self.target = np.asarray(config.default_angles, dtype=np.float32).copy()
        self.transition_start = None
        self.start_position = None
        self.last_time = None
        self.inference_count = 0

    def reset_history(self):
        self.policy.reset()
        self.action.fill(0)

    def request_mode(self, mode):
        """按钮/话题共享一个入口；阻尼后先显式 zero 复位才能重新站立。"""
        if mode not in ('zero', 'default', 'policy', 'damping'):
            raise ValueError(f'Unknown mode: {mode}')
        if self.mode == 'damping' and mode not in ('damping', 'zero'):
            raise ValueError('Damping is latched; select zero before standing again')
        if mode == 'policy' and self.mode not in ('default', 'policy'):
            raise ValueError('Finish the default-pose transition before starting policy')
        if mode == 'policy' and self.mode == 'policy':
            return
        self.reset_history()
        self.mode = 'moving_to_default' if mode == 'default' else mode
        self.transition_start = None
        self.start_position = None

    def step(self, now, position, velocity, angular_velocity, quaternion, command):
        position, velocity = np.asarray(position), np.asarray(velocity)
        if position.shape != (18,) or velocity.shape != (18,) or not np.isfinite([position, velocity]).all():
            raise ValueError('Invalid joint feedback')
        # Gazebo /reset_simulation 会让 /clock 倒退：丢弃上一回合的 GRU、动作和状态。
        if self.last_time is not None and now < self.last_time - 1e-6:
            self.request_mode('zero')
        self.last_time = now
        kp, kd = np.zeros(18), np.zeros(18)
        if self.mode == 'moving_to_default':
            if self.transition_start is None:
                self.transition_start = now
                self.start_position = position.copy()
            alpha = np.clip((now-self.transition_start)/self.gains['stand_duration'], 0, 1)
            self.target = (1-alpha)*self.start_position + alpha*self.config.default_angles
            if alpha >= 1:
                self.mode = 'default'
        if self.mode in ('moving_to_default', 'default'):
            kp.fill(self.gains['stand_kp'])
            kd.fill(self.gains['stand_kd'])
        elif self.mode == 'policy':
            self.observation[:] = build_observation(
                np.asarray(angular_velocity)*self.config.ang_vel,
                projected_gravity(quaternion),
                (position-self.config.default_angles)*self.config.dof_pos,
                velocity*self.config.dof_vel,
                self.action,
                command,  # /cmd_vel 已经是 m/s、rad/s，手柄在输入层做一次 command_scale。
            )
            # 与真机完全相同：先裁剪原始动作到 ±10，再乘 0.25。
            self.action = np.clip(self.policy.predict(self.observation), -10.0, 10.0)
            self.target = self.config.default_angles + self.config.action_scale*self.action
            kp.fill(self.gains['policy_kp'])
            kd.fill(self.gains['policy_kd'])
            self.inference_count += 1
        elif self.mode == 'damping':
            kd.fill(self.gains['damping_kd'])
        # 不给目标位置额外裁剪：保留真机 action→target 映射；Gazebo 关节自身有限位，
        # 电机层把最终 PD 力矩裁剪到 ±30 Nm。zero/damping 时 target 因 kp=0 不生效。
        return np.concatenate((self.target, np.zeros(18), kp, kd, np.zeros(18)))
