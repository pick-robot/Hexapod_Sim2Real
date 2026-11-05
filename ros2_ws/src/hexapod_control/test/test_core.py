"""重点验证真机观测合同、GRU 回合边界和坐标转换，不启动实机硬件。"""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'deploy' / 'deploy_real'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config_hexapod_estimate import Config
from hexapod_estimate_utils import HexapodEstimatePolicy, build_observation
from hexapod_control.core import EstimateCore, projected_gravity

GAINS = dict(stand_duration=2., stand_kp=150., stand_kd=11., policy_kp=100., policy_kd=3., damping_kd=11.)


class FakePolicy:
    def __init__(self):
        self.resets = 0
    def reset(self):
        self.resets += 1
    def predict(self, observation):
        return np.linspace(-20, 20, 18, dtype=np.float32)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.config = Config(ROOT / 'deploy/deploy_real/configs/hexapod_estimate.yaml')
        self.policy = FakePolicy()
        self.core = EstimateCore(self.config, self.policy, GAINS)

    def step(self, t, **overrides):
        args = dict(position=np.zeros(18), velocity=np.zeros(18), angular_velocity=np.zeros(3),
                    quaternion=[0, 0, 0, 1], command=np.zeros(3))
        args.update(overrides)
        return self.core.step(t, **args)

    def standing(self):
        self.core.request_mode('default')
        self.step(0.)
        self.step(2.)
        self.assertEqual(self.core.mode, 'default')

    def test_gravity_rotations_and_normalization(self):
        np.testing.assert_allclose(projected_gravity([0, 0, 0, 2]), [0, 0, -1])
        np.testing.assert_allclose(projected_gravity([0, 0, 1, 0]), [0, 0, -1])
        np.testing.assert_allclose(projected_gravity([1, 0, 0, 1]), [0, -1, 0], atol=1e-7)
        np.testing.assert_allclose(projected_gravity([0, 1, 0, 1]), [1, 0, 0], atol=1e-7)
        for q in ([0, 0, 0, 0], [float('nan'), 0, 0, 1]):
            with self.assertRaises(ValueError): projected_gravity(q)

    def test_transition_and_policy_gate(self):
        with self.assertRaises(ValueError): self.core.request_mode('policy')
        self.core.request_mode('default')
        self.step(1., position=np.full(18, .2))
        packet = self.step(2.)
        np.testing.assert_allclose(packet[:18], .1)
        with self.assertRaises(ValueError): self.core.request_mode('policy')
        self.step(3.)
        self.core.request_mode('policy')
        packet = self.step(3.02)
        np.testing.assert_allclose(packet[36:54], 100)
        np.testing.assert_allclose(packet[54:72], 3)
        np.testing.assert_allclose(packet[:18], np.clip(np.linspace(-20, 20, 18), -10, 10)*.25)

    def test_history_and_damping_latch(self):
        self.standing()
        self.core.request_mode('policy'); self.step(2.02)
        self.assertTrue(np.any(self.core.action))
        resets = self.policy.resets
        self.core.request_mode('policy')
        self.assertEqual(self.policy.resets, resets)  # 按住 A 不重置历史。
        self.core.request_mode('damping')
        packet = self.step(2.04)
        np.testing.assert_allclose(self.core.action, 0)
        np.testing.assert_allclose(packet[36:54], 0)
        np.testing.assert_allclose(packet[54:72], 11)
        with self.assertRaises(ValueError): self.core.request_mode('default')
        self.core.request_mode('zero')
        np.testing.assert_allclose(self.step(2.06)[36:72], 0)

    def test_clock_rewind_clears_recurrent_state(self):
        self.standing(); self.core.request_mode('policy'); self.step(2.02)
        packet = self.step(.01)
        self.assertEqual(self.core.mode, 'zero')
        np.testing.assert_allclose(self.core.action, 0)
        np.testing.assert_allclose(packet[36:72], 0)

    def test_actual_policy_matches_real_deployment_sequence(self):
        # 两个独立 GRU 实例比较多步结果，捕获观测重排、命令二次缩放和历史串回合。
        import torch
        torch.set_num_threads(1)
        actual = HexapodEstimatePolicy(self.config.policy_path)
        expected = HexapodEstimatePolicy(self.config.policy_path)
        self.core = EstimateCore(self.config, actual, GAINS)
        self.standing(); self.core.request_mode('policy')
        previous = np.zeros(18, dtype=np.float32)
        rng = np.random.default_rng(5)
        for i in range(8):
            q = rng.uniform(-.1, .1, 18).astype(np.float32)
            dq = rng.uniform(-.5, .5, 18).astype(np.float32)
            gyro = rng.uniform(-.1, .1, 3).astype(np.float32)
            command = np.array([.2, -.1, .05], dtype=np.float32)
            obs = build_observation(gyro, [0, 0, -1], q, dq, previous, command)
            previous = np.clip(expected.predict(obs), -10, 10)
            self.step(2.02+i*.02, position=q, velocity=dq, angular_velocity=gyro, command=command)
            np.testing.assert_allclose(self.core.observation, obs, atol=1e-7)
            np.testing.assert_allclose(self.core.action, previous, atol=1e-6)
        self.core.request_mode('default'); self.step(3.); self.step(5.)
        self.core.request_mode('policy'); expected.reset()
        self.step(5.02)
        obs = build_observation([0, 0, 0], [0, 0, -1], np.zeros(18), np.zeros(18), np.zeros(18), [0, 0, 0])
        np.testing.assert_allclose(self.core.action, np.clip(expected.predict(obs), -10, 10), atol=1e-6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
