#!/usr/bin/env python3
"""在专用测试场景中验证策略、真实位移、超时、暂停和阻尼；结束停在阻尼状态。"""
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Empty


def main():
    rclpy.init()
    node = rclpy.create_node('check_hexapod_estimate')
    data = {'mode': None, 'pose': None, 'action_count': 0, 'clock': 0., 'obs': None, 'packet': None, 'motor_count': 0}
    def models(message):
        if 'hexapod' in message.name:
            data['pose'] = message.pose[message.name.index('hexapod')]
    def action(message):
        assert len(message.data) == 18 and all(math.isfinite(x) for x in message.data)
        data['action_count'] += 1
    def motor(message):
        assert len(message.effort)==18 and all(math.isfinite(x) and abs(x)<=30.000001 for x in message.effort)
        data['motor_count'] += 1
    subscriptions = [
        node.create_subscription(String, '/hexapod/state', lambda m: data.update(mode=m.data), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(ModelStates, '/gazebo/model_states', models, qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', lambda m: data.update(clock=m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data),
        node.create_subscription(Float64MultiArray, '/hexapod/policy_action', action, 10),
        node.create_subscription(Float64MultiArray, '/hexapod/observation', lambda m: data.update(obs=list(m.data)), 10),
        node.create_subscription(Float64MultiArray, '/hexapod_pd_controller/commands', lambda m: data.update(packet=list(m.data)), 10),
        node.create_subscription(JointState, '/hexapod_pd_controller/state', motor, qos_profile_sensor_data),
    ]
    mode = node.create_publisher(String, '/hexapod/mode', 10)
    velocity = node.create_publisher(Twist, '/cmd_vel', 1)
    def until(predicate, timeout=30, heartbeat=None):
        start = time.monotonic()
        last_heartbeat = -1.
        while not predicate():
            assert time.monotonic()-start < timeout, f'Timed out; state={data["mode"]}'
            if heartbeat and time.monotonic()-last_heartbeat > .05:
                heartbeat(); last_heartbeat = time.monotonic()
            rclpy.spin_once(node, timeout_sec=.01)
    def service(name):
        c = node.create_client(Empty, name)
        assert c.wait_for_service(timeout_sec=10), name
        f=c.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node,f,timeout_sec=10)
        assert f.done() and f.result() is not None, name
        node.destroy_client(c)
    def send_velocity(x):
        msg=Twist(); msg.linear.x=x; velocity.publish(msg)
    paused = False
    try:
        until(lambda: data['mode']=='default' and data['pose'] is not None and mode.get_subscription_count()>0)
        assert data['pose'].position.z > .22, f'Robot is not standing: {data["pose"].position.z}'
        print('PASS: standing controller ready.', flush=True)
        mode.publish(String(data='policy'))
        until(lambda: data['mode']=='policy' and data['action_count']>5)
        start_clock, start_count, start_x = data['clock'], data['action_count'], data['pose'].position.x
        minimum_height = [data['pose'].position.z]
        def record():
            minimum_height.append(data['pose'].position.z)
            send_velocity(.2)
        until(lambda: data['clock']-start_clock >= 8., timeout=50, heartbeat=record)
        elapsed = data['clock']-start_clock
        rate=(data['action_count']-start_count)/elapsed
        displacement=data['pose'].position.x-start_x
        print(f'Walking: {elapsed:.2f}s, dx={displacement:.3f}m, policy={rate:.1f}Hz, min_z={min(minimum_height):.3f}m', flush=True)
        assert 43 < rate < 57, f'Policy is not running at 50Hz: {rate}'
        assert min(minimum_height) > .20, 'Robot fell while walking'
        assert data['motor_count']>100, 'No PD telemetry received'
        assert displacement > .15, f'Forward command did not produce forward walking: {displacement}'
        assert len(data['obs'])==63 and abs(data['obs'][60]-.2)<1e-6
        # 停止速度心跳；0.5 秒墙钟后 obs 的命令应归零，策略本身继续运行。
        deadline=time.monotonic()+.8
        until(lambda: time.monotonic()>deadline, timeout=2)
        until(lambda: max(abs(x) for x in data['obs'][60:63])<1e-6, timeout=3)
        print('PASS: torque limits, physical-unit command and velocity timeout.', flush=True)
        service('/pause_physics'); paused=True
        drain=time.monotonic()+.15
        until(lambda: time.monotonic()>drain, timeout=1)
        count=data['action_count']
        deadline=time.monotonic()+.3
        until(lambda: time.monotonic()>deadline, timeout=1)
        assert data['action_count']==count, 'GRU advanced while simulation was paused'
        service('/unpause_physics'); paused=False
        print('PASS: pause freezes policy updates.', flush=True)
        mode.publish(String(data='default'))
        until(lambda: data['mode']=='moving_to_default')
        until(lambda: data['mode']=='default')
        mode.publish(String(data='policy'))
        until(lambda: data['mode']=='policy')
        mode.publish(String(data='damping'))
        until(lambda: data['mode']=='damping' and data['packet'] is not None and max(abs(x) for x in data['packet'][36:54])==0)
        assert all(abs(x-11)<1e-6 for x in data['packet'][54:72])
        print('PASS: default → policy → damping state transitions.', flush=True)
        service('/reset_simulation')
        until(lambda: data['clock']<1., timeout=8)
        deadline=time.monotonic()+.3
        until(lambda: time.monotonic()>deadline, timeout=2)
        frozen=data['clock']
        deadline=time.monotonic()+.3
        until(lambda: time.monotonic()>deadline, timeout=2)
        assert data['mode']=='damping' and data['clock']==frozen, 'Reset should latch stop and pause incompatible Gazebo control plugin'
        print('PASS: unsupported time reset is detected and simulation pauses; restart launch to resume.', flush=True)
    finally:
        velocity.publish(Twist()); mode.publish(String(data='damping'))
        if paused: service('/unpause_physics')
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
