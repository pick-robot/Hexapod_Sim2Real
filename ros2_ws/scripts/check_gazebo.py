#!/usr/bin/env python3
"""Check a freshly spawned, paused robot; briefly run physics and reset it."""

import argparse
import math
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from controller_manager_msgs.srv import ListHardwareInterfaces
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Empty


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-control', action='store_true',
                        help='Also check effort interfaces (launch use_ros2_control:=true).')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('hexapod_import_check')
    samples = {}
    subscriptions = [
        node.create_subscription(JointState, '/joint_states',
                                 lambda m: samples.update(joints=m), qos_profile_sensor_data),
        node.create_subscription(Imu, '/imu/data',
                                 lambda m: samples.update(imu=m), qos_profile_sensor_data),
    ]

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f'Service unavailable: {name}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'Service failed: {name}')
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        request = GetEntityState.Request(name='hexapod', reference_frame='world')
        state = call(GetEntityState, '/gazebo/get_entity_state', request)
        assert state.success, 'hexapod entity missing'
        assert abs(state.state.pose.position.z - 0.313) < 0.001, 'Expected fresh default spawn'
        if args.check_control:
            hardware = call(ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces',
                            ListHardwareInterfaces.Request())
            expected_efforts = {f'{leg}_{axis}/effort' for leg in ('RF', 'RM', 'RB', 'LF', 'LM', 'LB')
                                for axis in ('HAA', 'HFE', 'KFE')}
            assert {i.name for i in hardware.command_interfaces} == expected_efforts
            assert all(i.is_available for i in hardware.command_interfaces)
            print('PASS: all 18 effort command interfaces are available.')
        try:
            call(Empty, '/unpause_physics', Empty.Request())
            deadline = time.monotonic() + 5
            while len(samples) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert len(samples) == 2, f'Missing sensor messages: {samples.keys()}'
            joints = samples['joints']
            expected = [f'{leg}_{axis}' for leg in ('RF', 'RM', 'RB', 'LF', 'LM', 'LB')
                        for axis in ('HAA', 'HFE', 'KFE')]
            assert list(joints.name) == expected, joints.name
            assert len(joints.position) == len(joints.velocity) == 18
            assert all(math.isfinite(x) for x in (*joints.position, *joints.velocity))
            imu = samples['imu']
            assert imu.header.frame_id == 'trunk', imu.header.frame_id
            values = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w,
                      imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                      imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
            assert all(math.isfinite(x) for x in values), values
            assert abs(sum(x*x for x in values[:4]) - 1) < 1e-3, 'Invalid IMU orientation'
            print('PASS: hexapod spawned; all 18 ordered joints and finite IMU data received.')
        finally:
            call(Empty, '/pause_physics', Empty.Request())
            call(Empty, '/reset_simulation', Empty.Request())
            state = call(GetEntityState, '/gazebo/get_entity_state', request)
            assert state.success and abs(state.state.pose.position.z - 0.313) < 0.001
            print('Restored paused simulation and initial robot pose.')
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
