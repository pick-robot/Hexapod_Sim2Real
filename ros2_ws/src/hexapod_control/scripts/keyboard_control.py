#!/usr/bin/env python3
"""终端控制：状态机和速度分开，始终以 10 Hz 发送速度心跳。"""
import select
import os
import sys
import termios
import time
import tty
import signal

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.signals import SignalHandlerOptions


def main():
    if not sys.stdin.isatty():
        raise SystemExit('请在交互终端运行 keyboard_control.py')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('hexapod_keyboard')
    velocity = node.create_publisher(Twist, '/cmd_vel', 1)
    mode = node.create_publisher(String, '/hexapod/mode', 10)
    command = [0.0, 0.0, 0.0]
    saved = termios.tcgetattr(sys.stdin)
    print('y: 默认姿态 | a: 策略 | 空格: 阻尼 | z: 零力矩复位\n'
          'w/s: 前后速度 ±0.1 | j/l: 左右 ±0.1 | q/e: 转向 ±0.1\n'
          'x: 速度清零 | Ctrl+C / Esc: 阻尼并退出\n'
          '速度使用 m/s、rad/s；控制端按真机 command_scale 限幅。')
    last_publish = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.03)[0]:
                # 直接读取 fd，避免 TextIOWrapper 预读后 select 看不到缓冲区里的后续按键。
                key = os.read(sys.stdin.fileno(), 1).decode(errors='ignore').lower()
                if key in ('\x03', '\x1b'):
                    break
                if key in ('y', 'a', ' ', 'z'):
                    selected = {'y': 'default', 'a': 'policy', ' ': 'damping', 'z': 'zero'}[key]
                    if key in (' ', 'z', 'y'):
                        command = [0.0, 0.0, 0.0]
                    mode.publish(String(data=selected))
                    print(f'\r模式请求: {selected}                  ')
                elif key == 'x':
                    command = [0.0, 0.0, 0.0]
                elif key in ('w', 's', 'j', 'l', 'q', 'e'):
                    index, delta = {'w': (0, .1), 's': (0, -.1), 'j': (1, .1),
                                    'l': (1, -.1), 'q': (2, .1), 'e': (2, -.1)}[key]
                    command[index] = round(max(-.8, min(.8, command[index]+delta)), 2)
                    print(f'\r速度 vx/vy/wz: {command}                  ')
            if time.monotonic()-last_publish >= .1:
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = command
                velocity.publish(msg)
                last_publish = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # 先恢复终端；即使 Gazebo/ROS 已退出，也不能把用户终端留在 cbreak。
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        if rclpy.ok():
            velocity.publish(Twist())
            mode.publish(String(data='damping'))
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
