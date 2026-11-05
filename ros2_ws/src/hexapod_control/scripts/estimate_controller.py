#!/usr/bin/env python3
"""ROS 入口；launch 可指定装有 PyTorch 的 Python 3.10 解释器。"""
from hexapod_control.node import main

if __name__ == '__main__':
    main()
