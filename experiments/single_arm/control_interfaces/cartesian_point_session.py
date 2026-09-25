#!/usr/bin/env python3
"""Nero 法兰点位运动：默认只读，显式 --run 才发送一次 move_p。"""

from experiments.single_arm.control_interfaces.cartesian_linear_session import main


if __name__ == "__main__":
    raise SystemExit(main(motion_mode="p"))
