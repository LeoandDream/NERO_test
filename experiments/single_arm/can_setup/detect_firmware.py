#!/usr/bin/env python3
"""Read the Nero firmware version over SocketCAN and select its SDK profile."""

import argparse
import sys
import time

from pyAgxArm import (
    AgxArmFactory,
    ArmModel,
    create_agx_arm_config,
    resolve_firmware_profile,
)


def main() -> int:
    """按重试次数读取固件响应并选择 SDK 档位；仅查询 CAN，不使能机械臂。"""
    parser = argparse.ArgumentParser(description="查询 Nero 机械臂固件版本")
    parser.add_argument("--channel", default="can0", help="CAN 接口名，默认 can0")
    parser.add_argument("--timeout", type=float, default=2.0, help="每次查询超时秒数，默认 2")
    parser.add_argument("--attempts", type=int, default=3, help="查询次数，默认 3")
    args = parser.parse_args()

    if args.timeout <= 0:
        parser.error("--timeout 必须大于 0")
    if args.attempts < 1:
        parser.error("--attempts 必须至少为 1")

    config = create_agx_arm_config(robot=ArmModel.NERO, channel=args.channel)
    robot = AgxArmFactory.create_arm(config)
    firmware = None

    try:
        robot.connect()
        for attempt in range(args.attempts):
            firmware = robot.get_firmware(timeout=args.timeout)
            if firmware is not None:
                break
            if attempt + 1 < args.attempts:
                time.sleep(1.0)
    except Exception as exc:
        print(f"固件查询失败：{exc}", file=sys.stderr)
        return 1
    finally:
        robot.disconnect()

    if firmware is None:
        print(f"未收到固件响应；请检查 {args.channel}、CAN 接线和机械臂电源。", file=sys.stderr)
        return 1

    version = firmware["software_version"]
    profile = resolve_firmware_profile(ArmModel.NERO, version)
    print(f"固件版本：{version}")
    print(f"SDK 驱动档位：{profile}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
