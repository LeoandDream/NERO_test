#!/usr/bin/env python3
"""对照只读 SDK 客户端被强制终止前后的机械臂状态；默认只读。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh
from experiments.single_arm.can_setup.recover_emergency_stop import snapshot


REPORT_DIR = Path("experiments/single_arm/can_setup/data/diagnostics")
MODULE = "experiments.single_arm.can_setup.probe_client_exit"


def create_robot(channel):
    return AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
        channel=channel, local_loopback=True,
    ))


def read_state(channel):
    """每次新建连接，读取状态及七个驱动使能位，然后断开。"""
    robot = create_robot(channel)
    robot.connect()
    try:
        result = snapshot(robot)
        deadline = time.monotonic() + 3.0
        drivers = [None] * 7
        while time.monotonic() < deadline and any(value is None for value in drivers):
            for index in range(7):
                feedback = robot.get_driver_states(index + 1)
                if fresh(feedback):
                    drivers[index] = bool(feedback.msg.foc_status.driver_enable_status)
            if any(value is None for value in drivers):
                time.sleep(0.05)
        if any(value is None for value in drivers):
            raise RuntimeError("七轴驱动使能反馈不完整")
        result["joint_enabled"] = drivers
        return result
    finally:
        robot.disconnect()


def child(channel):
    """只建立 SDK 读连接；父进程会用 SIGKILL 结束本进程。"""
    robot = create_robot(channel)
    robot.connect()
    snapshot(robot)
    print("READY", flush=True)
    while True:
        robot.get_arm_status()
        time.sleep(0.1)


def run_trial(channel, before):
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(),
              "channel": channel, "before": before, "child_sent_commands": False,
              "kill_signal": "SIGKILL", "after": None}
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    path = REPORT_DIR / ("nero_client_exit_" +
                         datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    process = subprocess.Popen(
        [sys.executable, "-m", MODULE, "--child", "--channel", channel],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        readable, _, _ = select.select([process.stdout], [], [], 10.0)
        line = process.stdout.readline().strip() if readable else ""
        if line != "READY":
            raise RuntimeError(f"只读子进程未准备好：{line or '无输出'}")
        report["child_pid"] = process.pid
        report["child_ready_at_utc"] = datetime.now(timezone.utc).isoformat()
        print(f"只读客户端 PID {process.pid} 已连接；2 秒后强制结束。", flush=True)
        time.sleep(2.0)
        process.kill()
        process.wait(timeout=3.0)
        report["child_killed_at_utc"] = datetime.now(timezone.utc).isoformat()
        print("子进程已结束；只观察 5 秒，不发送使能、复位或运动指令。", flush=True)
        time.sleep(5.0)
        report["after"] = read_state(channel)
        return path, report
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.0)
        process.stdout.close()
        process.stderr.close()
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--run", action="store_true",
                        help="强制结束一个只读 SDK 客户端；可能触发控制器失能")
    parser.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.child:
        child(args.channel)
        return 0
    try:
        before = read_state(args.channel)
        print("当前机械臂状态码：", before["arm_status"])
        print("七轴使能：", before["joint_enabled"])
        print("当前关节角 (rad)：", before["joint_rad"])
        if not args.run:
            print("只读预检完成；未启动或终止对照客户端。")
            return 0
        if (before["control_mode"] != 1 or before["arm_status"] != 0
                or before["teach_status"] != 0 or before["joint_error_code"] != 0
                or not all(before["joint_enabled"])):
            raise RuntimeError("机械臂未处于正常 CAN 使能状态；拒绝对照")
        print("本试验可能使机械臂失能下落；现场必须可靠支撑并避开夹点。")
        for remaining in range(5, 0, -1):
            print(f"{remaining} 秒后开始只读客户端退出对照；按 Ctrl+C 取消。", flush=True)
            time.sleep(1.0)
        path, report = run_trial(args.channel, before)
        print("对照记录：", path)
        print("终止 5 秒后机械臂状态码：", report["after"]["arm_status"])
        print("终止 5 秒后七轴使能：", report["after"]["joint_enabled"])
        return 0
    except (OSError, RuntimeError, ValueError, TimeoutError, KeyboardInterrupt) as exc:
        print(f"对照未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
