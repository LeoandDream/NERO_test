"""One-time monitored disable at the operator-confirmed, lightly supported H0 candidate.

This is an acceptance trial, not an unattended parking command. It sends no
motion target and never retries the disable command.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.planned_return import _write_new
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


SOURCE_Q = [0.22998203553529278, 0.31084313978019007,
            -0.028082347664588763, -0.05707226654021458,
            1.6599302916942469, 0.13151055913777274,
            0.592539281052075]
REPORT_DIR = Path("experiments/single_arm/lab/data/h0_disable_trials")


def check_pre_disable(robot, state, config):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("仅适用于这次左侧安装")
    if max(abs(a-b) for a, b in zip(state["joint_rad"], SOURCE_Q)) > 0.003:
        raise ValueError("当前姿态已偏离现场轻触承托点；未发送失能命令")
    if math.dist(robot.fk(state["joint_rad"])[:3],
                 state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    return state


def run(*, execute=False, countdown_s=5):
    config = load_config(DEFAULT_CONFIG)
    report = {"kind": "one_time_h0_candidate_disable_trial",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "execution_requested": execute, "disable_commands_sent": 0,
              "completed": False, "source_joint_rad": SOURCE_Q}
    path = _write_new(REPORT_DIR, "nero_h0_disable_trial_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            before = check_pre_disable(robot, read_state(robot), config)
            report.update(before_state=before, preflight_passed=True)
            print("现场轻触点关节角：", before["joint_rad"], flush=True)
            print("法兰 XYZ：", before["flange_pose_m_rad"][:3], flush=True)
            if not execute:
                print("只读预检完成；未发送失能或运动指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送失能指令")
                print(f"{remaining} 秒后发送一次七轴失能；按 Ctrl+C 取消。",
                      flush=True)
                time.sleep(1)
            live = check_pre_disable(robot, read_state(robot), config)
            if max(abs(a-b) for a, b in zip(before["joint_rad"],
                                             live["joint_rad"])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送失能指令")
            report["disable_commands_sent"] = 1
            report["disable_return"] = bool(robot.disable())
            time.sleep(0.5)
            after = read_state(robot)
            report["after_0_5s_state"] = after
            time.sleep(1.5)
            final = read_state(robot)
            report["after_2s_state"] = final
            report["max_joint_change_2s_rad"] = max(
                abs(a-b) for a, b in zip(final["joint_rad"],
                                           live["joint_rad"]))
            if any(d["enabled"] for d in final["drivers"]):
                raise RuntimeError("单次失能后仍有驱动使能位为 True")
            report["completed"] = True
            print("一次失能已发送；七轴使能反馈均为 False。", flush=True)
            print("2 秒最大单关节变化：",
                  report["max_joint_change_2s_rad"], flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"H0 失能验收未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("H0 失能验收报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        run(execute=args.run)
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
