#!/usr/bin/env python3
"""沿完整示教记录正向回放，再沿相同关节路线返回实际起点。"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.replay.can_trace import ReplayCanTrace
from experiments.single_arm.start_transfer.go_to_start import (
    PLAN_STEP_RAD, RoutePaused, execute_route, flange_envelope, stable_joint_feedback,
)
from experiments.single_arm.teaching.return_session import load_recording
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, simplify_path, validate_joints, validate_route_workspace,
)


RECORDING = Path("experiments/single_arm/teaching/data/recordings/nero_session_20260923T145952Z.csv")
START_TOLERANCE_RAD = 0.01
FINAL_TOLERANCE_RAD = 0.005
REPLAY_TARGET_TOLERANCE_RAD = 0.005
REPLAY_MAX_WAYPOINT_SECONDS = 15.0


def build_cycle(recorded, actual_start, original, config):
    """将完整记录简化并插值，再用同一组点的逆序组成闭合往返路线。"""
    actual_start = validate_joints(actual_start, config["zero_exclusion_radius_rad"])
    if max(abs(a - b) for a, b in zip(actual_start, original)) > START_TOLERANCE_RAD:
        raise ValueError("当前位置未贴近该次示教的真实起点 S1")
    forward_raw = [actual_start, *recorded]
    if any(math.dist(a, b) > config["max_recorded_step_rad"]
           for a, b in zip(forward_raw, forward_raw[1:])):
        raise ValueError("示教记录不连续，拒绝回放")
    # 简化只删去近似共线点，随后按运动小步上限重新插值，保留原轨迹转折。
    simplified = simplify_path(forward_raw, config["return_path_tolerance_rad"])
    forward = [actual_start]
    for target in simplified[1:]:
        previous = forward[-1]
        segments = max(1, math.ceil(math.dist(previous, target) / PLAN_STEP_RAD))
        for index in range(1, segments + 1):
            fraction = index / segments
            point = [a + fraction * (b - a) for a, b in zip(previous, target)]
            forward.append(validate_joints(point, config["zero_exclusion_radius_rad"]))
    forward[-1] = list(recorded[-1])
    # 去程和回程共享同一关节折线，末端点只保留一次以避免重复指令。
    cycle = [*forward, *reversed(forward[:-1])]
    if len(cycle) > 2500:
        raise ValueError("回放路线点数过多")
    return cycle, len(forward) - 1


def verify_return(robot, target):
    """等待关节反馈收敛到本轮实际起点并计算最大单关节误差。"""
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        current = joint_point(robot)
        error = max(abs(a - b) for a, b in zip(current, target))
        if error <= FINAL_TOLERANCE_RAD:
            return current, error
        time.sleep(0.1)
    raise RuntimeError(f"已完成回放，但返回实际起点误差 {error:.4f} rad 超出要求")


def main():
    """按指定轮数预检、回放和保存执行报告；默认仅规划。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, default=RECORDING)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--cycles", type=int, default=1, help="连续完成的回放往返轮数，默认 1")
    parser.add_argument("--speed-percent", type=int, choices=(5, 10), default=5,
                        help="关节运动速度百分比；默认 5，10 须先做只读规划和现场核对")
    parser.add_argument("--run", action="store_true", help="显式执行运动；默认只读规划")
    args = parser.parse_args()
    if not 1 <= args.cycles <= 5:
        parser.error("--cycles 必须在 1 到 5 之间")
    try:
        config = load_config(args.config)
        _, metadata, original, recorded = load_recording(args.recording, config, allow_completed=True)
        if not metadata.get("return_completed") or metadata.get("stop_reason") != "time_limit":
            raise ValueError("只允许回放已完成原路返回的完整限时示教记录")
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, lambda item: fresh(item), timeout=3) is None:
                raise RuntimeError("机械臂状态反馈缺失")
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            current = stable_joint_feedback(robot)
            route, forward_steps = build_cycle(recorded, current, original, config)
            validate_route_workspace(robot, route, config)
            envelope = flange_envelope(robot, route, config)
            print(f"记录 {len(recorded)} 条；单轮正向 {forward_steps} 小步，往返 {len(route)-1} 小步。")
            print("法兰中心沿途范围 (m)：", envelope)
            waypoint_timeout = float(config["return_waypoint_timeout_s"])
            if waypoint_timeout > REPLAY_MAX_WAYPOINT_SECONDS:
                raise ValueError("回放单步基础超时超过 15 秒硬上限")
            print(f"计划轮数：{args.cycles}；速度：{args.speed_percent}%。")
            print(f"整轮时限：{config['return_max_seconds']} 秒；每步基础/进展等待上限："
                  f"{waypoint_timeout:g}/{REPLAY_MAX_WAYPOINT_SECONDS:g} 秒。")
            if not args.run:
                print("只读规划完成；未发送运动指令。")
                return 0

            aborted = False

            def on_signal(_signum, _frame):
                nonlocal aborted
                aborted = True

            previous_int = signal.signal(signal.SIGINT, on_signal)
            previous_term = signal.signal(signal.SIGTERM, on_signal)
            output = Path("experiments/single_arm/replay/data/replays") / (
                "nero_replay_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            report = {
                "source_recording": str(args.recording),
                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                "requested_cycles": args.cycles,
                "completed_cycles": 0,
                "speed_percent": args.speed_percent,
                "cycle_time_limit_s": config["return_max_seconds"],
                "waypoint_timeout_s": waypoint_timeout,
                "waypoint_max_timeout_s": REPLAY_MAX_WAYPOINT_SECONDS,
                "target_tolerance_rad": REPLAY_TARGET_TOLERANCE_RAD,
                "flange_xyz_envelope_m": envelope,
                "cycles": [],
            }
            report_created = False
            trace = None
            try:
                with output.open("x", encoding="utf-8") as stream:
                    stream.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
                report_created = True
                # 在倒计时前启动只读 SocketCAN 旁路监听，覆盖第一个目标帧。
                # 监听失败时不进入运动阶段；CSV 只记录总线上实际观察到的帧。
                trace_path = output.with_suffix(".can.csv")
                trace = ReplayCanTrace(config["channel"], trace_path)
                trace.start()
                report["can_trace_csv"] = str(trace_path)
                for remaining in range(5, 0, -1):
                    if aborted:
                        raise InterruptedError("已取消回放")
                    print(f"{remaining} 秒后开始；按 Ctrl+C 取消。", flush=True)
                    time.sleep(1)
                for number in range(1, args.cycles + 1):
                    if aborted:
                        raise InterruptedError("已取消后续回放")
                    healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                    check_drivers(robot)
                    current = stable_joint_feedback(robot)
                    route, forward_steps = build_cycle(recorded, current, original, config)
                    validate_route_workspace(robot, route, config)
                    flange_envelope(robot, route, config)
                    attempt = {
                        "cycle": number,
                        "started_at_utc": datetime.now(timezone.utc).isoformat(),
                        "actual_start_joint_rad": current,
                        "forward_steps": forward_steps,
                        "total_steps": len(route) - 1,
                        "last_reached_step": 0,
                        "last_sent_step": 0,
                        "last_sdk_publish_completed_step": 0,
                        # 保留末尾发送时序，供 CAN 原始帧与失败小步逐一对齐。
                        "recent_command_events": [],
                        "stationary_target_retries": [],
                        "emergency_stop_requested": False,
                        "completed": False,
                    }
                    report["cycles"].append(attempt)
                    try:
                        print(f"开始第 {number}/{args.cycles} 轮。", flush=True)
                        trace.check()

                        def reached_waypoint(index, _target):
                            attempt["last_reached_step"] = index
                            # 仅在小步已到位的边界处理监听故障，避免旁路记录器
                            # 故障本身在机械臂运动中触发阻尼急停。
                            try:
                                trace.check()
                            except RuntimeError as exc:
                                raise RoutePaused(str(exc), reason="can_trace_failed") from exc

                        def command_started(index, target):
                            attempt["last_sent_step"] = index
                            attempt["last_sent_target_joint_rad"] = list(target)
                            events = attempt["recent_command_events"]
                            events.append({
                                "step": index,
                                "kind": "first_send",
                                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                                "target_joint_rad": list(target),
                            })
                            del events[:-8]

                        def command_published(index, _target):
                            attempt["last_sdk_publish_completed_step"] = index
                            attempt["recent_command_events"][-1]["sdk_returned_at_utc"] = (
                                datetime.now(timezone.utc).isoformat()
                            )

                        def stationary_retry(index, retry):
                            attempt["stationary_target_retries"].append({
                                "step": index, "retry": retry,
                            })
                            events = attempt["recent_command_events"]
                            events.append({
                                "step": index,
                                "kind": "same_target_retry",
                                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                                "target_joint_rad": list(route[index]),
                            })
                            del events[:-8]

                        # 整轮时限只在小步确认到位后检查。不要把时间到期伪装成
                        # 关节未到位故障并因此触发阻尼急停。
                        cycle_deadline = time.monotonic() + config["return_max_seconds"]
                        execute_route(
                            robot, config, route, lambda: aborted,
                            # 每到达一个小步就记下进度；失败报告可定位最后成功点。
                            on_waypoint=reached_waypoint,
                            on_command=command_started,
                            on_command_published=command_published,
                            on_stationary_retry=stationary_retry,
                            on_emergency_stop=lambda reason: attempt.update(
                                emergency_stop_requested=True,
                                emergency_stop_reason=reason,
                            ),
                            on_feedback=lambda joints: attempt.update(
                                last_feedback_joint_rad=list(joints)
                            ),
                            on_status=lambda status: attempt.update(
                                last_motion_status=str(status.msg.motion_status),
                                last_arm_status=int(status.msg.arm_status),
                                last_status_timestamp_s=float(status.timestamp),
                            ),
                            stop_after_waypoint=lambda index, _target: (
                                index < len(route) - 1 and time.monotonic() >= cycle_deadline
                            ),
                            target_tolerance_rad=REPLAY_TARGET_TOLERANCE_RAD,
                            speed_percent=args.speed_percent,
                            waypoint_timeout_s=waypoint_timeout,
                            waypoint_max_timeout_s=REPLAY_MAX_WAYPOINT_SECONDS,
                            pause_on_stationary_timeout=True,
                            stationary_retry_limit=1,
                        )
                        final, error = verify_return(robot, current)
                        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                        check_drivers(robot)
                        attempt["final_joint_rad"] = final
                        attempt["max_joint_error_rad"] = error
                        attempt["completed"] = True
                        report["completed_cycles"] += 1
                        print(f"第 {number} 轮已回到起点，最大关节误差 {error:.6f} rad。", flush=True)
                    except RoutePaused as exc:
                        attempt["error"] = str(exc)
                        attempt["pause_reason"] = exc.reason
                        # 当前处于中途目标；报告姿态供现场核对，不自动回起点。
                        try:
                            attempt["joint_at_pause_rad"] = joint_point(robot)
                        except Exception as feedback_exc:
                            attempt["joint_at_pause_error"] = str(feedback_exc)
                        raise
                    except Exception as exc:
                        attempt["error"] = str(exc)
                        if hasattr(exc, "elapsed_s"):
                            attempt["waypoint_elapsed_s"] = exc.elapsed_s
                            attempt["waypoint_best_error_rad"] = exc.best_error_rad
                            attempt["waypoint_last_error_rad"] = exc.last_error_rad
                        # execute_route 已处理运行中的急停；这里只读一次故障后姿态。
                        # 若 CAN 反馈也失效，不覆盖导致运动中止的原始异常。
                        try:
                            attempt["joint_after_error_rad"] = joint_point(robot)
                        except Exception as feedback_exc:
                            attempt["joint_after_error_error"] = str(feedback_exc)
                        raise
                    finally:
                        attempt["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            finally:
                if trace is not None:
                    try:
                        trace.stop()
                    except Exception as trace_exc:
                        report["can_trace_stop_error"] = str(trace_exc)
                    finally:
                        report["can_trace_frames"] = trace.count
                if report_created:
                    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                signal.signal(signal.SIGINT, previous_int)
                signal.signal(signal.SIGTERM, previous_term)
                if report_created:
                    print(f"回放报告：{output}", flush=True)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError, can.CanError) as exc:
        print(f"回放未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
