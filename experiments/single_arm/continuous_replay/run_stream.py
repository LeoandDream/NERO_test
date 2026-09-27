"""按固定时钟发布 move_j 目标并逐周期保存实测反馈。仅供显式 --run 调用。"""

import csv
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time

from experiments.single_arm.continuous_replay.continuous_session import (
    CAN_IDS, FINAL_TOLERANCE_RAD,
    FLANGE_HARD_M, FLANGE_SOURCE_MARGIN_M, FLANGE_WARN_M,
    MAX_LATE_S, RUN_DIR, START_TOLERANCE_RAD, TRACKING_HARD_RAD,
    TRACKING_WARN_DURATION_S, TRACKING_WARN_RAD, configure_joint_mode,
    driver_snapshot, read_realtime, robot_instance,
    source_envelope, utc_now, verify_start, write_json_exclusive,
)
from experiments.single_arm.continuous_replay.trajectory import read_source
from experiments.single_arm.replay.can_trace import ReplayCanTrace
from experiments.single_arm.teaching.teach_session import validate_joints


class StreamStopped(RuntimeError):
    """连续目标已停止，报告中保留最后目标和实际姿态。"""


def _read_snapshot(robot):
    result = {"time_utc": utc_now()}
    for key, getter in (("status", robot.get_arm_status),
                        ("joints", robot.get_joint_angles),
                        ("flange", robot.get_flange_pose)):
        try:
            item = getter()
            if item is None:
                result[key] = None
            elif key == "status":
                result[key] = {
                    "arm_status": int(item.msg.arm_status),
                    "control_mode": int(item.msg.ctrl_mode),
                    "teach_status": int(item.msg.teach_status),
                    "motion_status": int(item.msg.motion_status),
                    "error_code": int(item.msg.err_code),
                    "feedback_unix_s": float(item.timestamp),
                }
            else:
                result[key] = [float(value) for value in item.msg]
                result[f"{key}_feedback_unix_s"] = float(item.timestamp)
        except Exception as exc:
            result[f"{key}_error"] = str(exc)
    return result


def _controlled_stop(robot, report):
    """正常反馈时以当前姿态下发保持目标；无法确认保持则阻尼急停。"""
    try:
        raw_status = robot.get_arm_status()
        if raw_status is not None and int(raw_status.msg.arm_status) == 1:
            report["stop_action"] = "controller_already_emergency_stopped"
            return
        status, current, _ = read_realtime(robot)
        if int(status.msg.motion_status) not in (0, 1):
            raise RuntimeError("控制器运动状态异常")
        robot.move_j(current)
        anchor = current
        recent = []
        for _ in range(10):
            time.sleep(0.1)
            status, latest, _ = read_realtime(robot)
            recent.append(latest)
            if max(abs(a-b) for a,b in zip(latest, anchor)) > 0.02:
                raise RuntimeError("保持目标下关节偏移超过 0.02 rad")
        if max(abs(a-b) for point in recent[-3:] for a,b in zip(point, recent[-1])) > 0.002:
            raise RuntimeError("保持目标下关节尚未停稳")
        report["stop_action"] = "move_j_hold_current_confirmed"
        report["hold_joint_rad"] = latest
    except Exception as exc:
        report["hold_error"] = str(exc)
        try:
            robot.electronic_emergency_stop()
            report["stop_action"] = "electronic_emergency_stop"
        except Exception as stop_exc:
            report["stop_action"] = "electronic_emergency_stop_failed"
            report["stop_error"] = str(stop_exc)


def run(plan, plan_path, config, robot_factory=None, *, source_reader=read_source,
        run_dir=RUN_DIR):
    """执行已复核的定时关节流；source_reader 仅决定源记录的校验格式。

    默认仍读取原来固定的 20 秒示教源。新封存记录可提供严格的读取器，
    两者共用相同的实时反馈、发布节拍和异常停止逻辑。
    """
    robot = (robot_factory or robot_instance)(config["channel"])
    signal_previous = None
    trace = None
    report_created = False
    motion_sent = False
    aborted = False
    stem = "nero_continuous_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    report_path = Path(run_dir) / f"{stem}.json"
    samples_path = Path(run_dir) / f"{stem}.csv"
    can_path = Path(run_dir) / f"{stem}.can.csv"
    report = {
        "schema_version": 1,
        "started_at_utc": utc_now(),
        "plan": str(plan_path),
        "recording": plan["recording"],
        "source_sha256": plan["source_sha256"],
        "source_used_rows": plan["source_used_rows"],
        "source_used_duration_s": plan["source_used_duration_s"],
        "planned_target_count": len(plan["waypoints"]),
        "planned_duration_s": plan["total_duration_s"],
        "frequency_hz": plan["frequency_hz"],
        "turnaround_hold_s": plan.get("turnaround_hold_s", 0.0),
        "recovery_mode": bool(plan.get("recovery_mode", False)),
        "one_way_mode": bool(plan.get("one_way_mode", False)),
        "recovery_parent_report": plan.get("recovery_parent_report"),
        "samples_csv": str(samples_path),
        "can_trace_csv": str(can_path),
        "last_published_index": -1,
        "last_attempted_index": -1,
        "completed": False,
        "driver_snapshots": [],
    }

    def on_signal(_number, _frame):
        nonlocal aborted
        aborted = True

    signal_previous = (signal.signal(signal.SIGINT, on_signal),
                       signal.signal(signal.SIGTERM, on_signal))
    try:
        write_json_exclusive(report_path, report)
        report_created = True
        robot.connect()
        actual_start = verify_start(robot, plan, config)
        report["actual_start_joint_rad"] = actual_start
        nominal_start = plan["waypoints"][0]["joint_rad"]
        offset = [a-b for a,b in zip(actual_start, nominal_start)]
        report["joint_plan_offset_rad"] = offset
        source = source_reader(Path(plan["recording"]), config)
        if source["sha256"] != plan["source_sha256"]:
            raise ValueError("连续源记录在执行前发生变化")
        envelope = source_envelope(source)
        # 依据本轮实际起点把整条计划等量平移，回程精确返回本轮起点。
        targets = []
        expected_xyz = []
        for item in plan["waypoints"]:
            joints = validate_joints(
                [a+b for a,b in zip(item["joint_rad"], offset)],
                config["zero_exclusion_radius_rad"],
            )
            pose = robot.fk(joints)
            if any(not (pair[0]-FLANGE_SOURCE_MARGIN_M <= value <= pair[1]+FLANGE_SOURCE_MARGIN_M)
                   for value, pair in zip(pose[:3], envelope)):
                raise RuntimeError("实际起点偏移后的目标超出源法兰包围盒外扩范围")
            targets.append(joints)
            expected_xyz.append([float(value) for value in pose[:3]])
        report["actual_plan_flange_xyz_range_m"] = {
            axis: [min(p[index] for p in expected_xyz), max(p[index] for p in expected_xyz)]
            for index, axis in enumerate("xyz")
        }
        trace = ReplayCanTrace(config["channel"], can_path, frame_ids=CAN_IDS, flush_every=20)
        trace.start()
        report["driver_snapshots"].append(driver_snapshot(robot))
        for remaining in range(5, 0, -1):
            if aborted:
                raise StreamStopped("倒计时取消")
            print(f"{remaining} 秒后开始连续目标；按 Ctrl+C 停止。", flush=True)
            time.sleep(1)
        configure_joint_mode(robot, config)
        if max(abs(a-b) for a,b in zip(actual_start, read_realtime(robot)[1])) > START_TOLERANCE_RAD:
            raise StreamStopped("切换关节模式后姿态变化超过 0.005 rad")
        trace.check()
        report["motion_started_at_utc"] = utc_now()
        fields = (
            "index", "phase", "scheduled_t_s", "actual_t_s", "lateness_s", "period_s",
            "command_started_utc", "command_returned_utc", "command_duration_s",
            "joint_feedback_unix_s", "flange_feedback_unix_s",
            "arm_status", "motion_status", "error_code", "tracking_max_rad",
            "flange_error_m", "joint_target_rad", "joint_feedback_rad",
            "flange_target_xyz_m", "flange_feedback_xyz_m",
        )
        samples_path.parent.mkdir(parents=True, exist_ok=True)
        with samples_path.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            stream.flush()
            started = time.monotonic() + 0.2
            last_sent_at = None
            last_driver_at = started - 1.0
            tracking_warn_since = None
            flange_warn_since = None
            periods, errors, flange_errors = [], [], []
            forward_sent_at = None
            turnaround_settle = deque(maxlen=max(10, plan["frequency_hz"] // 2))
            for index, (item, target, expected) in enumerate(zip(
                    plan["waypoints"], targets, expected_xyz)):
                scheduled = started + item["t_s"]
                while True:
                    if aborted:
                        raise StreamStopped("收到中止信号")
                    remaining = scheduled - time.monotonic()
                    if remaining <= 0:
                        break
                    time.sleep(min(remaining, 0.02))
                trace.check()
                status, current, flange, joint_stamp, flange_stamp = read_realtime(
                    robot, with_timestamps=True)
                if time.monotonic() - last_driver_at >= 1.0:
                    report["driver_snapshots"].append(driver_snapshot(robot))
                    last_driver_at = time.monotonic()
                now = time.monotonic()
                lateness = now - scheduled
                if lateness > MAX_LATE_S:
                    raise StreamStopped(f"目标周期迟到 {lateness:.3f} s 超过 0.10 s")
                error = max(abs(a-b) for a,b in zip(target,current))
                flange_error = math.dist(expected, flange[:3])
                if error > TRACKING_HARD_RAD or flange_error > FLANGE_HARD_M:
                    raise StreamStopped("跟踪误差超过即时停止门槛")
                if error > TRACKING_WARN_RAD:
                    tracking_warn_since = now if tracking_warn_since is None else tracking_warn_since
                else:
                    tracking_warn_since = None
                if flange_error > FLANGE_WARN_M:
                    flange_warn_since = now if flange_warn_since is None else flange_warn_since
                else:
                    flange_warn_since = None
                if ((tracking_warn_since is not None and now - tracking_warn_since >= TRACKING_WARN_DURATION_S)
                        or (flange_warn_since is not None and now - flange_warn_since >= TRACKING_WARN_DURATION_S)):
                    raise StreamStopped("跟踪误差持续超过停止门槛")
                if item["phase"] == "hold":
                    # 连续发布同一 move_j 目标时，控制器的运动状态可在
                    # 运行/到位之间交替；以实际关节反馈的误差和变化量判定停稳。
                    if error <= FINAL_TOLERANCE_RAD:
                        turnaround_settle.append(current)
                    else:
                        turnaround_settle.clear()
                    if (index + 1 == len(plan["waypoints"])
                            or plan["waypoints"][index + 1]["phase"] != "hold"):
                        report["turnaround_settle_periods"] = len(turnaround_settle)
                        variation = max(
                            max(point[joint] for point in turnaround_settle)
                            - min(point[joint] for point in turnaround_settle)
                            for joint in range(7)
                        ) if turnaround_settle else float("inf")
                        report["turnaround_hold_variation_rad"] = variation
                        if (len(turnaround_settle) < turnaround_settle.maxlen
                                or variation > 0.002):
                            raise StreamStopped("源前缀终点关节反馈未连续停稳至少 0.5 秒，不反向")
                        report["turnaround_hold_confirmed"] = True
                        report["turnaround_hold_joint_rad"] = current
                        report["turnaround_hold_flange_pose"] = flange
                report["last_attempted_index"] = index
                report["last_attempted_target_joint_rad"] = target
                command_started_utc = utc_now()
                motion_sent = True
                robot.move_j(target)
                returned_at = time.monotonic()
                command_returned_utc = utc_now()
                report["last_published_index"] = index
                report["last_published_target_joint_rad"] = target
                period = None if last_sent_at is None else returned_at-last_sent_at
                if period is not None:
                    periods.append(period)
                last_sent_at = returned_at
                errors.append(error)
                flange_errors.append(flange_error)
                if item["phase"] == "return" and forward_sent_at is None:
                    forward_sent_at = now
                    report["forward_actual_duration_s"] = now-started
                writer.writerow({
                    "index": index, "phase": item["phase"],
                    "scheduled_t_s": f"{item['t_s']:.6f}",
                    "actual_t_s": f"{returned_at-started:.6f}", "lateness_s": f"{lateness:.6f}",
                    "period_s": "" if period is None else f"{period:.6f}",
                    "command_started_utc": command_started_utc,
                    "command_returned_utc": command_returned_utc,
                    "command_duration_s": f"{returned_at-now:.6f}",
                    "joint_feedback_unix_s": f"{joint_stamp:.6f}",
                    "flange_feedback_unix_s": f"{flange_stamp:.6f}",
                    "arm_status": int(status.msg.arm_status),
                    "motion_status": int(status.msg.motion_status),
                    "error_code": int(status.msg.err_code),
                    "tracking_max_rad": f"{error:.6f}",
                    "flange_error_m": f"{flange_error:.6f}",
                    "joint_target_rad": json.dumps(target),
                    "joint_feedback_rad": json.dumps(current),
                    "flange_target_xyz_m": json.dumps(expected),
                    "flange_feedback_xyz_m": json.dumps(flange[:3]),
                })
                if index % 20 == 0:
                    stream.flush()
                if index % 20 == 0 or index == len(targets)-1:
                    print(f"连续进度 {index+1}/{len(targets)}；"
                          f"最大单关节跟踪误差 {error:.4f} rad", flush=True)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                trace.check()
                status, current, _ = read_realtime(robot)
                final_target = targets[-1] if (plan.get("recovery_mode") or plan.get("one_way_mode")) else actual_start
                if max(abs(a-b) for a,b in zip(current, final_target)) <= FINAL_TOLERANCE_RAD:
                    break
                time.sleep(0.05)
            else:
                raise StreamStopped("已发送全部目标，但未在 5 秒内到达最终目标")
            report["final_joint_rad"] = current
            report["max_return_joint_error_rad"] = max(abs(a-b) for a,b in zip(current, final_target))
            report["max_tracking_error_rad"] = max(errors)
            report["max_flange_error_m"] = max(flange_errors)
            sorted_periods = sorted(periods)
            report["period_p99_s"] = sorted_periods[math.ceil(0.99*len(sorted_periods))-1]
            report["period_max_s"] = max(periods)
            report["actual_motion_duration_s"] = time.monotonic()-started
            if forward_sent_at is not None:
                report["return_actual_duration_s"] = time.monotonic()-forward_sent_at
            report["driver_snapshots"].append(driver_snapshot(robot))
            # SocketCAN 旁路监听在另一线程读取；最后一批帧可能晚于 SDK
            # move_j 返回。等待监听追平后再关闭，否则会丢失最后一组证据。
            frame_ids = (0x155, 0x156, 0x157, 0x170)
            trace_deadline = time.monotonic() + 1.0
            while (any(trace.id_counts[frame_id] < len(targets) for frame_id in frame_ids)
                   and time.monotonic() < trace_deadline):
                trace.check()
                time.sleep(0.01)
            report["joint_command_frame_counts"] = {
                f"0x{frame_id:03X}": trace.id_counts[frame_id] for frame_id in frame_ids
            }
            report["can_joint_frames_complete"] = all(
                trace.id_counts[frame_id] >= len(targets) for frame_id in frame_ids
            )
            report["acceptance_period_pass"] = report["period_p99_s"] <= 0.08
            report["completed"] = True
            label = ("单向命名起点转移" if plan.get("one_way_mode") else
                     "连续原路回位" if plan.get("recovery_mode") else "连续往返")
            print(f"{label}完成；最终最大关节误差 {report['max_return_joint_error_rad']:.6f} rad。", flush=True)
    except Exception as exc:
        report["error"] = str(exc)
        report["failure_snapshot"] = _read_snapshot(robot)
        if motion_sent:
            _controlled_stop(robot, report)
        raise
    finally:
        if trace is not None:
            try:
                trace.stop()
            except Exception as exc:
                report["can_trace_stop_error"] = str(exc)
            report["can_trace_frames"] = trace.count
        try:
            robot.disconnect()
        except Exception as exc:
            report["disconnect_error"] = str(exc)
        if report_created:
            report["finished_at_utc"] = utc_now()
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
            print("连续回放报告：", report_path, flush=True)
        if signal_previous is not None:
            signal.signal(signal.SIGINT, signal_previous[0])
            signal.signal(signal.SIGTERM, signal_previous[1])
    return report_path
