"""NERO 只读来源与离线资产预览入口；没有真实运动命令。"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import time

from .acquisition import collect_snapshot
from .device import DeviceIOError, connected_nero, validate_channel
from .environment import format_environment_profile, load_environment_profile
from .motion import prepare_joint_route
from .recording import (
    RecordingParameters, format_recording_summary, list_recordings,
    load_recording_summary, record_trajectory, recording_paths,
)
from .replay import format_replay_plan, prepare_replay
from .waypoints import (
    capture_waypoint, list_waypoints, load_waypoint, save_waypoint,
    validate_name,
)


SDK_SOURCE = "pyAgxArm@841a625f Nero V121 CAN cached feedback"


class RecordingFailed(RuntimeError):
    """录制已保存 fault 摘要；CLI 必须返回非零。"""


def parser_create() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "READ-ONLY DEVICE (explicit connection; real feedback not field-validated):\n"
            "  status | waypoint capture | trajectory record\n\n"
            "OFFLINE (local files only):\n"
            "  waypoint show/list | route show | trajectory show/list | replay show | environment show\n\n"
            "LOCKED / NOT PROMOTED:\n"
            "  motion, drag, teach, replay execution, start, park, return, disable, reset"
        ))
    groups = parser.add_subparsers(dest="command", required=True)
    status = groups.add_parser("status", help="显式连接并有限等待状态反馈")
    point = groups.add_parser("waypoint", help="单次关节点位观测记录")
    point_groups = point.add_subparsers(dest="action", required=True)
    capture = point_groups.add_parser("capture", help="读取并保存一个命名观测点")
    point_groups.add_parser("show", help="离线展示已保存点位").add_argument("path", type=Path)
    point_groups.add_parser("list", help="离线列举目录中的点位").add_argument("directory", type=Path)
    route = groups.add_parser("route", help="仅离线查看调用者指定的关节路线")
    route_groups = route.add_subparsers(dest="action", required=True)
    route_show = route_groups.add_parser("show", help="按给定顺序查看起点及目标点位 JSON")
    route_show.add_argument("expected_start", type=Path)
    route_show.add_argument("targets", type=Path, nargs="+")
    trajectory = groups.add_parser("trajectory", help="独立只读轨迹记录及离线摘要")
    trajectory_groups = trajectory.add_subparsers(dest="action", required=True)
    record = trajectory_groups.add_parser("record", help="显式连接并流式记录实际反馈")
    record.add_argument("--recording-id", required=True)
    record.add_argument("--output-directory", type=Path, required=True)
    record.add_argument("--duration-seconds", type=float, required=True)
    record.add_argument("--rate-hz", type=float, required=True)
    trajectory_groups.add_parser("show", help="离线展示并核验一条记录").add_argument("path", type=Path)
    trajectory_groups.add_parser("list", help="离线列举记录摘要").add_argument("directory", type=Path)
    replay = groups.add_parser("replay", help="仅离线检查关节路点回放候选")
    replay_groups = replay.add_subparsers(dest="action", required=True)
    replay_groups.add_parser("show", help="核对记录并展示回放候选").add_argument("path", type=Path)
    environment = groups.add_parser("environment", help="仅离线展示环境 Profile")
    environment_groups = environment.add_subparsers(dest="action", required=True)
    environment_groups.add_parser("show", help="核对 Profile 和全部点位原件").add_argument("path", type=Path)
    capture.add_argument("--name", required=True)
    capture.add_argument("--output", type=Path, required=True)
    for command in (status, capture):
        command.add_argument("--wait-seconds", type=float, default=2.0,
                             help="连接后等待必需反馈的最长时间；默认 2 秒")
    for command in (status, capture, record):
        command.add_argument("--channel", default="can0")
        command.add_argument("--robot-id", default=None,
                             help="调用者确认的机器人身份；不填则记录未知")
        command.add_argument("--max-age-seconds", type=float, default=1.0,
                             help="各来源反馈最大年龄，默认 1 秒；不证明多帧同步")
    return parser


def _source_time(value: float | None, basis: str) -> str:
    if value is None:
        return "缺失"
    if basis == "unix_epoch_s":
        try:
            return datetime.fromtimestamp(value, timezone.utc).isoformat()
        except (OSError, OverflowError, ValueError):
            pass
    return f"{value:.6f} s ({basis})"


def _quality_line(label: str, quality: dict, basis: str) -> str:
    age = quality["age_s"]
    age_text = "未知" if age is None else f"{age:.3f} s"
    return (f"{label}质量：{quality['state']}；来源时间："
            f"{_source_time(quality['timestamp_s'], basis)}；年龄：{age_text}")


def format_snapshot(snapshot) -> str:
    """只展示设备事实和缺项，不给 READY、停稳或运动许可结论。"""
    data = asdict(snapshot) if not isinstance(snapshot, dict) else snapshot
    basis = data["time_basis"]
    lines = [
        f"机器人身份：{data['robot_id'] or '未知'}",
        f"采集观察时间：{_source_time(data['observed_at_s'], basis)}",
        f"时基：{basis}",
        f"控制模式/整机状态/错误码：{data['control_mode']} / "
        f"{data['arm_status']} / {data['error_code']}（原始事实，非运动许可）",
        _quality_line("控制器", data["status_quality"], basis),
        _quality_line("七轴", data["joints_quality"], basis),
        _quality_line("法兰", data["flange_quality"], basis),
    ]
    joints = data["joint_rad"]
    if joints is None:
        lines.append("七轴角：缺失或无效，不能捕获七轴点位")
    else:
        lines.append("七轴角 (rad)：" + ", ".join(
            f"J{index}={value:.6f}" for index, value in enumerate(joints, 1)))
    flange = data["flange_pose_m_rad"]
    if flange is None:
        lines.append("法兰位姿：缺失或无效；不影响有效关节点的记录")
    else:
        lines.append("法兰位置 (m)：" + ", ".join(f"{value:.6f}" for value in flange[:3]))
        lines.append("法兰姿态 (rad)：" + ", ".join(f"{value:.6f}" for value in flange[3:]))
        lines.append("法兰坐标系：未在本功能中标定；此位姿不是 TCP")
    def flag(value):
        return "未知" if value is None else ("是" if value else "否")

    for item in data["drivers"]:
        lines.append(
            f"J{item['joint']} 驱动：enabled={flag(item['enabled'])}；"
            f"undervoltage={flag(item['undervoltage'])}；"
            f"driver_error={flag(item['driver_error'])}；"
            f"质量={item['quality']['state']}")
    missing_drivers = [str(item["joint"]) for item in data["drivers"]
                       if item["quality"]["state"] != "valid"]
    if missing_drivers:
        lines.append("驱动状态缺项/无效关节：" + ", ".join(missing_drivers))
    if data["issues"]:
        lines.append("来源问题：" + "；".join(data["issues"]))
    lines.append("SDK 多帧缓存的组合时间戳不证明七轴同步更新、停稳或安全。")
    return "\n".join(lines)


def format_waypoint(record: dict, path: str | Path) -> str:
    return "\n".join([
        f"点位名：{record['name']}（观测记录，非 READY/PARK 或运动许可）",
        f"文件：{Path(path).resolve()}",
        f"来源：{record['source']}",
        f"记录时刻：{_source_time(record['captured_at_s'], record['time_basis'])}",
        f"工具身份/法兰坐标系：{record['tool_id'] or '未知'} / "
        f"{record['flange_frame'] or '未知'}",
        format_snapshot(record["snapshot"]),
        "读取异常：" + ("；".join(record["read_errors"]) or "无"),
    ])


def format_route(route) -> str:
    lines = ["关节路线数据预览；不是碰撞检查、现场许可或已执行结果。",
             f"目标数：{len(route.targets)}；单位：rad；顺序：J1～J7"]
    previous = route.expected_start
    for index, point in enumerate((previous, *route.targets)):
        label = "预期起点" if index == 0 else f"目标 {index}"
        lines.append(f"{label}：{point.name or '未命名'}；来源={point.source}；"
                     f"机器人={point.robot_id or '未知'}；工具={point.tool_id or '未知'}；"
                     f"法兰坐标系={point.flange_frame or '未知'}")
        lines.append("七轴角 (rad)：" + ", ".join(
            f"J{joint}={value:.6f}" for joint, value in enumerate(point.joint_rad, 1)))
        if index:
            lines.append("与前点逐轴差 (rad)：" + ", ".join(
                f"J{joint}={value - before:+.6f}"
                for joint, (before, value) in enumerate(
                    zip(previous.joint_rad, point.joint_rad), 1)))
            previous = point
    return "\n".join(lines)


def _validate_read_args(args) -> None:
    validate_channel(args.channel)
    if args.robot_id is not None and (not args.robot_id.strip() or
                                      args.robot_id != args.robot_id.strip()):
        raise ValueError("robot_id must be a non-empty confirmed identity")
    if (not math.isfinite(args.max_age_seconds) or
            args.max_age_seconds <= 0 or args.max_age_seconds > 60):
        raise ValueError("max-age-seconds must be finite and within (0, 60]")


def _collect(source, args, clock_s):
    return collect_snapshot(
        source, robot_id=args.robot_id, clock_s=clock_s,
        time_basis="unix_epoch_s",
        max_status_age_s=args.max_age_seconds,
        max_joints_age_s=args.max_age_seconds,
        max_flange_age_s=args.max_age_seconds,
        max_driver_age_s=args.max_age_seconds,
    )


def _collect_until(source, args, clock_s, monotonic_s, sleep_s, *, status: bool):
    """重复单轮采集至必需反馈有效或截止，返回最后一轮部分结果。"""
    deadline = monotonic_s() + args.wait_seconds
    while True:
        result = _collect(source, args, clock_s)
        snap = result.snapshot
        if (snap.joints_quality.state == "valid" and
                (not status or snap.status_quality.state == "valid")):
            return result
        remaining = deadline - monotonic_s()
        if remaining <= 0:
            return result
        sleep_s(min(0.05, remaining))


def dispatch(args, *, source_factory=None, clock_s=time.time,
             monotonic_s=time.monotonic, sleep_s=time.sleep,
             source_label: str | None = None) -> str:
    """组合现有 Runtime；只在 status/capture/trajectory record 打开设备来源。"""
    if args.command == "route" and args.action == "show":
        return format_route(prepare_joint_route(args.expected_start, args.targets))
    if args.command == "replay" and args.action == "show":
        return format_replay_plan(prepare_replay(args.path))
    if args.command == "environment" and args.action == "show":
        return format_environment_profile(load_environment_profile(args.path))
    if args.command == "trajectory" and args.action == "show":
        return format_recording_summary(load_recording_summary(args.path), args.path)
    if args.command == "trajectory" and args.action == "list":
        entries = list_recordings(args.directory)
        if not entries:
            return f"目录中无轨迹摘要：{args.directory.resolve()}"
        return "\n".join(
            (f"无效记录：{entry['path'].resolve()}；{entry['error']}"
             if entry["error"] else
             f"{entry['summary']['recording_id']}；"
             f"样本={entry['summary']['sample_count']}；"
             f"结束={entry['summary']['stop_reason']}；"
             f"摘要={entry['path'].resolve()}")
            for entry in entries)
    if args.command == "waypoint" and args.action == "show":
        return format_waypoint(load_waypoint(args.path), args.path)
    if args.command == "waypoint" and args.action == "list":
        entries = list_waypoints(args.directory)
        if not entries:
            return f"目录中无 JSON 点位：{args.directory.resolve()}"
        lines = []
        for entry in entries:
            path = entry["path"].resolve()
            if entry["error"]:
                lines.append(f"无效文件：{path}；{entry['error']}")
            else:
                record = entry["record"]
                quality = record["snapshot"]["joints_quality"]["state"]
                lines.append(f"{record['name']}；七轴质量={quality}；"
                             f"来源时间={_source_time(record['snapshot']['joints_quality']['timestamp_s'], record['time_basis'])}；"
                             f"文件={path}")
        return "\n".join(lines)

    if args.command == "trajectory" and args.action == "record":
        parameters = RecordingParameters(args.duration_seconds, args.rate_hz)
        _, summary_path = recording_paths(args.output_directory, args.recording_id)
        _validate_read_args(args)
        opener = connected_nero if source_factory is None else source_factory
        with opener(args.channel) as source:
            summary = record_trajectory(
                recording_id=args.recording_id,
                output_directory=args.output_directory, parameters=parameters,
                read_snapshot=lambda: _collect(source, args, clock_s),
                monotonic_s=monotonic_s, sleep_s=sleep_s,
                cancel_requested=lambda: False, finish_requested=lambda: False,
                wall_clock_s=clock_s)
        result = format_recording_summary(summary, summary_path)
        if summary["stop_reason"] == "cancelled":
            raise KeyboardInterrupt(f"轨迹记录已保存：{summary_path}")
        if summary["stop_reason"] == "fault":
            raise RecordingFailed(result)
        return result

    _validate_read_args(args)
    if (not math.isfinite(args.wait_seconds) or
            not 0 <= args.wait_seconds <= 10):
        raise ValueError("wait-seconds must be finite and within [0, 10]")
    if args.command == "waypoint":
        validate_name(args.name)
        if args.output.exists() or args.output.is_symlink():
            raise FileExistsError(f"waypoint file already exists: {args.output}")
        if args.output.parent.exists() and not args.output.parent.is_dir():
            raise ValueError(f"waypoint output parent is not a directory: {args.output.parent}")

    opener = connected_nero if source_factory is None else source_factory
    label = source_label if source_label is not None else (
        SDK_SOURCE if source_factory is None else "injected feedback source")
    with opener(args.channel) as source:
        result = _collect_until(source, args, clock_s, monotonic_s, sleep_s,
                                status=args.command == "status")
        if args.command == "status":
            snap = result.snapshot
            missing = [label for label, quality in (
                ("控制器", snap.status_quality), ("七轴", snap.joints_quality))
                if quality.state != "valid"]
            tail = ("\n等候截止，必需反馈未齐：" + "、".join(missing)) if missing else ""
            return format_snapshot(snap) + tail + "\n读取异常：" + (
                "；".join(result.read_errors) or "无")
        record = capture_waypoint(args.name, result.snapshot,
                                  source=f"{label}; channel={args.channel}",
                                  read_errors=result.read_errors)
    path = save_waypoint(record, args.output)
    return format_waypoint(record, path)


def main(argv=None, *, source_factory=None, clock_s=time.time,
         monotonic_s=time.monotonic, sleep_s=time.sleep,
         source_label: str | None = None) -> int:
    args = parser_create().parse_args(argv)
    try:
        print(dispatch(args, source_factory=source_factory, clock_s=clock_s,
                       monotonic_s=monotonic_s, sleep_s=sleep_s,
                       source_label=source_label))
        return 0
    except KeyboardInterrupt as exc:
        detail = f"；{exc}" if str(exc) else ""
        print(f"读取已取消{detail}；未发送控制指令。", file=sys.stderr)
        return 130
    except RecordingFailed as exc:
        print(f"轨迹记录失败，已保存摘要：\n{exc}", file=sys.stderr)
        return 1
    except (DeviceIOError, OSError, ValueError) as exc:
        cleanup = exc.__cause__
        detail = (f"；关闭通信失败：{type(cleanup).__name__}: {cleanup}"
                  if isinstance(cleanup, (OSError, RuntimeError)) and
                  not isinstance(exc, DeviceIOError) else "")
        label = ("环境预览" if args.command == "environment" else
                 "Replay candidate: no；回放候选" if args.command == "replay" else
                 "路线预览" if args.command == "route" else
                 "轨迹记录" if args.command == "trajectory" else "状态/点位")
        print(f"{label}操作未完成：{exc}{detail}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
