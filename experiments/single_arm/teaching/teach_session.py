#!/usr/bin/env python3
"""限时 CAN 拖动示教，并沿已记录轨迹返回示教前的非零位置。"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

from experiments.single_arm.can_setup.can_drag_teach import MOUNT_CODES, fresh, send_frame, wait_status
from experiments.single_arm.pose_recording.record_poses import FIELDNAMES, JOINT_COLUMNS, POSE_COLUMNS, STATUS_COLUMNS


DEFAULT_CONFIG = Path("experiments/single_arm/config/nero_teach.json")
JOINT_LIMITS = [ROBOT_JOINT_LIMIT_PRESET_RAD["nero"][f"joint{i}"] for i in range(1, 8)]
JOINT_LIMIT_MARGIN_RAD = 0.01
# 各关节独立限位仍要保留；法兰中心在允许区域不代表关节和其他连杆安全。
# 0x06 是固件退出示教轨迹时保留的“终止执行”反馈；仅在 CAN 控制模式下接受。
CAN_IDLE_TEACH_STATUSES = {0, 2, 6}


def validate_joints(joints, zero_radius):
    """校验七关节数值、SDK 限位余量和全零位排除半径；失败即拒绝运动。"""
    if len(joints) != 7 or not all(math.isfinite(float(value)) for value in joints):
        raise ValueError("必须提供七个有限的关节角")
    values = [float(value) for value in joints]
    if math.dist(values, [0.0] * 7) < zero_radius:
        raise ValueError("关节位置落入全零位禁用范围")
    for index, (value, (lower, upper)) in enumerate(zip(values, JOINT_LIMITS), 1):
        allowed_lower = lower + JOINT_LIMIT_MARGIN_RAD
        allowed_upper = upper - JOINT_LIMIT_MARGIN_RAD
        if not allowed_lower <= value <= allowed_upper:
            raise ValueError(
                f"关节 {index} 当前 {value:.6f} rad 超出带余量的 SDK 限位 "
                f"[{allowed_lower:.6f}, {allowed_upper:.6f}] rad；"
                f"SDK 原始限位 [{lower:.6f}, {upper:.6f}] rad"
            )
    return values


def load_config(path):
    """加载 S1、安装方向、示教和回位阈值，并拒绝不完整或无效配置。"""
    config = json.loads(path.read_text(encoding="utf-8"))
    if config["mount"] not in MOUNT_CODES:
        raise ValueError("配置中的安装方向不受支持")
    for key in (
        "start_max_joint_error_rad", "zero_exclusion_radius_rad", "max_teach_seconds",
        "record_rate_hz", "max_recorded_step_rad", "return_max_joint_step_rad",
        "return_path_tolerance_rad",
        "return_target_tolerance_rad", "return_waypoint_timeout_s", "return_max_seconds",
    ):
        if not math.isfinite(float(config[key])) or float(config[key]) <= 0:
            raise ValueError(f"配置 {key} 必须为正数")
    if (not isinstance(config["return_speed_percent"], int)
            or not 1 <= config["return_speed_percent"] <= 20):
        raise ValueError("返回速度必须在 1% 到 20% 之间")
    if (not isinstance(config["return_countdown_seconds"], int)
            or not 0 <= config["return_countdown_seconds"] <= 30):
        raise ValueError("返回倒计时必须在 0 到 30 秒之间")
    config["safe_start_joint_rad"] = validate_joints(
        config["safe_start_joint_rad"], float(config["zero_exclusion_radius_rad"])
    )
    workspace = config.get("flange_workspace_m")
    if workspace is not None:
        if not isinstance(workspace, dict) or set(workspace) != {"x", "y", "z"}:
            raise ValueError("法兰工作空间须提供 x、y、z 三轴范围")
        for axis, pair in workspace.items():
            if (not isinstance(pair, list) or len(pair) != 2
                    or not all(math.isfinite(float(value)) for value in pair)
                    or float(pair[0]) >= float(pair[1])):
                raise ValueError(f"法兰工作空间 {axis} 范围无效")
            workspace[axis] = [float(value) for value in pair]
    return config


def joint_point(robot, max_age=1.0):
    """只返回新鲜的七关节反馈，避免使用 SDK 中过期的缓存值。"""
    message = robot.get_joint_angles()
    if not fresh(message, max_age):
        raise RuntimeError("关节角反馈缺失或过期")
    return [float(value) for value in message.msg]


def validate_flange_workspace(robot, joints, config):
    """如已配置工作空间，用 FK 检查关节姿态对应的法兰中心。"""
    workspace = config.get("flange_workspace_m")
    if workspace is None:
        return
    flange = robot.fk(joints)
    for index, axis in enumerate(("x", "y", "z")):
        value = float(flange[index])
        lower, upper = workspace[axis]
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(
                f"法兰中心 {axis}={value:.4f} m 超出工作空间 "
                f"[{lower:.4f}, {upper:.4f}] m"
            )


def validate_route_workspace(robot, route, config):
    """对相邻回位小步再插值检查，覆盖两个目标点之间的法兰位置。"""
    if config.get("flange_workspace_m") is None:
        return
    for first, second in zip(route, route[1:]):
        segments = max(1, math.ceil(math.dist(first, second) / 0.01))
        for part in range(segments + 1):
            fraction = part / segments
            point = [a + fraction * (b - a) for a, b in zip(first, second)]
            validate_flange_workspace(robot, point, config)


def healthy_status(robot, allowed_modes, allowed_teach):
    """同时核对控制模式、示教状态、机械臂状态与错误码。"""
    status = robot.get_arm_status()
    if not fresh(status):
        raise RuntimeError("机械臂状态反馈缺失或过期")
    if (int(status.msg.ctrl_mode) not in allowed_modes
            or int(status.msg.teach_status) not in allowed_teach
            or int(status.msg.arm_status) != 0 or status.msg.err_code):
        raise RuntimeError(
            f"机械臂状态不允许继续：{status.msg.ctrl_mode}, "
            f"{status.msg.arm_status}, {status.msg.teach_status}, "
            f"错误码=0x{status.msg.err_code:04X}"
        )
    return status


def healthy_preteach_status(robot):
    """接受正常未示教状态；固件 0x06 只在 CAN 模式下作为已终止示教接受。"""
    status = healthy_status(robot, {1, 2}, {0, 6})
    if int(status.msg.teach_status) == 6 and int(status.msg.ctrl_mode) != 1:
        raise RuntimeError("仅在 CAN 控制模式下接受已终止的示教状态")
    return status


def check_drivers(robot):
    """逐一核对七个驱动新鲜反馈、使能、欠压与故障标志。"""
    deadline = time.monotonic() + 2.0
    for index in range(1, 8):
        driver = robot.get_driver_states(index)
        while not fresh(driver) and time.monotonic() < deadline:
            time.sleep(0.05)
            driver = robot.get_driver_states(index)
        if not fresh(driver):
            raise RuntimeError(f"关节 {index} 驱动反馈缺失或过期")
        foc = driver.msg.foc_status
        if not foc.driver_enable_status or foc.voltage_too_low or foc.driver_error_status:
            raise RuntimeError(f"关节 {index} 未使能、欠压或有驱动故障")


def check_start(robot, config):
    """在示教前核对状态、驱动和当前姿态是否靠近配置中的非零起点。"""
    if wait_status(robot, lambda item: fresh(item), timeout=3.0) is None:
        raise RuntimeError("机械臂状态反馈缺失或过期")
    healthy_preteach_status(robot)
    check_drivers(robot)
    deadline = time.monotonic() + 2.0
    joint_message = robot.get_joint_angles()
    while not fresh(joint_message) and time.monotonic() < deadline:
        time.sleep(0.05)
        joint_message = robot.get_joint_angles()
    if not fresh(joint_message):
        raise RuntimeError("关节角反馈缺失或过期")
    current = validate_joints(list(joint_message.msg), config["zero_exclusion_radius_rad"])
    validate_flange_workspace(robot, current, config)
    errors = [abs(a - b) for a, b in zip(current, config["safe_start_joint_rad"])]
    if max(errors) > config["start_max_joint_error_rad"]:
        raise RuntimeError(
            f"当前姿态偏离安全起点 {config['safe_start_name']}：最大关节偏差 "
            f"{max(errors):.3f} rad，阈值 {config['start_max_joint_error_rad']:.3f} rad。"
            "不会自动从当前位置移动到起点。"
        )
    return current


def point_segment_distance(point, start, end):
    """计算七维关节反馈到一段规划路线的最短距离。"""
    delta = [b - a for a, b in zip(start, end)]
    norm_sq = sum(value * value for value in delta)
    if norm_sq == 0:
        return math.dist(point, start)
    alpha = max(0.0, min(1.0, sum((p - a) * d for p, a, d in zip(point, start, delta)) / norm_sq))
    projection = [a + alpha * d for a, d in zip(start, delta)]
    return math.dist(point, projection)


def simplify_path(points, tolerance):
    """在七维关节空间简化轨迹，保留偏离折线超过 tolerance 的点。"""
    if len(points) <= 2:
        return list(points)
    kept = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        first, last = pending.pop()
        if last - first < 2:
            continue
        middle = max(
            range(first + 1, last),
            key=lambda index: point_segment_distance(points[index], points[first], points[last]),
        )
        distance = point_segment_distance(points[middle], points[first], points[last])
        if distance > tolerance:
            kept.add(middle)
            pending.extend([(first, middle), (middle, last)])
    return [points[index] for index in sorted(kept)]


def plan_reverse_return(recorded_points, current, start, config):
    """保留原始轨迹的弯曲段，倒序插值为受限关节小步并返回原始起点。"""
    if not recorded_points:
        raise ValueError("没有有效的示教关节轨迹")
    if math.dist(current, recorded_points[-1]) > 0.05:
        raise ValueError("当前姿态与最后一个记录点差距过大，不能安全倒放")
    forward = [list(start), *[list(point) for point in recorded_points], list(current)]
    for point in forward:
        validate_joints(point, config["zero_exclusion_radius_rad"])
    for first, second in zip(forward, forward[1:]):
        if math.dist(first, second) > config["max_recorded_step_rad"]:
            raise ValueError("相邻示教反馈跨度过大，轨迹不连续，拒绝自动返回")
    backward = simplify_path(list(reversed(forward)), config["return_path_tolerance_rad"])
    route = [backward[0]]
    step = config["return_max_joint_step_rad"]
    for target in backward[1:]:
        previous = route[-1]
        segments = max(1, math.ceil(math.dist(previous, target) / step))
        for part in range(1, segments + 1):
            fraction = part / segments
            waypoint = [a + fraction * (b - a) for a, b in zip(previous, target)]
            validate_joints(waypoint, config["zero_exclusion_radius_rad"])
            route.append(waypoint)
    if math.dist(route[-1], start) > 1e-6:
        raise ValueError("返回路径未回到示教前位置")
    if len(route) > 2500:
        raise ValueError("返回路径点数超过 2500，拒绝自动回位")
    return route


def sample_row(robot, index, start_mono):
    """将同一时刻附近的关节、法兰和状态反馈整理为一条 CSV 记录。"""
    now = time.time()
    joints = robot.get_joint_angles()
    pose = robot.get_flange_pose()
    status = robot.get_arm_status()
    if not (fresh(joints) and fresh(pose) and fresh(status)):
        return None
    joint_values = [float(value) for value in joints.msg]
    pose_values = [float(value) for value in pose.msg]
    if len(joint_values) != 7 or len(pose_values) != 6:
        return None
    if not all(math.isfinite(value) for value in pose_values):
        raise RuntimeError("法兰位姿包含非有限值")
    row = {
        "sample_index": index,
        "captured_at_utc": datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="milliseconds"),
        "elapsed_s": round(time.monotonic() - start_mono, 3),
        "joint_feedback_unix_s": float(joints.timestamp),
        "pose_feedback_unix_s": float(pose.timestamp),
        "status_feedback_unix_s": float(status.timestamp),
    }
    for column, attribute in zip(
        STATUS_COLUMNS, ("ctrl_mode", "arm_status", "teach_status", "motion_status")
    ):
        value = getattr(status.msg, attribute)
        row[column] = getattr(value, "name", str(value))
    row.update(zip(JOINT_COLUMNS, joint_values))
    row.update(zip(POSE_COLUMNS, pose_values))
    return row, joint_values


def capture_settling(robot, output, recorded, start_mono, config, metadata, last_joint_ts):
    """退出拖动后继续记录关节，直至反馈显示机械臂已经停稳。"""
    deadline = time.monotonic() + 3.0
    last_good = time.monotonic()
    stable_since = None
    stable_anchor = None
    count = 0
    # 退出拖动并非瞬时停止。把这段真实位移继续加入原轨迹，倒放时才有连续起点。
    with output.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        while time.monotonic() < deadline:
            healthy_status(robot, {1, 2}, {0, 2})
            sampled = sample_row(robot, metadata["recorded_samples"], start_mono)
            if sampled is not None:
                row, point = sampled
                joint_ts = row["joint_feedback_unix_s"]
                if joint_ts > last_joint_ts:
                    validate_joints(point, config["zero_exclusion_radius_rad"])
                    validate_flange_workspace(robot, point, config)
                    if math.dist(recorded[-1], point) > config["max_recorded_step_rad"]:
                        raise RuntimeError("退出示教后的关节反馈跳变过大，拒绝自动回位")
                    writer.writerow(row)
                    stream.flush()
                    recorded.append(point)
                    metadata["recorded_samples"] += 1
                    count += 1
                    last_joint_ts = joint_ts
                    now = time.monotonic()
                    last_good = now
                    # 连续至少 0.3 秒未离开 1 mrad 关节球，才视为停稳。
                    if stable_anchor is None or math.dist(stable_anchor, point) > 0.001:
                        stable_anchor = point
                        stable_since = now
                    elif now - stable_since >= 0.3:
                        print(f"退出示教后已停稳，补录 {count} 条。", flush=True)
                        return count
            if time.monotonic() - last_good > 1.0:
                raise RuntimeError("退出示教后关节反馈中断，拒绝自动回位")
            time.sleep(1.0 / config["record_rate_hz"])
    raise TimeoutError("退出示教后 3 秒内未确认停稳，拒绝自动回位")


def stop_drag(bus, robot):
    """发送 CAN 结束示教帧并等待控制器确认退出拖动。"""
    for _ in range(2):
        send_frame(bus, 0x150, [0, 0, 2, 0, 0, 0, 0, 0])
        status = wait_status(
            robot, lambda item: fresh(item) and int(item.msg.teach_status) in (0, 2), timeout=1.5
        )
        if status is not None:
            print("已结束拖动示教：", status.msg.teach_status, flush=True)
            return
    raise RuntimeError("已发送结束示教指令，但未确认退出示教；请现场检查")


def wait_waypoint(robot, target, config, abort_return):
    """逐步监测关节目标、模式、驱动和超时；异常时交给上层急停。"""
    deadline = time.monotonic() + config["return_waypoint_timeout_s"]
    reached_count = 0
    poll_count = 0
    while time.monotonic() < deadline:
        if abort_return():
            raise InterruptedError("返回过程收到中止信号")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        if poll_count % 4 == 0:
            check_drivers(robot)
        poll_count += 1
        current = joint_point(robot)
        validate_flange_workspace(robot, current, config)
        if max(abs(a - b) for a, b in zip(current, target)) <= config["return_target_tolerance_rad"]:
            reached_count += 1
            if reached_count >= 2:
                return
        else:
            reached_count = 0
        time.sleep(0.05)
    raise TimeoutError("返回过程中关节未在规定时间到达目标")


def return_to_start(robot, bus, recorded, original, config, abort_return,
                    on_emergency_stop=None):
    """切回关节控制并沿完整示教轨迹倒序回到本次实际起点。"""
    # 延迟导入，避免 go_to_start 与本模块相互导入时形成循环。
    from experiments.single_arm.start_transfer.go_to_start import (
        RoutePaused, confirm_stationary_at_previous,
    )
    current = validate_joints(joint_point(robot), config["zero_exclusion_radius_rad"])
    # 返回从退出示教后的实际位置开始，不能假设它等于最后一条拖动样本。
    route = plan_reverse_return(recorded, current, original, config)
    validate_route_workspace(robot, route, config)
    print(f"倒序返回路径包含 {len(route) - 1} 个小步；准备切到 CAN 关节控制。", flush=True)

    mount_code = MOUNT_CODES[config["mount"]]
    send_frame(bus, 0x151, [1, 1, config["return_speed_percent"], 0, 0, mount_code, 0, 0])
    status = wait_status(
        robot, lambda item: fresh(item) and int(item.msg.ctrl_mode) == 1, timeout=3.0
    )
    if status is None:
        raise RuntimeError("未确认切回 CAN 控制模式；不发送运动指令")
    healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
    check_drivers(robot)
    robot.set_speed_percent(config["return_speed_percent"])
    robot.set_motion_mode("j")
    move_mode = wait_status(
        robot,
        lambda item: fresh(item) and int(item.msg.ctrl_mode) == 1
        and int(item.msg.mode_feedback) == 1 and int(item.msg.arm_status) == 0
        and item.msg.err_code == 0,
        timeout=3.0,
    )
    if move_mode is None:
        raise RuntimeError("未确认进入 CAN 关节运动模式；不发送运动指令")

    deadline = time.monotonic() + config["return_max_seconds"]
    # 中途状态、路径或到位检查失败时，已发送运动指令则调用电子急停。
    motion_sent = False
    try:
        for index, waypoint in enumerate(route[1:], 1):
            if abort_return():
                raise InterruptedError("返回过程收到中止信号")
            if time.monotonic() > deadline:
                # 上一小步已确认到位，此时停止发布后续目标即可；
                # 总时间用尽不应被误判为运动中故障而发送阻尼急停。
                raise RoutePaused("返回总时间已到；停在上一小步，未发送电子急停",
                                  reason="return_time_limit")
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            current = joint_point(robot)
            validate_flange_workspace(robot, current, config)
            if max(abs(a - b) for a, b in zip(current, waypoint)) <= config["return_target_tolerance_rad"]:
                continue
            motion_sent = True
            robot.move_j(waypoint)
            try:
                wait_waypoint(robot, waypoint, config, abort_return)
            except TimeoutError as exc:
                # 目标未执行而整机仍正常、连续停在发送前的姿态时，
                # 暂停本次返回；不让软件计时触发会失去位置保持的急停。
                if (confirm_stationary_at_previous(robot, current, config)
                        and max(abs(a - b) for a, b in zip(
                            joint_point(robot), waypoint
                        )) > config["return_target_tolerance_rad"] + 0.005):
                    raise RoutePaused(
                        f"第 {index} 步未执行；已暂停返回，须核对状态和当前位置",
                        reason="stationary_waypoint_timeout",
                    ) from exc
                raise
            if index % 20 == 0:
                print(f"返回进度：{index}/{len(route) - 1}", flush=True)
        if max(abs(a - b) for a, b in zip(joint_point(robot), original)) > config["return_target_tolerance_rad"]:
            raise RuntimeError("返回结束，但示教前位置误差超过阈值")
    except RoutePaused:
        raise
    except Exception as exc:
        if motion_sent:
            if on_emergency_stop is not None:
                try:
                    on_emergency_stop(type(exc).__name__)
                except Exception:
                    pass
            robot.electronic_emergency_stop()
        raise
    print("已沿示教轨迹返回示教前位置。", flush=True)


def run_session(robot, config, args):
    """统筹限时拖动、记录、停稳、轨迹校验与可中止的原路回位。"""
    original = check_start(robot, config)
    print(f"起点 {config['safe_start_name']} 已核对；本次原位关节角：{original}", flush=True)

    output = args.output or Path("experiments/single_arm/teaching/data/recordings") / (
        "nero_session_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".csv"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError(f"记录文件已存在：{output}")
    metadata_path = output.with_suffix(".json")
    if metadata_path.exists():
        raise ValueError(f"元数据文件已存在：{metadata_path}")
    metadata = {
        "recording": str(output),
        "safe_start_name": config["safe_start_name"],
        "safe_start_source": config["safe_start_source"],
        "original_joint_rad": original,
        "max_teach_seconds": args.max_seconds,
        "mount": config["mount"],
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "recorded_samples": 0,
        "post_stop_samples": 0,
        "stop_reason": None,
        "return_completed": False,
        "emergency_stop_requested": False,
    }
    # 先写失败默认值；即使进程随后异常退出，也不会把不完整记录误判为成功。
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    phase = "recording"
    stop_requested = False
    abort_requested = False

    def on_signal(_signum, _frame):
        nonlocal stop_requested, abort_requested
        if phase == "recording":
            stop_requested = True
        else:
            abort_requested = True

    prior_int = signal.signal(signal.SIGINT, on_signal)
    prior_term = signal.signal(signal.SIGTERM, on_signal)
    try:
        with can.Bus(channel=config["channel"], interface="socketcan", local_loopback=True) as bus:
            mount_code = MOUNT_CODES[config["mount"]]
            baseline_status = healthy_preteach_status(robot)
            baseline_timestamp = float(baseline_status.timestamp)
            send_frame(bus, 0x151, [1, 0xFF, 50, 0, 0, mount_code, 0, 0])
            ready = wait_status(
                robot,
                lambda item: fresh(item) and float(item.timestamp) > baseline_timestamp
                and int(item.msg.ctrl_mode) == 1
                and int(item.msg.arm_status) == 0 and item.msg.err_code == 0,
                timeout=3.0,
            )
            if ready is None:
                raise RuntimeError("未确认正常 CAN 控制模式")

            started = False
            stopped = False
            recording_error = None
            recorded = []
            last_joint_ts = float("-inf")
            start_mono = time.monotonic()
            try:
                started = True
                send_frame(bus, 0x150, [0, 0, 1, 0, 0, 0, 0, 0])
                entered = wait_status(
                    robot,
                    lambda item: fresh(item) and int(item.msg.teach_status) == 1,
                    timeout=3.0,
                )
                if entered is None:
                    raise RuntimeError("未确认进入拖动示教")
                with output.open("x", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
                    writer.writeheader()
                    stream.flush()
                    print(f"示教开始；最多 {args.max_seconds:g} 秒，记录到 {output}", flush=True)
                    count = 0
                    last_good = time.monotonic()
                    next_tick = time.monotonic()
                    while time.monotonic() - start_mono < args.max_seconds and not stop_requested:
                        status = robot.get_arm_status()
                        if not fresh(status) or status.msg.err_code or int(status.msg.arm_status) not in (0, 11):
                            raise RuntimeError("示教中机械臂状态异常或反馈中断")
                        if int(status.msg.teach_status) != 1:
                            print("检测到外部结束示教指令。", flush=True)
                            break
                        sampled = sample_row(robot, count, start_mono)
                        if sampled is not None:
                            row, point = sampled
                            try:
                                validate_joints(point, config["zero_exclusion_radius_rad"])
                                validate_flange_workspace(robot, point, config)
                                if recorded and math.dist(recorded[-1], point) > config["max_recorded_step_rad"]:
                                    raise ValueError("示教中相邻关节反馈跳变过大，立即结束拖动")
                            except ValueError:
                                metadata["rejected_joint_rad"] = point
                                metadata["rejected_sample_elapsed_s"] = row["elapsed_s"]
                                raise
                            writer.writerow(row)
                            stream.flush()
                            recorded.append(point)
                            count += 1
                            metadata["recorded_samples"] = count
                            last_joint_ts = row["joint_feedback_unix_s"]
                            last_good = time.monotonic()
                        if time.monotonic() - last_good > 1.0:
                            raise RuntimeError("连续一秒没有完整的位置反馈")
                        next_tick += 1.0 / config["record_rate_hz"]
                        time.sleep(max(0.0, next_tick - time.monotonic()))
                    if stop_requested:
                        metadata["stop_reason"] = "user_interrupt"
                        print("收到提前结束请求。", flush=True)
                    elif time.monotonic() - start_mono >= args.max_seconds:
                        metadata["stop_reason"] = "time_limit"
                        print("已达到最长示教时间。", flush=True)
                    else:
                        metadata["stop_reason"] = "external_stop"
                    print(f"示教采样结束：{count} 条。", flush=True)
            except Exception as exc:
                metadata["stop_reason"] = "error"
                recording_error = exc
            finally:
                if started:
                    try:
                        stop_drag(bus, robot)
                        stopped = True
                    except Exception as exc:
                        recording_error = recording_error or exc

            if recording_error:
                raise recording_error
            if not stopped or len(recorded) < 2:
                raise RuntimeError("示教未正常结束或有效采样不足，拒绝自动返回")

            metadata["post_stop_samples"] = capture_settling(
                robot, output, recorded, start_mono, config, metadata, last_joint_ts
            )

            preview_current = validate_joints(
                joint_point(robot), config["zero_exclusion_radius_rad"]
            )
            preview_route = plan_reverse_return(recorded, preview_current, original, config)
            validate_route_workspace(robot, preview_route, config)
            print(f"已检查倒序路径：{len(preview_route) - 1} 个小步。", flush=True)

            phase = "return"
            for remaining in range(config["return_countdown_seconds"], 0, -1):
                if abort_requested:
                    raise InterruptedError("已取消自动返回")
                print(f"{remaining} 秒后沿原轨迹返回；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            if abort_requested:
                raise InterruptedError("已取消自动返回")

            return_to_start(
                robot, bus, recorded, original, config, lambda: abort_requested,
                on_emergency_stop=lambda reason: metadata.update(
                    emergency_stop_requested=True,
                    emergency_stop_reason=reason,
                ),
            )
            metadata["return_completed"] = True
            print(f"会话完成；原始示教文件：{output}", flush=True)
    except Exception as exc:
        metadata["error"] = str(exc)
        if hasattr(exc, "reason"):
            metadata["pause_reason"] = exc.reason
        raise
    finally:
        metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        signal.signal(signal.SIGINT, prior_int)
        signal.signal(signal.SIGTERM, prior_term)


def main():
    """默认只读预检；只有显式 --run 才启动有限时长的拖动会话。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true", help="启动限时示教及自动原路返回；默认只读检查")
    parser.add_argument("--max-seconds", type=float, help="覆盖配置的最长示教秒数")
    parser.add_argument("--output", type=Path, help="本次示教 CSV 路径")
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        if args.max_seconds is None:
            args.max_seconds = float(config["max_teach_seconds"])
        if not math.isfinite(args.max_seconds) or not 1 <= args.max_seconds <= 300:
            parser.error("--max-seconds 必须在 1 到 300 秒之间")
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if args.run:
                run_session(robot, config, args)
            else:
                current = check_start(robot, config)
                print(f"只读检查通过：当前位置接近 {config['safe_start_name']}。")
                print(f"最长示教时间：{args.max_seconds:g} 秒；返回速度：{config['return_speed_percent']}%。")
                print("当前关节角：", current)
                print("运行时加 --run；示教结束将沿本次记录的轨迹返回当前起点。")
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError, can.CanError) as exc:
        print(f"会话未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
