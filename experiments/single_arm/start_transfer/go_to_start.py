#!/usr/bin/env python3
"""规划并在现场确认后，沿小步关节路线移动到配置中的安全起点。"""

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

from experiments.single_arm.can_setup.can_drag_teach import MOUNT_CODES, fresh, send_frame, wait_status
from experiments.single_arm.control_interfaces.publishers import JointTargetPublisher
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_preteach_status,
    healthy_status, joint_point, load_config, point_segment_distance,
    validate_flange_workspace, validate_joints, validate_route_workspace,
)


PLAN_STEP_RAD = 0.05
MAX_TRAVEL_RAD = 0.8
MAX_ESCAPE_TRAVEL_RAD = 1.2
START_TOLERANCE_RAD = 0.005
PATH_TOLERANCE_RAD = 0.06
SPEED_PERCENT = 5
# 所有关节角及关节距离使用 rad；法兰坐标由 SDK FK 给出，单位为 m。


class RoutePaused(RuntimeError):
    """在确认小步到位后停止后续下发；须现场核对实际保持状态。"""

    def __init__(self, message, reason="cycle_time_limit"):
        super().__init__(message)
        self.reason = reason


class WaypointTimeout(TimeoutError):
    """保存急停前的到位误差；用于区分持续变慢和完全无进展。"""

    def __init__(self, elapsed_s, best_error_rad, last_error_rad):
        self.elapsed_s = elapsed_s
        self.best_error_rad = best_error_rad
        self.last_error_rad = last_error_rad
        super().__init__(
            f"关节小步在 {elapsed_s:.1f} 秒内未到位；"
            f"最小误差 {best_error_rad:.4f} rad，末次误差 {last_error_rad:.4f} rad"
        )


def plan_route(start, target, config):
    """从实时姿态规划到 S1；靠近全零位时先单独让 J4 远离危险姿态。"""
    zero_radius = config["zero_exclusion_radius_rad"]
    start = validate_joints(start, 0.0)
    target = validate_joints(target, config["zero_exclusion_radius_rad"])
    route = [start]
    if math.dist(start, [0.0] * 7) < zero_radius:
        # 靠近全零位时不直接对七关节同时线性插值，先检查 J4 单关节脱离段。
        escape = list(start)
        escape[3] = target[3]
        if (start[3] >= -0.02 or target[3] >= start[3] - 0.1
                or math.dist(escape, [0.0] * 7) < zero_radius + 0.03):
            raise ValueError("近零位无法按 J4 远离零位的策略脱离；需人工指定路径")
        if math.dist(start, escape) + math.dist(escape, target) > MAX_ESCAPE_TRAVEL_RAD:
            raise ValueError("近零位脱离路线过长；需人工指定路径")
        previous_norm = math.dist(start, [0.0] * 7)
        for point in interpolate(start, escape)[1:]:
            point = validate_joints(point, 0.0)
            norm = math.dist(point, [0.0] * 7)
            if norm + 1e-9 < previous_norm:
                raise ValueError("J4 脱离路线更靠近全零位")
            previous_norm = norm
            route.append(point)
        for point in interpolate(escape, target)[1:]:
            route.append(validate_joints(point, zero_radius))
    else:
        distance = math.dist(start, target)
        if distance > MAX_TRAVEL_RAD:
            raise ValueError(f"起点关节空间距离 {distance:.3f} rad 超过 {MAX_TRAVEL_RAD:.1f} rad；需人工指定中间路径")
        for point in interpolate(start, target)[1:]:
            route.append(validate_joints(point, zero_radius))
    route[-1] = target
    return route


def interpolate(start, target):
    """按关节空间距离把一段目标拆成不超过规定长度的小步。"""
    segments = max(1, math.ceil(math.dist(start, target) / PLAN_STEP_RAD))
    points = []
    for index in range(segments + 1):
        fraction = index / segments
        points.append([a + fraction * (b - a) for a, b in zip(start, target)])
    points[0] = list(start)
    points[-1] = list(target)
    return points


def flange_envelope(robot, route, config):
    """在关节小步之间继续细采样，用 FK 汇总法兰中心范围。"""
    limits = [[float("inf"), float("-inf")] for _ in range(3)]
    for first, second in zip(route, route[1:]):
        segments = max(1, math.ceil(math.dist(first, second) / 0.01))
        for index in range(segments + 1):
            fraction = index / segments
            point = [a + fraction * (b - a) for a, b in zip(first, second)]
            validate_flange_workspace(robot, point, config)
            pose = robot.fk(point)
            for axis in range(3):
                value = float(pose[axis])
                if not math.isfinite(value):
                    raise ValueError("正运动学返回了非有限法兰坐标")
                limits[axis][0] = min(limits[axis][0], value)
                limits[axis][1] = max(limits[axis][1], value)
    return dict(zip(("x", "y", "z"), limits))


def stable_joint_feedback(robot):
    """对两次关节读数做停稳检查，并交叉核对 FK 与 CAN 法兰反馈。"""
    first = joint_point(robot)
    time.sleep(0.25)
    second = joint_point(robot)
    if max(abs(a - b) for a, b in zip(first, second)) > 0.002:
        raise RuntimeError("机械臂尚未停稳，拒绝规划或运动")
    flange = robot.get_flange_pose()
    if not fresh(flange):
        raise RuntimeError("法兰位置反馈缺失")
    fk = robot.fk(second)
    # 两套位置信息不一致时可能存在零点、安装方向或反馈解释问题，停止规划。
    if math.dist([float(v) for v in fk[:3]], [float(v) for v in flange.msg[:3]]) > 0.02:
        raise RuntimeError("正运动学与实测法兰位置不一致")
    return second


def check_preplan_drivers(robot):
    """规划阶段只要求各驱动状态可读且无欠压或故障。"""
    for index in range(1, 8):
        deadline = time.monotonic() + 2.0
        driver = robot.get_driver_states(index)
        while not fresh(driver) and time.monotonic() < deadline:
            time.sleep(0.05)
            driver = robot.get_driver_states(index)
        if not fresh(driver):
            raise RuntimeError(f"关节 {index} 驱动反馈缺失")
        foc = driver.msg.foc_status
        if foc.voltage_too_low or foc.driver_error_status:
            raise RuntimeError(f"关节 {index} 驱动报告欠压或故障")


def check_plan(plan, config):
    """重新推导路线并比较规划文件，阻止篡改、复用或配置不匹配。"""
    if plan.get("schema_version") != 1 or plan.get("mount") != config["mount"]:
        raise ValueError("规划文件版本或安装方向不匹配")
    if plan.get("executed"):
        raise ValueError("规划文件已执行，不能重复使用")
    if plan.get("flange_workspace_m") != config["flange_workspace_m"]:
        raise ValueError("规划文件与当前法兰工作空间配置不匹配")
    target = config["safe_start_joint_rad"]
    if len(plan["target_joint_rad"]) != 7:
        raise ValueError("规划文件目标关节数错误")
    if max(abs(a - b) for a, b in zip(plan["target_joint_rad"], target)) > 1e-9:
        raise ValueError("规划文件与当前配置起点不匹配")
    expected = plan_route(plan["start_joint_rad"], target, config)
    route = plan["waypoints_joint_rad"]
    if len(route) != len(expected) or any(
        len(actual) != 7 or max(abs(a - b) for a, b in zip(actual, planned)) > 1e-9
        for actual, planned in zip(route, expected)
    ):
        raise ValueError("规划文件关节路线与当前规划算法不匹配")
    if plan.get("speed_percent") != SPEED_PERCENT:
        raise ValueError("规划文件速度不匹配")
    route_kind = (
        "j4_escape_then_linear" if math.dist(expected[0], [0.0] * 7)
        < config["zero_exclusion_radius_rad"] else "joint_linear"
    )
    if plan.get("route_kind") != route_kind:
        raise ValueError("规划文件路线类型不匹配")
    return expected


def joint_box_deviation(current, start, target):
    """关节反馈超出起终点逐轴范围的最大距离；允许控制器各轴不同步。"""
    if len(current) != 7 or len(start) != 7 or len(target) != 7:
        raise ValueError("关节盒范围需要七个关节角")
    return max(
        max(min(a, b) - value, value - max(a, b), 0.0)
        for value, a, b in zip(current, start, target)
    )


def wait_waypoint(robot, previous, target, config, aborted, on_feedback=None,
                  target_tolerance_rad=0.02, timeout_s=None, max_timeout_s=None,
                  on_status=None, path_mode="line", joint_box_margin_rad=0.03):
    """逐步核对到位；持续取得进展时延长等待，但仍有硬上限。"""
    base_timeout = float(config["return_waypoint_timeout_s"] if timeout_s is None else timeout_s)
    hard_timeout = float(base_timeout if max_timeout_s is None else max_timeout_s)
    if not 0 < base_timeout <= hard_timeout:
        raise ValueError("小步到位时间上限无效")
    if path_mode not in ("line", "joint_box") or joint_box_margin_rad < 0:
        raise ValueError("关节路径检查参数无效")
    started = time.monotonic()
    base_deadline = started + base_timeout
    hard_deadline = started + hard_timeout
    last_progress = started
    best_error = float("inf")
    last_error = float("inf")
    reached = 0
    polls = 0
    while time.monotonic() < hard_deadline:
        if aborted():
            raise InterruptedError("收到中止信号")
        status = healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        if on_status is not None:
            on_status(status)
        if polls % 4 == 0:
            check_drivers(robot)
        polls += 1
        current = joint_point(robot)
        validate_flange_workspace(robot, current, config)
        if on_feedback is not None:
            on_feedback(current)
        if path_mode == "line":
            if point_segment_distance(current, previous, target) > PATH_TOLERANCE_RAD:
                raise RuntimeError("关节反馈偏离规划小步路线")
        elif joint_box_deviation(current, previous, target) > joint_box_margin_rad:
            raise RuntimeError("关节反馈超出起终点逐轴范围")
        last_error = max(abs(a - b) for a, b in zip(current, target))
        if last_error < best_error - 0.001:
            best_error = last_error
            last_progress = time.monotonic()
        if last_error <= target_tolerance_rad:
            reached += 1
            if reached >= 2:
                return
        else:
            reached = 0
        # 达到基础超时后，只有近期仍在接近目标才继续等；最多等到硬上限。
        now = time.monotonic()
        if now >= base_deadline and now - last_progress > 1.0:
            break
        time.sleep(0.05)
    raise WaypointTimeout(time.monotonic() - started, best_error, last_error)


def confirm_stationary_at_previous(robot, previous, config, duration_s=0.8):
    """小步未执行时，持续确认机械臂正常且保持在上一目标附近。

仅供超时后的无急停暂停判定。控制器报告到位失败也可以暂停，前提是
连续反馈证明关节仍在上一点；状态异常、反馈缺失或任何移动都返回 False。
    """
    started = time.monotonic()
    anchor = None
    samples = 0
    try:
        while time.monotonic() - started < duration_s:
            status = healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            if int(status.msg.motion_status) not in (0, 1):
                return False
            check_drivers(robot)
            current = joint_point(robot)
            validate_flange_workspace(robot, current, config)
            if max(abs(a - b) for a, b in zip(current, previous)) > 0.003:
                return False
            if anchor is None:
                anchor = current
            elif max(abs(a - b) for a, b in zip(current, anchor)) > 0.001:
                return False
            samples += 1
            time.sleep(0.1)
        return samples >= 5
    except (AttributeError, RuntimeError, ValueError, OSError):
        return False


def execute_route(robot, config, route, aborted, on_waypoint=None, on_feedback=None,
                  target_tolerance_rad=0.02, speed_percent=SPEED_PERCENT,
                  waypoint_timeout_s=None, waypoint_max_timeout_s=None,
                  on_command=None, stop_after_waypoint=None, on_status=None,
                  pause_on_stationary_timeout=False, on_command_published=None,
                  path_mode="line", joint_box_margin_rad=0.03,
                  on_emergency_stop=None, stationary_retry_limit=0,
                  on_stationary_retry=None):
    """显式切换 CAN 关节模式，逐小步发指令并在运动异常后电子急停。"""
    if not isinstance(speed_percent, int) or not 1 <= speed_percent <= 20:
        raise ValueError("低速实验只允许 1%～20% 的整数速度")
    if stationary_retry_limit not in (0, 1):
        raise ValueError("停在上一点时最多允许重发同一目标一次")
    if max(abs(a - b) for a, b in zip(joint_point(robot), route[0])) > START_TOLERANCE_RAD:
        raise RuntimeError("当前位置已偏离规划起点；未发送运动指令")

    def request_emergency_stop(reason):
        # 报告写入故障不能妨碍真正的急停指令。
        if on_emergency_stop is not None:
            try:
                on_emergency_stop(reason)
            except Exception:
                pass
        robot.electronic_emergency_stop()

    with can.Bus(channel=config["channel"], interface="socketcan", local_loopback=True) as bus:
        baseline = robot.get_arm_status()
        if not fresh(baseline):
            raise RuntimeError("切换模式前状态反馈缺失")
        # 模式切换后必须看到时间更新的 CAN 状态，不能拿切换前的缓存作为确认。
        baseline_timestamp = float(baseline.timestamp)
        send_frame(bus, 0x151, [1, 1, speed_percent, 0, 0, MOUNT_CODES[config["mount"]], 0, 0])
        status = wait_status(
            robot,
            lambda item: fresh(item) and float(item.timestamp) > baseline_timestamp
            and int(item.msg.ctrl_mode) == 1,
            timeout=3,
        )
        if status is None:
            raise RuntimeError("未确认 CAN 控制模式；未发送运动指令")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        robot.set_speed_percent(speed_percent)
        robot.set_motion_mode("j")
        mode = wait_status(
            robot,
            lambda item: fresh(item) and int(item.msg.ctrl_mode) == 1
            and int(item.msg.mode_feedback) == 1 and int(item.msg.arm_status) == 0
            and item.msg.err_code == 0,
            timeout=3,
        )
        if mode is None:
            raise RuntimeError("未确认关节运动模式；未发送运动指令")
        mode_shift = max(abs(a - b) for a, b in zip(joint_point(robot), route[0]))
        if mode_shift > START_TOLERANCE_RAD:
            if mode_shift > 0.03:
                request_emergency_stop("mode_shift")
            raise RuntimeError("切换模式后当前位置改变；未发送运动指令")

        # 标记是否真正下发过 move_j；运动中的异常会触发 SDK 阻尼急停。
        # 阻尼急停可能让抬起的机械臂下落，不能把它理解为保持当前位置。
        motion_sent = False
        try:
            for index, target in enumerate(route[1:], 1):
                if aborted():
                    raise InterruptedError("收到中止信号")
                healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                check_drivers(robot)
                current = joint_point(robot)
                if max(abs(a - b) for a, b in zip(current, route[index - 1])) > 0.03:
                    raise RuntimeError("当前位置偏离上一小步目标")
                if on_command is not None:
                    # 这里记录的是发送尝试，不能证明 SDK 已返回，更不能证明控制器收到。
                    on_command(index, target)
                motion_sent = True
                JointTargetPublisher().publish(robot, target)
                if on_command_published is not None:
                    on_command_published(index, target)
                for retry in range(stationary_retry_limit + 1):
                    try:
                        wait_waypoint(robot, route[index - 1], target, config, aborted,
                                      on_feedback=on_feedback,
                                      target_tolerance_rad=target_tolerance_rad,
                                      timeout_s=waypoint_timeout_s,
                                      max_timeout_s=waypoint_max_timeout_s,
                                      on_status=on_status,
                                      path_mode=path_mode,
                                      joint_box_margin_rad=joint_box_margin_rad)
                        break
                    except WaypointTimeout as exc:
                        if pause_on_stationary_timeout:
                            # 目标未执行且连续反馈确认仍保持上一点时，停止后续下发，
                            # 无须让抬起的机械臂因软件超时而进入阻尼急停。
                            # 若位置漂移、状态异常或反馈缺失，则走外层故障急停。
                            if (confirm_stationary_at_previous(
                                    robot, route[index - 1], config)
                                    and max(abs(a - b) for a, b in zip(
                                        joint_point(robot), target
                                    )) > target_tolerance_rad + 0.005):
                                status = healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                                if retry < stationary_retry_limit and int(
                                    status.msg.motion_status
                                ) == 0 and not aborted():
                                    # 仅在控制器仍报告正常到位状态时重发原目标；
                                    # 报告到位失败(1)时直接暂停，不反复尝试。
                                    check_drivers(robot)
                                    if on_stationary_retry is not None:
                                        on_stationary_retry(index, retry + 1)
                                    JointTargetPublisher().publish(robot, target)
                                    if on_command_published is not None:
                                        on_command_published(index, target)
                                    continue
                                raise RoutePaused(
                                    f"第 {index} 步未执行；机械臂仍停在上一小步附近，"
                                    "已暂停后续目标，须现场核对状态并重新规划",
                                    reason="stationary_waypoint_timeout",
                                ) from exc
                        raise
                if on_waypoint is not None:
                    on_waypoint(index, target)
                print(f"已到达 {index}/{len(route)-1} 小步", flush=True)
                if stop_after_waypoint is not None and stop_after_waypoint(index, target):
                    raise RoutePaused("整轮运行时间已到；在小步到位后停止，未发送电子急停")
            if max(abs(a - b) for a, b in zip(joint_point(robot), route[-1])) > 0.02:
                raise RuntimeError("最终位置偏离 S1")
        except RoutePaused:
            # 已确认落在目标附近才退出；停止下发后的姿态仍要现场监护。
            raise
        except Exception as exc:
            if motion_sent:
                request_emergency_stop(type(exc).__name__)
            raise


def main():
    """默认只生成或复核规划；--run 才在倒计时后执行一次路线。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--plan", type=Path, help="执行已生成的规划文件；没有 --run 时只复核")
    parser.add_argument("--run", action="store_true", help="执行 --plan 指定的关节路线")
    args = parser.parse_args()
    if args.run and args.plan is None:
        parser.error("--run 必须同时指定 --plan")
    try:
        config = load_config(args.config)
        plan = json.loads(args.plan.read_text(encoding="utf-8")) if args.plan else None
        route = check_plan(plan, config) if plan else None
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, lambda item: fresh(item), timeout=3) is None:
                raise RuntimeError("机械臂状态反馈缺失")
            status = robot.get_arm_status()
            if (not fresh(status) or int(status.msg.ctrl_mode) not in (1, 2)
                    or int(status.msg.arm_status) not in (0, 6)
                    or int(status.msg.teach_status) != 0 or status.msg.err_code):
                raise RuntimeError("机械臂状态不适合规划或运动")
            check_preplan_drivers(robot)
            if args.run:
                healthy_preteach_status(robot)
                check_drivers(robot)
            current = validate_joints(stable_joint_feedback(robot), 0.0)
            if route is None:
                route = plan_route(current, config["safe_start_joint_rad"], config)
                envelope = flange_envelope(robot, route, config)
                output = Path("experiments/single_arm/start_transfer/data/plans") / (
                    "nero_to_start_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
                )
                output.parent.mkdir(parents=True, exist_ok=True)
                plan = {
                    "schema_version": 1,
                    "created_at_utc": datetime.now(timezone.utc).isoformat(),
                    "mount": config["mount"],
                    "start_joint_rad": route[0],
                    "target_joint_rad": route[-1],
                    "route_kind": (
                        "j4_escape_then_linear" if math.dist(route[0], [0.0] * 7)
                        < config["zero_exclusion_radius_rad"] else "joint_linear"
                    ),
                    "waypoints_joint_rad": route,
                    "flange_xyz_envelope_m": envelope,
                    "speed_percent": SPEED_PERCENT,
                    "flange_workspace_m": config["flange_workspace_m"],
                    "executed": False,
                }
                with output.open("x", encoding="utf-8") as stream:
                    stream.write(json.dumps(plan, ensure_ascii=False, indent=2) + "\n")
                print(f"只读规划完成：{len(route)-1} 小步，关节空间距离 {math.dist(route[0], route[-1]):.4f} rad")
                print("法兰中心沿途范围 (m)：", envelope)
                print(f"规划文件：{output}")
                print("未发送运动指令。")
                return 0

            if math.dist(current, route[0]) > START_TOLERANCE_RAD:
                raise RuntimeError("当前位置已偏离规划起点；请重新生成规划")
            envelope = flange_envelope(robot, route, config)
            validate_route_workspace(robot, route, config)
            if any(
                any(abs(a - b) > 1e-6 for a, b in zip(envelope[axis], plan["flange_xyz_envelope_m"][axis]))
                for axis in ("x", "y", "z")
            ):
                raise ValueError("规划文件法兰范围与当前正运动学结果不匹配")
            print(f"规划复核通过：{len(route)-1} 小步，法兰范围 {envelope}")
            if not args.run:
                print("只读复核完成；未发送运动指令。")
                return 0
            aborted = False

            def on_signal(_signum, _frame):
                nonlocal aborted
                aborted = True

            previous_int = signal.signal(signal.SIGINT, on_signal)
            previous_term = signal.signal(signal.SIGTERM, on_signal)
            attempt = {
                "started_at_utc": datetime.now(timezone.utc).isoformat(),
                "start_joint_rad": current,
                "completed": False,
            }
            plan.setdefault("execution_attempts", []).append(attempt)
            args.plan.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            try:
                for remaining in range(5, 0, -1):
                    if aborted:
                        raise InterruptedError("已取消去起点")
                    print(f"{remaining} 秒后移动到 S1；按 Ctrl+C 取消。", flush=True)
                    time.sleep(1)
                if aborted:
                    raise InterruptedError("已取消去起点")
                execute_route(robot, config, route, lambda: aborted)
                plan["executed"] = True
                plan["executed_at_utc"] = datetime.now(timezone.utc).isoformat()
                attempt["completed"] = True
                print("已到达配置中的 S1 起点。", flush=True)
            except Exception as exc:
                attempt["error"] = str(exc)
                raise
            finally:
                attempt["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                args.plan.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                signal.signal(signal.SIGINT, previous_int)
                signal.signal(signal.SIGTERM, previous_term)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError,
            KeyError, TypeError, IndexError, can.CanError) as exc:
        print(f"去起点未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
