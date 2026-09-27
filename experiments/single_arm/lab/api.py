"""单臂实验的组合入口。导入时不建立 CAN 连接，不隐式使能或运动。

所有距离为 m、关节角为 rad、时长为 s；动作分为 ``plan_*`` 和
``run_*``。后者必须收到由前者保存的计划路径。
"""

from argparse import Namespace
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

from experiments.single_arm.lab import cube, joint_probe, planned_return, pose_candidate, recordings, replay, starts
from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, check_start, load_config, run_session,
)


def h0_start_strategy(state):
    """Recognize only the two calibrated H0 states; route code checks again."""
    from experiments.single_arm.lab.optimized_home import HOME_Q
    from experiments.single_arm.lab.start_from_h0_candidate import SUPPORTED_Q

    joints = state.get("joint_rad", [])
    drivers = state.get("drivers", [])
    if (len(joints) != 7 or len(drivers) != 7 or
            state.get("control_mode") != 1 or state.get("error_code") != 0 or
            any(d.get("undervoltage") or d.get("driver_error") for d in drivers)):
        return None
    # Only choose the specialist here. Its preflight checks stability again and
    # retries a transient sample; a single noisy sample must not route H0 to
    # the generic S1 planner, whose flange-X floor intentionally rejects H0.
    if (state.get("arm_status") == 0 and
            all(d.get("enabled") for d in drivers) and
            max(abs(a-b) for a, b in zip(joints, HOME_Q)) <= 0.01):
        return "enabled"
    if (state.get("arm_status") == 6 and
            all(not d.get("enabled") for d in drivers) and
            max(abs(a-b) for a, b in zip(joints, SUPPORTED_Q)) <= 0.01):
        return "supported"
    return None


class LabAPI:
    """可注入 robot_factory 的统一接口；只有显式方法才连接/运动。"""

    def __init__(self, config_path=DEFAULT_CONFIG, *, robot_factory=None):
        self.config_path = Path(config_path)
        self.robot_factory = robot_factory

    def status(self):
        """只读七轴、法兰、控制器与停稳状态。"""
        config = load_config(self.config_path)
        with connected(config, self.robot_factory) as robot:
            return read_state(robot)

    def record_poses(self, *, duration_s=5, rate_hz=5, output_dir=None,
                     required_state=None):
        """只读记录状态、七轴与法兰；历史文件以独占方式写入。"""
        from experiments.single_arm.can_setup.can_drag_teach import fresh
        from experiments.single_arm.pose_recording.record_poses import FIELDNAMES
        if not (math.isfinite(duration_s) and 0 < duration_s <= 1800
                and math.isfinite(rate_hz) and 0 < rate_hz <= 20):
            raise ValueError("记录时长须在 0～1800 s，频率须在 0～20 Hz")
        if required_state not in (None, "drag"):
            raise ValueError("未知的位姿记录状态要求")
        path = Path(output_dir or "experiments/single_arm/pose_recording/data/recordings") / (
            "nero_poses_lab_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".csv")
        path.parent.mkdir(parents=True, exist_ok=True)
        count = 0
        config = load_config(self.config_path)
        with connected(config, self.robot_factory) as robot, path.open(
                "x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
            writer.writeheader()
            started = time.monotonic()
            tick = started
            last_valid = started
            while time.monotonic() - started < duration_s:
                joints = robot.get_joint_angles()
                pose = robot.get_flange_pose()
                status = robot.get_arm_status()
                if all(fresh(item, 0.5) for item in (joints, pose, status)):
                    if required_state == "drag" and (
                        int(status.msg.ctrl_mode) != 2
                        or int(status.msg.arm_status) != 0
                        or int(status.msg.teach_status) != 1
                        or status.msg.err_code
                    ):
                        raise RuntimeError(f"边界录制中拖动状态异常；部分原始 CSV 保留在 {path}")
                    q, xyzrpy = list(joints.msg), list(pose.msg)
                    if len(q) == 7 and len(xyzrpy) == 6:
                        row = {"sample_index": count,
                               "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                               "elapsed_s": round(time.monotonic()-started, 3),
                               "joint_feedback_unix_s": float(joints.timestamp),
                               "pose_feedback_unix_s": float(pose.timestamp),
                               "status_feedback_unix_s": float(status.timestamp)}
                        for column, attribute in (("control_mode","ctrl_mode"),
                                                  ("arm_status","arm_status"),
                                                  ("teach_status","teach_status"),
                                                  ("motion_status","motion_status")):
                            value=getattr(status.msg,attribute)
                            row[column]=getattr(value,"name",str(value))
                        row.update(zip((f"joint_{i}_rad" for i in range(1,8)), q))
                        row.update(zip(("x_m","y_m","z_m","roll_rad",
                                        "pitch_rad","yaw_rad"), xyzrpy))
                        writer.writerow(row)
                        stream.flush()
                        count += 1
                        last_valid = time.monotonic()
                if required_state == "drag" and time.monotonic() - last_valid > 1.0:
                    raise RuntimeError(f"边界录制连续一秒没有有效反馈；部分原始 CSV 保留在 {path}")
                tick += 1/rate_hz
                time.sleep(max(0, tick-time.monotonic()))
        if count == 0:
            raise RuntimeError(f"未收到有效反馈；空记录保留在 {path}")
        return {"recording_path": str(path), "samples": count,
                "duration_s": duration_s, "rate_hz": rate_hz,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    def home_demo(self, *, duration_s=30, rate_hz=20, command_runner=None,
                  data_dir=None):
        """Record a hand-guided supported-home demonstration without returning."""
        from experiments.single_arm.lab.home_demo import record_home_demo
        return record_home_demo(self, duration_s=duration_s, rate_hz=rate_hz,
                                command_runner=command_runner, data_dir=data_dir)

    def plan_supported_home(self, target_name="H0"):
        """Plan a sparse joint route between S1 and the supported H0 pose."""
        from experiments.single_arm.lab.supported_home import prepare
        return prepare(target_name, config_path=self.config_path,
                       robot_factory=self.robot_factory)

    def run_supported_home(self, plan_path):
        """Recheck a saved S1/H0 route and explicitly issue move_j targets."""
        from experiments.single_arm.lab.supported_home import run
        return {"report_path": str(run(plan_path, config_path=self.config_path,
                                       robot_factory=self.robot_factory))}

    def recordings(self):
        """离线列出当前配置下完整且封存的示教记录，最近在前。"""
        return [item.as_dict() for item in recordings.list_recordings(self.config_path)]

    def validate_recording(self, selector="latest"):
        return recordings.resolve_recording(selector, self.config_path).as_dict()

    def teach_preflight(self, max_seconds=20):
        if not math.isfinite(max_seconds) or not 1 <= max_seconds <= 300:
            raise ValueError("示教最长时间须在 1～300 s")
        config = load_config(self.config_path)
        with connected(config, self.robot_factory) as robot:
            start = check_start(robot, config)
            state = require_ready(read_state(robot))
        return {"start_name": config["safe_start_name"],
                "start_joint_rad": start, "max_seconds": max_seconds,
                "config_sha256": hashlib.sha256(self.config_path.read_bytes()).hexdigest(),
                "status": state, "motion_sent": False}

    def teach_run(self, max_seconds=20):
        """显式开启 CAN 拖动示教；结束后从停稳的实时姿态规划回 S1。"""
        self.teach_preflight(max_seconds)
        output = recordings.RECORDING_DIR / (
            "nero_session_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".csv")
        config = load_config(self.config_path)
        args = Namespace(config=self.config_path, output=output,
                         max_seconds=float(max_seconds), run=True)
        try:
            with connected(config, self.robot_factory) as robot:
                run_session(robot, config, args)
        except Exception as exc:
            raise RuntimeError(f"示教未完成：{exc}；原始 CSV {output}，元数据 {output.with_suffix('.json')}") from exc
        return recordings.validate_recording(output, self.config_path).as_dict()

    def plan_replay(self, selector="latest", *, mode=None, speed_percent=None,
                    frequency_hz=None, min_time_scale=None):
        if mode == "point":
            if frequency_hz is not None or min_time_scale is not None:
                raise ValueError("逐点回放使用 speed_percent，不使用 frequency_hz 或 min_time_scale")
            plan = replay.prepare_replay(selector, mode=mode, config_path=self.config_path,
                                         speed_percent=5 if speed_percent is None else speed_percent,
                                         robot_factory=self.robot_factory)
        elif mode == "continuous":
            if speed_percent is not None:
                raise ValueError("定时连续回放使用 frequency_hz，不使用 speed_percent")
            from experiments.single_arm.lab.smooth_replay import build_continuous_plan
            plan = build_continuous_plan(selector, config_path=self.config_path,
                                         frequency_hz=20 if frequency_hz is None else frequency_hz,
                                         min_time_scale=6.0 if min_time_scale is None else min_time_scale,
                                         robot_factory=self.robot_factory)
        else:
            raise ValueError("回放模式必须是 point 或 continuous")
        return {"plan_path": str(replay.save_replay_plan(plan)), "plan": plan}

    def run_replay(self, plan_path):
        plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
        if plan.get("replay_mode") == "point_move_j":
            result = replay.execute_replay(plan_path, config_path=self.config_path,
                                           robot_factory=self.robot_factory)
        elif plan.get("replay_mode") == "timed_move_j":
            from experiments.single_arm.lab.smooth_replay import execute_continuous_plan
            result = execute_continuous_plan(plan_path, config_path=self.config_path,
                                             robot_factory=self.robot_factory)
        else:
            raise ValueError("未知回放模式；拒绝发送运动指令")
        return {"report_path": str(result)}

    def quick_replay(self, selector="latest", *, mode=None, speed_percent=None,
                     frequency_hz=None, min_time_scale=None):
        """单次调用内完成记录与实时预检、保存计划、重新核对并回放一轮。"""
        planned = self.plan_replay(selector, mode=mode, speed_percent=speed_percent,
                                   frequency_hz=frequency_hz,
                                   min_time_scale=min_time_scale)
        executed = self.run_replay(planned["plan_path"])
        return {"recording_id":planned["plan"]["recording_id"],
                "replay_mode":planned["plan"]["replay_mode"],
                "plan_path":planned["plan_path"],**executed}

    def starts(self):
        return starts.list_starts(self.config_path)

    def plan_start(self, target="S1"):
        if target in ("S1", "H0"):
            state = self.status()
            h0_state = h0_start_strategy(state)
            if target == "H0" and h0_state:
                plan = {"schema_version": 1, "kind": "named_joint_target_hold",
                        "target_name": "H0", "already_at_target": True,
                        "controller_targets": 0,
                        "config_sha256": hashlib.sha256(
                            self.config_path.read_bytes()).hexdigest(),
                        "start_joint_rad": state["joint_rad"]}
                return {"plan_path": str(starts.save_start_plan(plan)),
                        "plan": plan}
            if target == "H0" or h0_state:
                from experiments.single_arm.lab.optimized_home import run as optimized_run
                from experiments.single_arm.lab.start_from_h0_candidate import run as supported_run
                if target == "H0":
                    kind, direction = "optimized_home", "home"
                    preview_path = optimized_run(
                        direction, execute=False, config_path=self.config_path,
                        robot_factory=self.robot_factory)
                elif h0_state == "enabled":
                    kind, direction = "optimized_home", "start"
                    preview_path = optimized_run(
                        direction, execute=False, config_path=self.config_path,
                        robot_factory=self.robot_factory)
                else:
                    kind, direction = "supported_h0_start", "start"
                    preview_path = supported_run(
                        execute=False, config_path=self.config_path,
                        robot_factory=self.robot_factory)
                preview_bytes = Path(preview_path).read_bytes()
                preview = json.loads(preview_bytes)
                expected_kind = ("optimized_sparse_s1_h0_trial"
                                 if kind == "optimized_home" else
                                 "h0_candidate_to_s1_move_j_trial")
                if (preview.get("kind") != expected_kind or
                        preview.get("execution_requested") is not False or
                        preview.get("preflight_passed") is not True or
                        preview.get("completed") or
                        (kind == "optimized_home" and
                         preview.get("direction") != direction)):
                    raise ValueError("命名目标的只读预检报告不完整")
                route = preview["route_joint_rad"]
                plan = {"schema_version": 1, "kind": "named_joint_target_plan",
                        "target_name": target, "route_kind": kind,
                        "direction": direction,
                        "strategy": preview.get("strategy"),
                        "config_sha256": hashlib.sha256(
                            self.config_path.read_bytes()).hexdigest(),
                        "preview_report_path": str(preview_path),
                        "preview_sha256": hashlib.sha256(preview_bytes).hexdigest(),
                        "start_joint_rad": route[0],
                        "route_joint_rad": route,
                        "controller_targets": len(route)-1,
                        "already_at_target": False}
                return {"plan_path": str(starts.save_start_plan(plan)),
                        "plan": plan}
            return self.plan_return()
        plan = starts.prepare_start(target, self.config_path,
                                    robot_factory=self.robot_factory)
        return {"plan_path": str(starts.save_start_plan(plan)), "plan": plan}

    def run_start(self, plan_path):
        plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
        if plan.get("kind") in ("named_joint_target_plan",
                                "named_joint_target_hold"):
            return self._run_named_joint_target(plan)
        if plan.get("kind") == "live_planned_return_to_s1":
            return self.run_return(plan_path)
        return starts.execute_start(plan_path, self.config_path,
                                    robot_factory=self.robot_factory)

    def _run_named_joint_target(self, plan):
        """Rebuild the saved named route from live feedback before execution."""
        from experiments.single_arm.lab.optimized_home import plan as optimized_plan
        from experiments.single_arm.lab.optimized_home import run as optimized_run
        from experiments.single_arm.lab.start_from_h0_candidate import (
            _check_supported_state, plan as supported_plan, run as supported_run,
        )
        config = load_config(self.config_path)
        if (plan.get("schema_version") != 1 or
                plan.get("config_sha256") != hashlib.sha256(
                    self.config_path.read_bytes()).hexdigest()):
            raise ValueError("命名目标计划与当前配置不匹配")
        if plan["kind"] == "named_joint_target_hold":
            state = self.status()
            if (plan.get("target_name") != "H0" or
                    not plan.get("already_at_target") or
                    plan.get("controller_targets") != 0 or
                    not h0_start_strategy(state) or
                    max(abs(a-b) for a,b in zip(
                        state["joint_rad"], plan["start_joint_rad"])) > 0.003):
                raise ValueError("H0 停留计划与实时姿态不符")
            return {"already_at_target": True, "status": state}
        kind = plan.get("route_kind")
        target = plan.get("target_name")
        direction = plan.get("direction")
        if not ((kind == "optimized_home" and
                 (target, direction) in (("H0", "home"), ("S1", "start")) and
                 plan.get("strategy") in ("validated", "reduced-j3")) or
                (kind == "supported_h0_start" and target == "S1" and
                 direction == "start" and plan.get("strategy") is None)):
            raise ValueError("命名目标、方向或规划器不匹配")
        preview_bytes = Path(plan["preview_report_path"]).read_bytes()
        preview = json.loads(preview_bytes)
        expected_kind = ("optimized_sparse_s1_h0_trial"
                         if kind == "optimized_home" else
                         "h0_candidate_to_s1_move_j_trial")
        if (hashlib.sha256(preview_bytes).hexdigest() != plan.get("preview_sha256") or
                preview.get("kind") != expected_kind or
                preview.get("execution_requested") is not False or
                preview.get("preflight_passed") is not True or
                preview.get("completed") or
                preview.get("route_joint_rad") != plan.get("route_joint_rad") or
                (kind == "optimized_home" and
                 (preview.get("direction") != direction or
                  preview.get("strategy") != plan.get("strategy")))):
            raise ValueError("只读预检报告与保存的路线不一致")
        with connected(config, self.robot_factory) as robot:
            state = read_state(robot)
            if kind == "optimized_home":
                current = optimized_plan(robot, state, config, direction,
                                         strategy=plan["strategy"])["route_joint_rad"]
            else:
                _check_supported_state(robot, state, config)
                current, _ = supported_plan(robot, state, config)
        saved = plan.get("route_joint_rad", [])
        # 起点来自两次独立编码器采样；从 H0 启动时第一步只动 J1，
        # 其余六轴也沿用新的实时采样值。后续固定目标仍须逐值一致。
        live_prefix = 2 if kind == "optimized_home" and direction == "start" else 1
        if (len(saved) != len(current) or
                plan.get("controller_targets") != len(current)-1 or
                not all(len(a) == len(b) == 7 and
                        max(abs(x-y) for x,y in zip(a,b)) <=
                        (0.003 if i < live_prefix else 1e-6)
                        for i, (a,b) in enumerate(zip(saved,current)))):
            raise RuntimeError("实时重规划与保存的命名路线不符；未发送运动")
        kwargs = {"execute": True, "config_path": self.config_path,
                  "robot_factory": self.robot_factory}
        report_path = (optimized_run(direction, strategy=plan["strategy"], **kwargs)
                       if kind == "optimized_home" else supported_run(**kwargs))
        return {"report_path": str(report_path), "target_name": target}

    def quick_start(self, target="S1"):
        """Use the same saved named-target plan for S1, H0 and C2."""
        planned=self.plan_start(target)
        if planned["plan"].get("already_at_target"):
            return {"plan_path": planned["plan_path"], "already_at_target": True}
        return {"plan_path":planned["plan_path"],
                "result":self.run_start(planned["plan_path"])}


    def plan_return(self, *, min_flange_x_m=0.15):
        """从当前姿态独立规划回 S1；不读取示教记录或发送运动命令。"""
        plan = planned_return.prepare(self.config_path, min_flange_x_m,
                                      self.robot_factory)
        plan["already_at_target"] = plan["controller_targets"] == 0
        path = planned_return.save_plan(plan)
        return {"plan_path": str(path), "plan": plan}

    def run_return(self, plan_path):
        """重新读取实时状态并复核计划，显式执行回 S1。"""
        return {"report_path": str(planned_return.run(
            plan_path, self.config_path, self.robot_factory))}

    def plan_pose_candidate(self):
        plan=pose_candidate.prepare(self.config_path,robot_factory=self.robot_factory)
        return {"plan_path":str(pose_candidate.save(plan)),"plan":plan}

    def run_pose_candidate(self, plan_path, stage):
        return pose_candidate.run(plan_path,stage,config_path=self.config_path,
                                  robot_factory=self.robot_factory)

    def cube_new(self):
        return {"session_path": str(cube.create_session(self.config_path))}

    def cube_capture(self, session_path, face):
        path = cube.capture_face(session_path, face, self.config_path, self.robot_factory)
        sample = json.loads(Path(path).read_text(encoding="utf-8"))
        return {"sample_path":str(path),"face":face,
                "flange_xyz_base_m":sample["flange_pose_m_rad"][:3]}

    def cube_fit(self, session_path):
        output, result = cube.fit_session(session_path, self.config_path)
        return {"fit_path": str(output), "fit": result}

    def cube_status(self, session_path):
        return cube.session_status(session_path, self.config_path)

    def cube_extrema_candidate(self, map_csv, *,
                               level="new_full_continuous_path_sample",
                               margin_m=0.01, bottom_margin_m=None):
        """离线从一种实测轨迹提取极值；只生成几何候选，不连接机器人。"""
        candidate = cube.extrema_candidate(
            map_csv, level=level, margin_m=margin_m,
            bottom_margin_m=bottom_margin_m)
        path = cube.save_extrema_candidate(candidate)
        return {"candidate_path": str(path), "candidate": candidate}

    def cube_extrema_from_recording(self, recording_csv, *, margin_m=0.01,
                                    bottom_margin_m=None):
        """把一条正常拖动 CSV 转成极值地图和几何候选；不连接机器人。"""
        map_path = cube.map_from_drag_recording(recording_csv)
        result = self.cube_extrema_candidate(
            map_path, level="new_hand_guided_sweep", margin_m=margin_m,
            bottom_margin_m=bottom_margin_m)
        return {"map_path": str(map_path), **result}

    def cube_sweep(self, *, duration_s=30, rate_hz=20, margin_m=0.01,
                   command_runner=None, report_dir=None):
        """一次调用完成拖动边界录制和极值分析；始终退出拖动，不自动回位。"""
        from experiments.single_arm.lab.cube_sweep import run_sweep
        return run_sweep(self, duration_s=duration_s, rate_hz=rate_hz,
                         margin_m=margin_m, command_runner=command_runner,
                         report_dir=report_dir)

    def cube_face_sweep(self, *, seconds_per_face=12, rate_hz=20,
                        phase_gate=None, command_runner=None, report_dir=None,
                        session_dir=None, session_path=None):
        """逐面限时采样并拟合；可续录缺面，不回位或启用运动边界。"""
        from experiments.single_arm.lab.cube_face_sweep import run_face_sweep
        return run_face_sweep(self, seconds_per_face=seconds_per_face,
                              rate_hz=rate_hz, phase_gate=phase_gate,
                              command_runner=command_runner,
                              report_dir=report_dir, session_dir=session_dir,
                              session_path=session_path)

    def reachability_summary(self):
        """只读实测证据分级；包围盒不代表内部或连杆可安全运动。"""
        from experiments.single_arm.reachability.build_map import (
            historic_rows, old_controlled_rows, new_probe_rows,
            new_continuous_rows, summary,
        )
        rows = list(historic_rows()) + list(old_controlled_rows())
        rows += list(new_probe_rows()) + list(new_continuous_rows())
        return summary(rows)

    def plan_probe(self, *, axis, delta_m, anchor_recording=None):
        """单段 2～10 mm 法兰直线探测；复用现有只读校验器。"""
        from experiments.single_arm.reachability.controlled_probe import (
            PLAN_DIR, create_plan,
        )
        config = load_config(self.config_path)
        with connected(config, self.robot_factory) as robot:
            plan = create_plan(robot, config, self.config_path, axis, delta_m,
                               Path(anchor_recording) if anchor_recording else None)
        path = Path(PLAN_DIR) / ("nero_lab_probe_" + datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%S%fZ") + ".json")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(plan, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        return {"plan_path": str(path), "plan": plan}

    def run_probe(self, plan_path):
        """显式发送单次 move_l；执行器按原有逻辑保存反馈和失败报告。"""
        from experiments.single_arm.reachability.controlled_probe import check_plan, execute
        path = Path(plan_path)
        plan = json.loads(path.read_text(encoding="utf-8"))
        config = load_config(self.config_path)
        with connected(config, self.robot_factory) as robot:
            check_plan(robot, plan, config, self.config_path)
            return {"report_path": str(execute(robot, plan, path, config,
                                                self.config_path))}

    def plan_joint_probe(self, *, speed_percent=5):
        plan = joint_probe.prepare(config_path=self.config_path,
                                   robot_factory=self.robot_factory,
                                   speed_percent=speed_percent)
        return {"plan_path":str(joint_probe.save(plan)),"plan":plan}

    def run_joint_probe(self, plan_path):
        return {"report_path":str(joint_probe.run(
            plan_path,config_path=self.config_path,robot_factory=self.robot_factory))}

    def plan_continuous_archive(self, *, frequency_hz=20):
        """仅对历史固定 20 秒源显式规划定时连续回放。"""
        from experiments.single_arm.continuous_replay.continuous_session import (
            PLAN_DIR, create_plan,
        )
        from experiments.single_arm.continuous_replay.trajectory import DEFAULT_SOURCE
        if frequency_hz not in (10, 20):
            raise ValueError("连续回放频率仅支持 10/20 Hz")
        config = load_config(self.config_path)
        plan = create_plan(DEFAULT_SOURCE, config, frequency_hz=frequency_hz)
        path = Path(PLAN_DIR) / ("nero_lab_continuous_" + datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%S%fZ") + ".json")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            json.dump(plan, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        return {"plan_path": str(path), "plan": plan,
                "mode": "timed_joint_stream_historical_20s"}

    def run_continuous_archive(self, plan_path):
        from experiments.single_arm.continuous_replay.continuous_session import (
            check_plan, require_prior_stage,
        )
        from experiments.single_arm.continuous_replay.run_stream import run
        config = load_config(self.config_path)
        plan = check_plan(Path(plan_path), config)
        require_prior_stage(plan)
        return {"report_path": str(run(plan, Path(plan_path), config,
                                       robot_factory=self.robot_factory)),
                "mode": "timed_joint_stream_historical_20s"}
