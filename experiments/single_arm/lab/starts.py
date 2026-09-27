"""命名起点目录与经实机验证的 S1→C2 去程适配。

回 S1 使用 planned_return 从实时七轴角生成 move_j 目标。历史反向
流式路线不再是公开的回位入口。
"""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from experiments.single_arm.continuous_replay.continuous_session import (
    check_plan, require_prior_stage,
)
from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config, validate_joints


STARTS_FILE = Path("experiments/single_arm/lab/config/starts.json")
PLAN_DIR = Path("experiments/single_arm/lab/data/start_plans")
RUN_DIR = Path("experiments/single_arm/lab/data/start_runs")
START_TOLERANCE_RAD = 0.005


def load_starts(config_path=DEFAULT_CONFIG, starts_path=STARTS_FILE):
    config_path, starts_path = Path(config_path), Path(starts_path)
    config = load_config(config_path)
    catalog = json.loads(starts_path.read_text(encoding="utf-8"))
    if (catalog.get("schema_version") != 1 or catalog.get("robot_model") != "NERO"
            or catalog.get("firmware_driver") != "V121"
            or catalog.get("mount") != config["mount"]
            or catalog.get("configuration_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or catalog.get("tool_offset_m") != [0,0,0]):
        raise ValueError("命名起点与当前配置、固件或工具约定不一致")
    starts = catalog["starts"]
    if catalog.get("default_start") != "S1" or set(starts) != {"S1", "C2"}:
        raise ValueError("命名起点表不是经验证的 S1/C2 版本")
    for name, item in starts.items():
        item["joint_rad"] = validate_joints(item["joint_rad"], config["zero_exclusion_radius_rad"])
    if max(abs(a-b) for a,b in zip(starts["S1"]["joint_rad"],
                                    config["safe_start_joint_rad"])) > 1e-9:
        raise ValueError("S1 命名起点与当前示教配置不一致")
    evidence = json.loads(Path(catalog["route_evidence"]).read_text(encoding="utf-8"))
    visit = json.loads(Path(starts["C2"]["source"]).read_text(encoding="utf-8"))
    if (evidence.get("acceptance_pass") is not True
            or visit.get("current_site_arrival_and_hold_verified") is not True
            or visit.get("candidate_joint_rad") != starts["C2"]["joint_rad"]):
        raise ValueError("S1↔C2 的实机路径或候选停稳证据未验收")
    return catalog


def list_starts(config_path=DEFAULT_CONFIG, starts_path=STARTS_FILE):
    catalog = load_starts(config_path, starts_path)
    from experiments.single_arm.lab.optimized_home import HOME_Q
    return {"default_start": catalog["default_start"],
            "site_conditions": catalog["site_conditions"],
            "starts": [{"name": name, **item} for name,item in catalog["starts"].items()] +
            [{"name": "H0", "joint_rad": list(HOME_Q),
              "source": "optimized_home.reduced-j3",
              "route_from": ["S1"],
              "description": "桌边轻触承托位；关节到位后仍保持使能"}]}


def _route(catalog, target, config):
    if target != "C2":
        raise ValueError("S1/H0 请使用 lab.cli starts plan/run 统一命名目标入口")
    source = check_plan(Path(catalog["route_source_plan"]), config)
    forward = [item for item in source["waypoints"] if item["phase"] == "forward"]
    start_q, candidate_q = forward[0]["joint_rad"], forward[-1]["joint_rad"]
    if (max(abs(a-b) for a,b in zip(start_q, catalog["starts"]["S1"]["joint_rad"])) > 0.001
            or max(abs(a-b) for a,b in zip(candidate_q, catalog["starts"]["C2"]["joint_rad"])) > 1e-9):
        raise ValueError("存档轨迹与两个命名起点不符")
    waypoints = [dict(item) for item in forward]
    period = 1/source["frequency_hz"]
    hold_periods = round(source["turnaround_hold_s"]*source["frequency_hz"])
    if hold_periods < source["frequency_hz"]:
        raise ValueError("C2 源路线停稳时间不足 1 秒")
    for index in range(1, hold_periods+1):
        waypoints.append({"t_s": forward[-1]["t_s"]+index*period,
                          "source_t_s": forward[-1]["source_t_s"],
                          "phase": "hold", "joint_rad": list(candidate_q)})
    if any(a["t_s"] >= b["t_s"] for a,b in zip(waypoints,waypoints[1:])):
        raise ValueError("命名起点路线时间不递增")
    return source, waypoints


def prepare_start(target="S1", config_path=DEFAULT_CONFIG, starts_path=STARTS_FILE,
                  robot_factory=None, snapshot=None):
    if target == "S1":
        raise ValueError("回 S1 请使用 lab.cli starts plan/run 统一命名目标入口")
    config_path = Path(config_path)
    config = load_config(config_path)
    catalog = load_starts(config_path, starts_path)
    if target not in catalog["starts"]:
        raise ValueError("未知命名起点")
    if snapshot is None:
        with connected(config, robot_factory) as robot:
            snapshot = read_state(robot)
    require_ready(snapshot)
    current = snapshot["joint_rad"]
    matches = [name for name,item in catalog["starts"].items()
               if max(abs(a-b) for a,b in zip(current, item["joint_rad"])) <= START_TOLERANCE_RAD]
    if len(matches) != 1:
        raise ValueError("实时姿态不在任何已验证命名起点附近；拒绝未知姿态直接跳转")
    source_name = matches[0]
    base = {"schema_version": 1, "kind": "named_start_transfer",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "starts_sha256": hashlib.sha256(Path(starts_path).read_bytes()).hexdigest(),
            "source_name": source_name, "target_name": target,
            "observed_start_joint_rad": current,
            "observed_start_flange_pose_m_rad": snapshot["flange_pose_m_rad"],
            "site_conditions": catalog["site_conditions"],
            "recording": None}
    if source_name == target:
        return {**base, "already_at_target": True, "waypoints": []}
    source, route = _route(catalog, target, config)
    return {**source, **base,
            "recording": source["recording"],
            "source_sha256": source["source_sha256"],
            "source_used_rows": source["source_used_rows"],
            "waypoints": route, "total_duration_s": route[-1]["t_s"],
            "turnaround_hold_s": source["turnaround_hold_s"] if target == "C2" else 0.0,
            "one_way_mode": True, "return_to_start": False,
            "already_at_target": False,
            "route_source_plan": catalog["route_source_plan"],
            "route_evidence": catalog["route_evidence"]}


def verify_start_plan(plan, config_path=DEFAULT_CONFIG, starts_path=STARTS_FILE):
    if plan.get("target_name") == "S1":
        raise ValueError("历史倒序 S1 计划已停用；请重新规划 move_j 回位")
    config_path = Path(config_path)
    config = load_config(config_path)
    catalog = load_starts(config_path, starts_path)
    if (plan.get("schema_version") != 1 or plan.get("kind") != "named_start_transfer"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("starts_sha256") != hashlib.sha256(Path(starts_path).read_bytes()).hexdigest()
            or plan.get("target_name") not in catalog["starts"]
            or plan.get("source_name") not in catalog["starts"]):
        raise ValueError("初始化计划版本、配置或命名起点不匹配")
    observed = validate_joints(plan["observed_start_joint_rad"],
                               config["zero_exclusion_radius_rad"])
    if max(abs(a-b) for a,b in zip(observed,
                      catalog["starts"][plan["source_name"]]["joint_rad"])) > START_TOLERANCE_RAD:
        raise ValueError("初始化计划的起点不在已验证姿态附近")
    if plan["source_name"] == plan["target_name"]:
        if not plan.get("already_at_target") or plan.get("waypoints"):
            raise ValueError("已经到位的初始化计划不得包含运动目标")
        return plan
    source, expected = _route(catalog, plan["target_name"], config)
    actual = plan.get("waypoints")
    if (plan.get("route_source_plan") != catalog["route_source_plan"]
            or plan.get("route_evidence") != catalog["route_evidence"]
            or plan.get("recording") != source["recording"]
            or plan.get("source_sha256") != source["source_sha256"]
            or plan.get("frequency_hz") != source["frequency_hz"]
            or plan.get("time_scale") != source["time_scale"]
            or plan.get("turnaround_hold_s") != (source["turnaround_hold_s"] if plan["target_name"] == "C2" else 0.0)
            or plan.get("total_duration_s") != expected[-1]["t_s"]
            or plan.get("return_to_start") is not False
            or plan.get("one_way_mode") is not True
            or not isinstance(actual, list) or len(actual) != len(expected)):
        raise ValueError("初始化路线来源、完整性或模式不匹配")
    for left,right in zip(actual, expected):
        if (left["phase"] != right["phase"] or abs(left["t_s"]-right["t_s"]) > 1e-9
                or abs(left["source_t_s"]-right["source_t_s"]) > 1e-9
                or max(abs(a-b) for a,b in zip(left["joint_rad"], right["joint_rad"])) > 1e-9):
            raise ValueError("初始化路线目标被修改")
    return plan


def execute_start(plan_path, config_path=DEFAULT_CONFIG, starts_path=STARTS_FILE,
                  robot_factory=None):
    """显式调用才运动；先独立连接只读复核，随后交给已验证的流式执行器。"""
    from experiments.single_arm.continuous_replay.run_stream import run
    plan_path = Path(plan_path)
    plan = verify_start_plan(json.loads(plan_path.read_text(encoding="utf-8")),
                             config_path, starts_path)
    config = load_config(Path(config_path))
    with connected(config, robot_factory) as robot:
        snapshot = require_ready(read_state(robot))
        if max(abs(a-b) for a,b in zip(snapshot["joint_rad"],
                                        plan["observed_start_joint_rad"])) > 0.002:
            raise RuntimeError("实时姿态已偏离初始化计划起点 0.002 rad；请重新规划")
    if plan["already_at_target"]:
        return {"already_at_target": True, "status": snapshot}
    require_prior_stage(plan)
    RUN_DIR.mkdir(parents=True, exist_ok=True)
    audit_path = RUN_DIR / ("nero_start_audit_" + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    audit = {"schema_version":1,"kind":"named_start_transfer_audit",
             "started_at_utc":datetime.now(timezone.utc).isoformat(),
             "plan":str(plan_path),"source_name":plan["source_name"],
             "target_name":plan["target_name"],"start_status":snapshot,
             "completed":False}
    with audit_path.open("x", encoding="utf-8") as stream:
        json.dump(audit, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    try:
        report_path = run(plan, plan_path, config, robot_factory=robot_factory)
        audit["stream_report"] = str(report_path)
        stream_report = json.loads(Path(report_path).read_text(encoding="utf-8"))
        if not stream_report.get("completed") or not stream_report.get("can_joint_frames_complete"):
            raise RuntimeError("连续目标或 CAN 旁路日志不完整；不可将本次初始化记为通过")
        with connected(config, robot_factory) as robot:
            final = require_ready(read_state(robot))
        audit["final_status"] = final
        target = load_starts(config_path, starts_path)["starts"][plan["target_name"]]["joint_rad"]
        error = max(abs(a-b) for a,b in zip(final["joint_rad"], target))
        audit["final_max_joint_error_rad"] = error
        if error > 0.005:
            raise RuntimeError(f"运动结束后命名起点最大关节误差 {error:.6f} rad 超出 0.005 rad")
        audit["completed"] = True
    except Exception as exc:
        audit.update(error=str(exc),error_type=type(exc).__name__)
        raise RuntimeError(f"命名起点转移未验收：{exc}；独立报告 {audit_path}") from exc
    finally:
        audit["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        audit_path.write_text(json.dumps(audit, ensure_ascii=False, indent=2)+"\n",
                              encoding="utf-8")
    return {"run_report": str(report_path), "audit_report":str(audit_path),
            "target_name": plan["target_name"],
            "final_max_joint_error_rad": error, "status": final}


def save_start_plan(plan, directory=PLAN_DIR):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ("nero_start_" + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(plan, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path
