"""S1 邻域的整数度、基座 X–Z 平面姿态试验。

候选姿态只用于现场外观评估。计划与运行都不会改写公共 S1；
第 1 段到候选，第 2 段沿相同的小范围关节目标返回 S1。
"""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.start_transfer.geometry.urdf_origins import (
    link_origins, parse_joint_origins,
)
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints, validate_route_workspace,
)


CANDIDATE_DEG = [0, -33, 0, -17, 0, 12, -27]
PLAN_DIR = Path("experiments/single_arm/lab/data/pose_candidate_plans")
RUN_DIR = Path("experiments/single_arm/lab/data/pose_candidate_runs")
START_TOLERANCE_RAD = 0.005
RUN_START_TOLERANCE_RAD = 0.002
FINAL_TOLERANCE_RAD = 0.005


def _utc():
    return datetime.now(timezone.utc).isoformat()


def _new_path(directory, prefix):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    return directory / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")


def _write(path, value, *, exclusive=False):
    with Path(path).open("x" if exclusive else "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def analyze(config_path=DEFAULT_CONFIG):
    """离线计算关节插值、法兰和各连杆原点；不连接 CAN。"""
    config = load_config(Path(config_path))
    start = config["safe_start_joint_rad"]
    candidate = [math.radians(value) for value in CANDIDATE_DEG]
    max_change = max(abs(a-b) for a,b in zip(start,candidate))
    if max_change > 0.035:
        raise ValueError("候选姿态与公共 S1 的单关节变化超过 0.035 rad")
    robot = robot_instance(config["channel"])
    model = parse_joint_origins()
    envelope = [[math.inf, -math.inf] for _ in range(3)]
    max_flange_shift = 0.0
    max_link_origin_shift = 0.0
    start_flange = [float(x) for x in robot.fk(start)]
    start_links = link_origins(start, model)
    for step in range(101):
        alpha = step / 100
        joints = validate_joints([a + alpha*(b-a) for a,b in zip(start,candidate)],
                                 config["zero_exclusion_radius_rad"])
        pose = robot.fk(joints)
        links = link_origins(joints, model)
        max_flange_shift = max(max_flange_shift, math.dist(pose[:3],start_flange[:3]))
        max_link_origin_shift = max(max_link_origin_shift, *(
            math.dist(position,start_links[name]) for name,position in links.items()))
        for axis in range(3):
            envelope[axis][0] = min(envelope[axis][0],float(pose[axis]))
            envelope[axis][1] = max(envelope[axis][1],float(pose[axis]))
    if max_flange_shift > 0.025 or max_link_origin_shift > 0.025:
        raise ValueError("候选路径超出 S1 的 25 mm 离线邻域")
    candidate_links = link_origins(candidate,model)
    # X–Z 平面指主要连杆原点 link1..link6。法兰有固定的 Y 向结构偏置。
    if max(abs(candidate_links[f"link{i}"][1]) for i in range(1,7)) > 1e-5:
        raise ValueError("候选的主要连杆原点不在基座 X–Z 平面")
    return {"start_joint_rad":start,"candidate_joint_deg":list(CANDIDATE_DEG),
            "candidate_joint_rad":candidate,
            "start_flange_pose_m_rad":start_flange,
            "candidate_flange_pose_m_rad":[float(x) for x in robot.fk(candidate)],
            "candidate_link_origin_y_m":{name:position[1] for name,position in candidate_links.items()},
            "flange_xyz_envelope_m":dict(zip("xyz",envelope)),
            "max_joint_change_rad":max_change,
            "max_flange_shift_m":max_flange_shift,
            "max_link_origin_shift_m":max_link_origin_shift,
            "interpolation_samples":101,
            "environment_checked":False,
            "actual_controller_path_checked":False}


def prepare(config_path=DEFAULT_CONFIG, *, robot_factory=None, snapshot=None):
    config_path = Path(config_path)
    config = load_config(config_path)
    if snapshot is None:
        with connected(config,robot_factory) as robot:
            snapshot = read_state(robot)
    require_ready(snapshot)
    geometry = analyze(config_path)
    if max(abs(a-b) for a,b in zip(snapshot["joint_rad"],geometry["start_joint_rad"])) > START_TOLERANCE_RAD:
        raise ValueError("实时姿态不在公共 S1 附近；不能规划候选试验")
    return {"schema_version":1,"kind":"s1_integer_xz_candidate",
            "created_at_utc":_utc(),
            "config_sha256":hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "observed_start_joint_rad":snapshot["joint_rad"],
            "geometry":geometry,"speed_percent":5,
            "stages":["S1→整数度候选","整数度候选→S1"],
            "default_start_changed":False}


def save(plan, directory=PLAN_DIR):
    path = _new_path(directory,"nero_pose_candidate_")
    _write(path,plan,exclusive=True)
    return path


def verify(plan, config_path=DEFAULT_CONFIG):
    config_path=Path(config_path)
    expected=analyze(config_path)
    if (plan.get("schema_version") != 1 or plan.get("kind") != "s1_integer_xz_candidate"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("geometry") != expected or plan.get("speed_percent") != 5
            or plan.get("default_start_changed") is not False
            or plan.get("stages") != ["S1→整数度候选","整数度候选→S1"]):
        raise ValueError("整数度姿态计划与固定候选或当前配置不符")
    observed=plan["observed_start_joint_rad"]
    if len(observed)!=7 or max(abs(a-b) for a,b in zip(observed,expected["start_joint_rad"])) > START_TOLERANCE_RAD:
        raise ValueError("候选计划的原始起点不在 S1 附近")
    return plan


def run(plan_path, stage, *, config_path=DEFAULT_CONFIG, robot_factory=None,
        run_dir=RUN_DIR, countdown_s=5):
    if stage not in (1,2):
        raise ValueError("候选姿态只允许第 1 或第 2 段")
    plan_path=Path(plan_path)
    plan=verify(json.loads(plan_path.read_text(encoding="utf-8")),config_path)
    geometry=plan["geometry"]
    source=(geometry["start_joint_rad"] if stage==1 else geometry["candidate_joint_rad"])
    target=(geometry["candidate_joint_rad"] if stage==1 else geometry["start_joint_rad"])
    report_path=_new_path(run_dir,f"nero_pose_candidate_stage{stage}_")
    report={"schema_version":1,"kind":"s1_integer_xz_candidate_run",
            "started_at_utc":_utc(),"plan":str(plan_path),"stage":stage,
            "source_joint_rad":source,"target_joint_rad":target,
            "completed":False,"last_attempted_step":0,"last_published_step":0,
            "last_reached_step":0,"feedback_samples":[],"status_samples":[]}
    _write(report_path,report,exclusive=True)
    config=load_config(Path(config_path))
    old_handlers=None
    interrupted=False

    def stop(_number,_frame):
        nonlocal interrupted
        interrupted=True

    try:
        with connected(config,robot_factory) as robot:
            start_state=require_ready(read_state(robot))
            report["start_status"]=start_state
            if max(abs(a-b) for a,b in zip(start_state["joint_rad"],source)) > RUN_START_TOLERANCE_RAD:
                raise RuntimeError("实时姿态偏离本段候选计划起点；请重新规划")
            route=[start_state["joint_rad"],target]
            validate_route_workspace(robot,route,config)
            old_handlers=(signal.signal(signal.SIGINT,stop),signal.signal(signal.SIGTERM,stop))
            for remaining in range(countdown_s,0,-1):
                if interrupted:raise InterruptedError("倒计时取消")
                print(f"{remaining} 秒后执行候选姿态第 {stage} 段；按 Ctrl+C 取消。",flush=True)
                time.sleep(1)
            second=require_ready(read_state(robot))
            if max(abs(a-b) for a,b in zip(second["joint_rad"],source)) > RUN_START_TOLERANCE_RAD:
                raise RuntimeError("倒计时期间起点改变")
            route[0]=second["joint_rad"]
            started=time.monotonic()
            execute_route(robot,config,route,lambda:interrupted,
                          speed_percent=5,target_tolerance_rad=FINAL_TOLERANCE_RAD,
                          waypoint_timeout_s=5,waypoint_max_timeout_s=15,
                          pause_on_stationary_timeout=True,
                          path_mode="joint_box",joint_box_margin_rad=.03,
                          on_command=lambda index,_:report.update(last_attempted_step=index),
                          on_command_published=lambda index,_:report.update(last_published_step=index),
                          on_waypoint=lambda index,_:report.update(last_reached_step=index),
                          on_feedback=lambda joints:report["feedback_samples"].append({
                              "elapsed_s":round(time.monotonic()-started,3),"joint_rad":list(joints)}),
                          on_status=lambda status:report["status_samples"].append({
                              "elapsed_s":round(time.monotonic()-started,3),
                              "arm_status":int(status.msg.arm_status),
                              "motion_status":int(status.msg.motion_status)}))
        # 重新建立连接，独立核对控制器状态与最终七轴反馈。
        with connected(config,robot_factory) as robot:
            final=require_ready(read_state(robot))
        error=max(abs(a-b) for a,b in zip(final["joint_rad"],target))
        report.update(final_status=final,final_max_joint_error_rad=error)
        if error > FINAL_TOLERANCE_RAD:
            raise RuntimeError("候选姿态最终误差超过 0.005 rad")
        report["completed"]=True
    except Exception as exc:
        report.update(error=str(exc),error_type=type(exc).__name__)
        raise RuntimeError(f"候选姿态第 {stage} 段未完成：{exc}；报告 {report_path}") from exc
    finally:
        if old_handlers is not None:
            signal.signal(signal.SIGINT,old_handlers[0])
            signal.signal(signal.SIGTERM,old_handlers[1])
        report["finished_at_utc"]=_utc()
        _write(report_path,report)
    return {"report_path":str(report_path),"stage":stage,
            "final_max_joint_error_rad":error,"status":final}
