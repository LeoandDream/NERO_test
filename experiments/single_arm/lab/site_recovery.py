"""2026-09-26 当前低位的现场恢复辅助；不属于通用命名起点初始化。

只适用于左侧装、裸法兰、基座 X 正向离桌且现场确认的当前姿态。
每次显式执行一段；失败后拒绝复用计划，须重读状态并重新规划。
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints, validate_route_workspace,
)


PLAN_DIR = Path("experiments/single_arm/lab/data/site_recovery_plans")
RUN_DIR = Path("experiments/single_arm/lab/data/site_recoveries")
MIN_X = (-0.045, 0.005, 0.17, 0.34)
MIN_END_X = (0.015, 0.18, 0.35, 0.37)


def _path(directory, prefix):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    return directory / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")


def _write(path, obj, *, exclusive=False):
    with Path(path).open("x" if exclusive else "w", encoding="utf-8") as stream:
        json.dump(obj, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def _targets(q, config):
    first=list(q); first[1]=q[1]-.1
    second=list(first); second[1]=-.2
    third=list(second); third[1]=config["safe_start_joint_rad"][1]
    return [first,second,third,list(config["safe_start_joint_rad"])]


def _geometry(robot, start, end, config, index):
    points=[]
    for step in range(101):
        q=[a+(b-a)*step/100 for a,b in zip(start,end)]
        validate_joints(q,config["zero_exclusion_radius_rad"])
        xyz=[float(x) for x in robot.fk(q)[:3]]
        if not all(math.isfinite(x) for x in xyz):
            raise ValueError("离线 FK 输出无效")
        points.append(xyz)
    if min(p[0] for p in points) < MIN_X[index] or points[-1][0] < MIN_END_X[index]:
        raise ValueError(f"恢复第 {index+1} 段的法兰 X 不满足离桌门槛")
    if index<3 and any(b[0]<a[0]-.001 for a,b in zip(points,points[1:])):
        raise ValueError("恢复路线没有持续沿 X 正向离桌")
    return {axis:[min(p[k] for p in points),max(p[k] for p in points)]
            for k,axis in enumerate("xyz")}


def prepare(config_path=DEFAULT_CONFIG, robot_factory=None):
    config_path=Path(config_path)
    config=load_config(config_path)
    if config["mount"]!="left":
        raise ValueError("仅对当前左侧装现场规划")
    with connected(config,robot_factory) as robot:
        state=require_ready(read_state(robot))
        q=state["joint_rad"]
        x=state["flange_pose_m_rad"][0]
        if not (.20<=q[1]<=.26 and -.39<=q[3]<=-.30 and -.05<=x<=-.015):
            raise ValueError("当前不是本次确认的低位姿态；拒绝套用恢复路线")
        ends=_targets(q,config)
        geometry=[]
        for index,end in enumerate(ends):
            start=q if index==0 else ends[index-1]
            validate_route_workspace(robot,[start,end],config)
            geometry.append(_geometry(robot,start,end,config,index))
    return {"schema_version":1,"kind":"site_specific_table_recovery",
            "created_at_utc":datetime.now(timezone.utc).isoformat(),
            "config_sha256":hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "observed_start_joint_rad":q,
            "observed_start_flange_pose_m_rad":state["flange_pose_m_rad"],
            "targets_joint_rad":ends,"predicted_flange_xyz_ranges_m":geometry,
            "speed_percent":5,
            "site_assumption":"Left mount, bare flange, unchanged 2026-09-25 table and cable layout; base X positive moves away from table. Confirm on site for every stage.",
            "note":"临时恢复，从未知低位依次 J2 小幅离桌、中间 J2、S1 J2、其余轴对齐；不升级为已验证命名路线。"}


def save(plan,directory=PLAN_DIR):
    path=_path(directory,"nero_site_recovery_")
    _write(path,plan,exclusive=True)
    return path


def verify(plan,config_path=DEFAULT_CONFIG):
    config_path=Path(config_path)
    config=load_config(config_path)
    if (plan.get("schema_version")!=1 or plan.get("kind")!="site_specific_table_recovery"
            or plan.get("config_sha256")!=hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("speed_percent")!=5):
        raise ValueError("恢复计划版本或配置不符")
    q=validate_joints(plan["observed_start_joint_rad"],config["zero_exclusion_radius_rad"])
    pose=plan.get("observed_start_flange_pose_m_rad")
    if (not isinstance(pose,list) or len(pose)!=6
            or not (.20<=q[1]<=.26 and -.39<=q[3]<=-.30
                    and -.05<=pose[0]<=-.015)):
        raise ValueError("恢复计划不是当前左侧装低位的已核对起点")
    if (plan.get("targets_joint_rad")!=_targets(q,config)
            or len(plan.get("predicted_flange_xyz_ranges_m",[]))!=4):
        raise ValueError("恢复目标被修改")
    return plan


def _completed_reports(plan_path,run_dir):
    prefix="nero_site_recovery_"+plan_path.stem+"_stage"
    results={}
    for file in Path(run_dir).glob(prefix+"*.json"):
        report=json.loads(file.read_text(encoding="utf-8"))
        if report.get("plan")!=str(plan_path): continue
        stage=report.get("stage")
        results.setdefault(stage,[]).append(report)
    return results


def run(plan_path,stage,config_path=DEFAULT_CONFIG,robot_factory=None,
        run_dir=RUN_DIR,countdown_s=5):
    plan_path=Path(plan_path)
    plan=verify(json.loads(plan_path.read_text(encoding="utf-8")),config_path)
    if stage not in (1,2,3,4): raise ValueError("恢复阶段须为 1～4")
    previous=_completed_reports(plan_path,run_dir)
    if previous.get(stage): raise ValueError("本段已有执行报告；不可重复使用计划")
    if any(not previous.get(index) or not previous[index][-1].get("completed")
           for index in range(1,stage)):
        raise ValueError("前段未完成；拒绝跳段")
    config=load_config(Path(config_path))
    start=(plan["observed_start_joint_rad"] if stage==1
           else plan["targets_joint_rad"][stage-2])
    target=plan["targets_joint_rad"][stage-1]
    report_path=_path(run_dir,"nero_site_recovery_"+plan_path.stem+f"_stage{stage}_")
    report={"schema_version":1,"kind":"site_specific_table_recovery",
            "started_at_utc":datetime.now(timezone.utc).isoformat(),
            "plan":str(plan_path),"stage":stage,"start_joint_rad":start,
            "target_joint_rad":target,"speed_percent":5,"completed":False,
            "command_published":False,"feedback_samples":[]}
    _write(report_path,report,exclusive=True)
    interrupted=False
    prior=None

    def stop(_number,_frame):
        nonlocal interrupted
        interrupted=True

    try:
        with connected(config,robot_factory) as robot:
            state=require_ready(read_state(robot))
            if max(abs(a-b) for a,b in zip(state["joint_rad"],start))>.002:
                raise RuntimeError("实时姿态偏离本段起点；请重新规划")
            validate_route_workspace(robot,[start,target],config)
            geometry=_geometry(robot,start,target,config,stage-1)
            if geometry!=plan["predicted_flange_xyz_ranges_m"][stage-1]:
                raise ValueError("恢复计划几何重新计算结果不同")
            prior=(signal.signal(signal.SIGINT,stop),signal.signal(signal.SIGTERM,stop))
            for remaining in range(countdown_s,0,-1):
                if interrupted: raise InterruptedError("倒计时取消")
                print(f"{remaining} 秒后现场恢复第 {stage}/4 段；按 Ctrl+C 取消。",flush=True)
                time.sleep(1)
            state=require_ready(read_state(robot))
            if max(abs(a-b) for a,b in zip(state["joint_rad"],start))>.002:
                raise RuntimeError("倒计时期间姿态改变")
            begun=time.monotonic()

            def feedback(joints):
                x=float(robot.fk(joints)[0])
                report["feedback_samples"].append({
                    "elapsed_s":round(time.monotonic()-begun,3),
                    "joint_rad":list(joints),"flange_x_m":x})
                if x<MIN_X[stage-1]-.005:
                    raise RuntimeError("实测法兰 X 靠近桌面区域，停止本段")

            execute_route(robot,config,[start,target],lambda:interrupted,
                          speed_percent=5,target_tolerance_rad=.005,
                          waypoint_timeout_s=20,waypoint_max_timeout_s=120,
                          pause_on_stationary_timeout=True,
                          path_mode="joint_box",joint_box_margin_rad=.03,
                          on_command_published=lambda *_:report.update(command_published=True),
                          on_feedback=feedback)
            final=require_ready(read_state(robot))
            error=max(abs(a-b) for a,b in zip(final["joint_rad"],target))
            if error>.005: raise RuntimeError("本段目标误差超过 0.005 rad")
            report.update(completed=True,final_status=final,
                          final_max_joint_error_rad=error)
    except Exception as exc:
        report.update(error=str(exc),error_type=type(exc).__name__)
        raise RuntimeError(f"现场恢复第 {stage} 段未完成：{exc}；报告 {report_path}") from exc
    finally:
        if prior is not None:
            signal.signal(signal.SIGINT,prior[0]);signal.signal(signal.SIGTERM,prior[1])
        report["finished_at_utc"]=datetime.now(timezone.utc).isoformat()
        _write(report_path,report)
        print("本段报告：",report_path,flush=True)
    return report_path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=DEFAULT_CONFIG)
    parser.add_argument("--plan",type=Path)
    parser.add_argument("--stage",type=int,choices=(1,2,3,4))
    parser.add_argument("--run",action="store_true")
    args=parser.parse_args(argv)
    try:
        if args.plan is None:
            if args.run or args.stage: parser.error("先生成只读计划，再指定 --plan/--stage")
            plan=prepare(args.config)
            path=save(plan)
            print("只读恢复计划：",path)
        else:
            path=args.plan
            plan=verify(json.loads(path.read_text(encoding="utf-8")),args.config)
        for index,(target,geometry) in enumerate(zip(
                plan["targets_joint_rad"],plan["predicted_flange_xyz_ranges_m"]),1):
            print(f"第 {index}/4 段目标 (rad)：{target}；法兰预测范围 (m)：{geometry}")
        if args.run:
            if args.stage is None: parser.error("执行必须指定 --stage")
            run(path,args.stage,args.config)
        else:
            print("只读规划/复核完成；未发送运动指令。")
        return 0
    except (OSError,ValueError,RuntimeError,TimeoutError,InterruptedError) as exc:
        print(f"现场恢复未完成：{exc}",file=sys.stderr,flush=True)
        return 1


if __name__=="__main__":
    raise SystemExit(main())
