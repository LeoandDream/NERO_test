#!/usr/bin/env python3
"""从 S1 逐段试验多方向 move_p；生成计划完全离线，--run 每次只发一条命令。"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.control_interfaces.cartesian_linear_session import (
    checked_pose, run_one_line,
)
from experiments.single_arm.control_interfaces.ik_preview import pose_error, solve_nearby_pose
from experiments.single_arm.start_transfer.go_to_start import stable_joint_feedback
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, JOINT_LIMITS, check_drivers,
    healthy_status, load_config, validate_joints,
)


ROOT = Path("experiments/single_arm/control_interfaces")
SOURCE_SHORT = Path("experiments/single_arm/teaching/data/recordings/nero_session_20260923T145952Z.csv")
SOURCE_WIDE = Path("experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z_teaching_only.csv")
PLAN_DIR = ROOT / "data/plans"
RUN_DIR = ROOT / "data/runs"
POSE_FIELDS = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
# 2026-09-25 第 7 段实机碰到桌面。在建立并验证现场环境模型前，
# 禁止此演示入口继续发送任何目标，包括重新生成计划后的目标。
MOTION_SUSPENDED_AFTER_CONTACT = True


def create_robot(config):
    """构造 SDK 对象；离线规划只调用 fk，绝不调用 connect。"""
    return AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
        channel=config["channel"], local_loopback=True,
    ))


def recorded_pose(path, sample_index):
    """用指定采样的原始六维法兰位姿，防止把示教关节终点误作 P 模式路径。"""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if sample_index >= len(rows):
        raise ValueError(f"记录 {path} 缺少样本 {sample_index}")
    row = rows[sample_index]
    pose = [float(row[name]) for name in POSE_FIELDS]
    joints = [float(row[f"joint_{index}_rad"]) for index in range(1, 8)]
    return pose, joints


def preview_reachable(robot, start_joints, start_pose, target_pose, zero_radius):
    """每约 2 mm 求近邻 IK，仅筛查候选终点；不能预测控制器 move_p 的路线。"""
    distance = math.dist(start_pose[:3], target_pose[:3])
    steps = max(2, math.ceil(distance / 0.002))
    route = [list(start_joints)]
    for index in range(1, steps + 1):
        fraction = index / steps
        interpolated = [a + fraction * (b-a) for a,b in zip(start_pose, target_pose)]
        route.append(solve_nearby_pose(robot, route[-1], interpolated, zero_radius))
    margin = min(min(q-lower, upper-q) for point in route
                 for q,(lower,upper) in zip(point, JOINT_LIMITS))
    if margin < 0.15:
        raise ValueError(f"候选终点的近邻 IK 关节限位余量仅 {margin:.3f} rad")
    return route, margin


def build_plan(config, output, profile="initial"):
    """依据已录到的空间坐标选取多方向候选，并离线求近邻解。"""
    robot = create_robot(config)
    s1_joint = validate_joints(config["safe_start_joint_rad"],
                               config["zero_exclusion_radius_rad"])
    s1_pose = [float(value) for value in robot.fk(s1_joint)]
    taught_pose, taught_joint = recorded_pose(SOURCE_SHORT, 195)
    if math.dist(robot.fk(taught_joint)[:3], taught_pose[:3]) > 0.005:
        raise ValueError("示教样本的关节 FK 与法兰位置不符")
    # 20 秒记录只用于限定曾观测到的三维位置范围；其回放曾急停，
    # 因此不把那条轨迹或其姿态直接作为 P 模式可执行路径。
    with SOURCE_WIDE.open(newline="", encoding="utf-8") as stream:
        wide = list(csv.DictReader(stream))
    with SOURCE_SHORT.open(newline="", encoding="utf-8") as stream:
        short = list(csv.DictReader(stream))
    samples = wide + short
    bounds = {axis: [min(float(row[axis]) for row in samples),
                     max(float(row[axis]) for row in samples)]
              for axis in ("x_m", "y_m", "z_m")}
    candidates = [
        ("taught_y_plus", taught_pose, "5 秒示教第 195 条实测六维法兰姿态"),
        ("y_minus_40mm", [s1_pose[0], s1_pose[1]-0.040, s1_pose[2], *s1_pose[3:]],
         "基于示教三维范围选点，姿态保持 S1"),
        ("x_minus_40mm", [s1_pose[0]-0.040, s1_pose[1], s1_pose[2], *s1_pose[3:]],
         "基于示教三维范围选点，姿态保持 S1"),
        ("z_minus_30mm", [s1_pose[0], s1_pose[1], s1_pose[2]-0.030, *s1_pose[3:]],
         "基于示教三维范围选点，姿态保持 S1"),
        ("xyz_diagonal", [s1_pose[0]-0.030, s1_pose[1]-0.020,
                          s1_pose[2]+0.010, *s1_pose[3:]],
         "三轴同时变化；位置位于已示教三维范围，姿态保持 S1"),
    ]
    if profile == "wide":
        # 宽范围阶段采用已录到的完整六维位姿作为空间对角点；
        # 20 秒记录的逐点回放曾失败，不能把该记录当成 P 控制器的中间路径。
        wide_249, wide_249_joints = recorded_pose(SOURCE_WIDE, 249)
        wide_748, wide_748_joints = recorded_pose(SOURCE_WIDE, 748)
        for name, pose, joints in (("taught_diagonal_114mm",wide_249,wide_249_joints),
                                   ("taught_lateral_92mm",wide_748,wide_748_joints)):
            if math.dist(robot.fk(joints)[:3],pose[:3]) > 0.005:
                raise ValueError(f"{name} 的示教关节 FK 与法兰记录不符")
        candidates = [
            ("y_minus_80mm",[s1_pose[0],s1_pose[1]-0.080,s1_pose[2],*s1_pose[3:]],
             "示教覆盖的 Y 负向范围；保持 S1 姿态"),
            ("taught_diagonal_114mm",wide_249,"20 秒示教第 249 条实测六维法兰姿态"),
            ("taught_lateral_92mm",wide_748,"20 秒示教第 748 条实测六维法兰姿态"),
            ("x_minus_60mm",[s1_pose[0]-0.060,s1_pose[1],s1_pose[2],*s1_pose[3:]],
             "示教覆盖的 X 负向范围；保持 S1 姿态"),
            ("z_minus_45mm",[s1_pose[0],s1_pose[1],s1_pose[2]-0.045,*s1_pose[3:]],
             "示教覆盖的 Z 负向范围；保持 S1 姿态"),
        ]
    targets = []
    for name, pose, origin in candidates:
        for axis,value in zip(("x_m","y_m","z_m"),pose[:3]):
            if not bounds[axis][0] <= value <= bounds[axis][1]:
                raise ValueError(f"{name} 的 {axis} 未落入示教记录的观测范围")
        route, margin = preview_reachable(robot,s1_joint,s1_pose,pose,
                                          config["zero_exclusion_radius_rad"])
        targets.append({
            "name":name,"source":origin,"pose":pose,
            "translation_m":math.dist(s1_pose[:3],pose[:3]),
            "preview_joint_rad":route,
            "preview_max_joint_change_rad":max(abs(a-b) for point in route
                                                for a,b in zip(point,s1_joint)),
            "preview_min_joint_margin_rad":margin,
        })
    plan = {
        "schema":"nero_point_workspace_demo_v1",
        "profile":profile,
        "created_at_utc":datetime.now(timezone.utc).isoformat(),
        "source_recordings": {str(path):hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in (SOURCE_SHORT,SOURCE_WIDE)},
        "observed_flange_coordinate_ranges_m":bounds,
        "s1_joint_rad":s1_joint,"s1_pose":s1_pose,
        "targets":targets,
        "notes":"每段从 S1 出发后返回 S1；每次 --run 只发一个 move_p。观测范围按轴统计，不证明三维组合安全；本机 IK 不预测控制器路径。",
    }
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(plan,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    return plan


def guard_limits(target):
    """按计划路程放大监测包络，避免沿用 5 mm 首次试验的门槛误触发。"""
    translation = target["translation_m"]
    angular = max(abs(value) for value in pose_error(target["pose"],target["start_pose"])[3:])
    return {
        "max_flange_excursion_m":min(0.30,max(0.035,2*translation+0.020)),
        "max_rotation_change_rad":min(1.5,max(0.08,2*angular+0.05)),
        "max_joint_change_rad":min(1.4,max(0.25,2*target["preview_max_joint_change_rad"]+0.10)),
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,default=DEFAULT_CONFIG)
    parser.add_argument("--build-plan",action="store_true",help="完全离线生成多方向候选，不连接 CAN")
    parser.add_argument("--profile",choices=("initial","wide"),default="initial",
                        help="初始多方向目标或 80～114 mm 宽范围目标；仅用于 --build-plan")
    parser.add_argument("--plan",type=Path,help="已生成的 JSON 计划")
    parser.add_argument("--leg",type=int,help="1～10：每个候选分别去程和回 S1")
    parser.add_argument("--run",action="store_true",help="显式发送本段唯一一条 move_p")
    args=parser.parse_args()
    if args.build_plan:
        if args.leg or args.run:
            parser.error("离线生成计划不能带 --leg 或 --run")
        output=args.plan or PLAN_DIR/("nero_point_demo_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+".json")
        plan=build_plan(load_config(args.config),output,args.profile)
        print("离线计划：",output)
        for index,target in enumerate(plan["targets"],1):
            print(f"目标 {index}: {target['name']}，{target['translation_m']*1000:.1f} mm，"
                  f"近邻 IK 最大关节变化 {target['preview_max_joint_change_rad']:.3f} rad，"
                  f"最小限位余量 {target['preview_min_joint_margin_rad']:.3f} rad")
        print("未连接 CAN；未发送运动指令。")
        return 0
    if args.plan is None or args.leg is None:
        parser.error("实时预检需同时指定 --plan 与 --leg")
    if args.run and MOTION_SUSPENDED_AFTER_CONTACT:
        parser.error("多方向点位演示因桌面接触已暂停；不能发送运动命令")
    robot=None
    try:
        config=load_config(args.config)
        plan=json.loads(args.plan.read_text(encoding="utf-8"))
        if plan.get("schema")!="nero_point_workspace_demo_v1":
            raise ValueError("计划格式不符")
        if plan.get("halted_after_contact"):
            raise RuntimeError("该点位计划发生桌面接触，已停用；不得继续执行任何段")
        if plan["s1_joint_rad"]!=config["safe_start_joint_rad"]:
            raise ValueError("S1 配置已改变，须重建计划")
        for filename,digest in plan["source_recordings"].items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=digest:
                raise ValueError(f"原始记录已改变：{filename}")
        if not 1<=args.leg<=2*len(plan["targets"]):
            raise ValueError("段编号超出计划")
        index=(args.leg-1)//2
        outward=args.leg%2==1
        target=plan["targets"][index]
        robot=create_robot(config)
        robot.connect()
        if wait_status(robot,fresh,timeout=3.0) is None:
            raise RuntimeError("控制器状态反馈缺失")
        healthy_status(robot,{1},CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        current_joint=validate_joints(stable_joint_feedback(robot),config["zero_exclusion_radius_rad"])
        current_pose=checked_pose(robot.get_flange_pose())
        expected_start=plan["s1_pose"] if outward else target["pose"]
        expected_joint=plan["s1_joint_rad"] if outward else target["preview_joint_rad"][-1]
        if math.dist(current_pose[:3],expected_start[:3])>0.003 or max(
            abs(value) for value in pose_error(current_pose,expected_start)[3:]
        )>0.04:
            raise ValueError("实时法兰起点与本段计划不符；重新核对现场和计划")
        if outward and max(abs(a-b) for a,b in zip(current_joint,expected_joint))>0.03:
            raise ValueError("实时关节未在 S1，拒绝外行程")
        goal=target["pose"] if outward else plan["s1_pose"]
        # 实时重新求近邻解，只筛查终点；后续不按预测关节折线管制 P 模式。
        route,margin=preview_reachable(robot,current_joint,current_pose,goal,
                                       config["zero_exclusion_radius_rad"])
        preview_max=max(abs(a-b) for point in route for a,b in zip(point,current_joint))
        limit_target={"pose":goal,"start_pose":current_pose,
                      "translation_m":math.dist(current_pose[:3],goal[:3]),
                      "preview_max_joint_change_rad":preview_max}
        limits=guard_limits(limit_target)
        print(f"只读预检：第 {args.leg}/{2*len(plan['targets'])} 段，"
              f"{'S1→'+target['name'] if outward else target['name']+'→S1'}")
        print("实时起点法兰位姿：",current_pose)
        print("单次 move_p 目标：",goal)
        print(f"平移 {limit_target['translation_m']*1000:.1f} mm；"
              f"近邻 IK 最大关节变化 {preview_max:.3f} rad；"
              f"最小限位余量 {margin:.3f} rad；速度 5%")
        print("实测反馈停止门槛：",limits)
        print("move_p 的控制器中间路径未知；请核对所有连杆、线缆与支撑的可能活动范围。")
        if not args.run:
            print("只读预检完成；未发送运动指令。")
            return 0
        stopped=False
        def on_signal(_signum,_frame):
            nonlocal stopped
            stopped=True
        old_int=signal.signal(signal.SIGINT,on_signal)
        old_term=signal.signal(signal.SIGTERM,on_signal)
        report={"plan":str(args.plan),"leg":args.leg,"target_name":target["name"],
                "outward":outward,"start_joint_rad":current_joint,
                "start_flange_pose":current_pose,"target_flange_pose":goal,
                "speed_percent":5,"control_command":"move_p", "command_count":0,
                "completed":False,"limits":limits,
                "started_at_utc":datetime.now(timezone.utc).isoformat()}
        try:
            for remaining in range(5,0,-1):
                if stopped:
                    raise InterruptedError("倒计时取消；未发送命令")
                print(f"{remaining} 秒后发送本段单次 move_p；按 Ctrl+C 取消。",flush=True)
                time.sleep(1)
            healthy_status(robot,{1},CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            now=stable_joint_feedback(robot)
            if max(abs(a-b) for a,b in zip(now,current_joint))>0.005:
                raise RuntimeError("倒计时期间起点变化；未发送命令")
            run_one_line(robot,current_pose,goal,route,5,report,lambda:stopped,
                         expected_joint_rad=plan["s1_joint_rad"] if not outward else None,
                         zero_radius_rad=config["zero_exclusion_radius_rad"],motion_mode="p",
                         timeout_s=90.0,**limits)
            print(f"第 {args.leg} 段到位；法兰位置误差 {report['position_error_m']:.6f} m")
        except Exception as exc:
            report["error"]=str(exc)
            raise
        finally:
            signal.signal(signal.SIGINT,old_int)
            signal.signal(signal.SIGTERM,old_term)
            report["finished_at_utc"]=datetime.now(timezone.utc).isoformat()
            RUN_DIR.mkdir(parents=True,exist_ok=True)
            output=RUN_DIR/("nero_point_demo_leg"+str(args.leg)+"_"+
                            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+".json")
            output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
            print("本段报告：",output,flush=True)
        return 0
    except (OSError,ValueError,RuntimeError,TimeoutError,InterruptedError,can.CanError) as exc:
        print(f"点位演示未完成：{exc}",file=sys.stderr,flush=True)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__=="__main__":
    raise SystemExit(main())
