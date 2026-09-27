"""统一单臂实验 CLI：plan/只读为默认，run 为显式动作。"""

import argparse
import json
from pathlib import Path
import sys
import time

from experiments.single_arm.lab.api import LabAPI
from experiments.single_arm.lab.cube import FACES
from experiments.single_arm.can_setup.drag_start_guard import require_drag_start_safe
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG


def parser_create():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    groups = parser.add_subparsers(dest="group", required=True)
    groups.add_parser("status", help="只读状态、七轴、法兰与停稳检查")
    poses = groups.add_parser("poses", help="只读位置记录")
    poses.add_argument("--duration", type=float, default=5)
    poses.add_argument("--rate", type=float, default=5)
    teach = groups.add_parser("teach", help="示教预检或显式运行")
    teach.add_argument("--max-seconds", type=float, default=20)
    teach.add_argument("--run", action="store_true")
    records = groups.add_parser("records", help="已封存示教记录")
    records.add_argument("selector", nargs="?", default=None,
                         help="省略则列出全部；latest/ID/目录内 CSV 则验证一份")
    replay = groups.add_parser("replay", help="新示教逐点或定时关节回放")
    replay_sub = replay.add_subparsers(dest="action", required=True)
    replay_plan = replay_sub.add_parser("plan", help="只读预检并保存计划")
    replay_plan.add_argument("--recording", default="latest")
    replay_plan.add_argument("--mode", choices=("point", "continuous"), required=True,
                             help="明确选择逐点或定时连续关节回放")
    replay_plan.add_argument("--speed-percent", type=int, choices=(5,10))
    replay_plan.add_argument("--frequency-hz", type=int, choices=(10,20))
    replay_plan.add_argument("--min-time-scale", type=float,
                             help="仅连续回放使用；至少 6，实际倍率可因限速而更大")
    replay_run = replay_sub.add_parser("run", help="显式执行已复核计划")
    replay_run.add_argument("--plan", type=Path, required=True)
    starts = groups.add_parser("starts", help="命名起点")
    start_sub = starts.add_subparsers(dest="action", required=True)
    start_sub.add_parser("list")
    start_plan = start_sub.add_parser("plan", help="只读实时姿态与计划")
    start_plan.add_argument("--target", choices=("S1", "C2", "H0"), default="S1")
    start_run = start_sub.add_parser("run", help="显式到命名起点；S1 从实时姿态重规划")
    start_run.add_argument("--plan", type=Path, required=True)
    live_return = groups.add_parser("return", help="从实时姿态新规划回 S1；不倒放示教记录")
    return_sub = live_return.add_subparsers(dest="action", required=True)
    return_plan = return_sub.add_parser("plan", help="只读生成关节目标与法兰范围")
    return_plan.add_argument("--min-flange-x-m", type=float, default=0.15)
    return_run = return_sub.add_parser("run", help="显式执行并重新核对实时起点")
    return_run.add_argument("--plan", type=Path, required=True)
    cube = groups.add_parser("cube", help="六面立方体几何标定；不授予运动许可")
    cube_sub = cube.add_subparsers(dest="action", required=True)
    cube_sub.add_parser("new", help="建立空采样会话")
    capture = cube_sub.add_parser("capture", help="采集当前停稳法兰点")
    capture.add_argument("--session", type=Path, required=True)
    capture.add_argument("--face", choices=FACES, required=True)
    fit = cube_sub.add_parser("fit", help="离线六面拟合")
    fit.add_argument("--session", type=Path, required=True)
    cube_status = cube_sub.add_parser("status", help="只读查看六面采样覆盖")
    cube_status.add_argument("--session", type=Path, required=True)
    guide = cube_sub.add_parser("guide", help="交互引导六面采样；仅采集，不切换拖动模式")
    guide.add_argument("--session", type=Path, help="已有会话；省略则新建")
    extrema = cube_sub.add_parser("extrema", help="离线从实测轨迹极值生成立方体候选")
    extrema_source = extrema.add_mutually_exclusive_group(required=True)
    extrema_source.add_argument("--map", type=Path, help="已有实测地图 CSV")
    extrema_source.add_argument("--recording", type=Path,
                                help="刚录制的正常拖动原始 CSV")
    extrema.add_argument("--level", default=None,
                         help="只使用一种地图证据级别")
    extrema.add_argument("--margin-m", type=float, default=0.01,
                         help="最窄轴两端各内缩的距离，默认 0.01 m")
    extrema.add_argument("--bottom-margin-m", type=float,
                         help="左侧安装的桌面侧 −X 额外内缩；须不小于普通余量")
    probe = groups.add_parser("probe", help="单段 2～10 mm move_l 探测")
    probe_sub = probe.add_subparsers(dest="action", required=True)
    probe_plan = probe_sub.add_parser("plan")
    probe_plan.add_argument("--axis", choices=("x","y","z"), required=True)
    probe_plan.add_argument("--delta-m", type=float, required=True)
    probe_plan.add_argument("--anchor-recording", type=Path)
    probe_run = probe_sub.add_parser("run")
    probe_run.add_argument("--plan", type=Path, required=True)
    joint = groups.add_parser("joint-probe", help="J4 +0.05 rad 后回起点；仅 J4 正角姿态")
    joint_sub = joint.add_subparsers(dest="action", required=True)
    joint_plan = joint_sub.add_parser("plan")
    joint_plan.add_argument("--speed-percent", type=int, choices=(5,10), default=5)
    joint_run = joint_sub.add_parser("run")
    joint_run.add_argument("--plan", type=Path, required=True)
    groups.add_parser("map", help="只读实测可达数据，按证据级别返回")
    continuous = groups.add_parser("continuous-archive", help="仅固定旧 20 秒源的时标连续回放")
    continuous_sub = continuous.add_subparsers(dest="action", required=True)
    continuous_plan = continuous_sub.add_parser("plan")
    continuous_plan.add_argument("--frequency-hz", type=int, choices=(10,20), default=20)
    continuous_run = continuous_sub.add_parser("run")
    continuous_run.add_argument("--plan", type=Path, required=True)
    quick = groups.add_parser("quick", help="一个命令内预检、键盘确认并执行示教/回放/边界录制")
    quick_sub = quick.add_subparsers(dest="action", required=True)
    for action in ("teach", "replay", "demo", "init", "cube-sweep", "cube-faces", "home-demo", "park"):
        item = quick_sub.add_parser(action)
        item.add_argument("--yes", action="store_true",
                          help="非交互脚本已明确授权本次动作；仍执行全部实时预检")
        if action in ("teach", "demo"):
            item.add_argument("--max-seconds", type=float,
                              help="最多拖动秒数；交互默认 5 秒")
        if action in ("replay", "demo"):
            item.add_argument("--mode", choices=("point", "continuous"),
                              help="point 逐点或 continuous 定时关节流；省略时在交互终端选择")
            item.add_argument("--speed-percent", type=int, choices=(5,10),
                              help="仅逐点回放使用；交互默认 5")
            item.add_argument("--frequency-hz", type=int, choices=(10,20),
                              help="仅连续回放使用；交互默认 20 Hz")
            item.add_argument("--min-time-scale", type=float,
                              help="仅连续回放使用；交互默认 6，实际倍率可更大")
        if action == "replay":
            item.add_argument("--recording", help="latest、记录 ID 或 CSV 路径")
        if action == "init":
            item.add_argument("--target", choices=("S1","C2","H0"),
                              help="已验证的命名起点；交互时可从列表选择")
        if action == "cube-sweep":
            item.add_argument("--duration-s", type=float,
                              help="拖动边界录制时长；交互默认 30 秒")
            item.add_argument("--rate-hz", type=float,
                              help="拖动反馈采样频率；交互默认 20 Hz")
            item.add_argument("--margin-m", type=float,
                              help="几何候选内缩余量；交互默认 0.01 m")
        if action == "cube-faces":
            item.add_argument("--seconds-per-face", type=float,
                              help="每个面的拖动录制时长；交互默认 12 秒")
            item.add_argument("--rate-hz", type=float,
                              help="法兰反馈采样频率；交互默认 20 Hz")
            item.add_argument("--session", type=Path,
                              help="续录已有六面会话；只跳过已有至少三个点的面")
        if action == "home-demo":
            item.add_argument("--duration-s", type=float,
                              help="拖动回零路线录制时长；交互默认 30 秒")
            item.add_argument("--rate-hz", type=float,
                              help="关节和法兰反馈采样频率；交互默认 20 Hz")
        if action == "park":
            item.add_argument("--target", choices=("H0", "S1"),
                              help="H0 为桌边实体支撑位；S1 为带电实验起点")
    candidate = groups.add_parser("pose-candidate", help="S1 邻域整数度 X–Z 平面外观候选")
    candidate_sub = candidate.add_subparsers(dest="action", required=True)
    candidate_sub.add_parser("plan", help="只读规划，不改变公共 S1")
    candidate_run = candidate_sub.add_parser("run", help="显式执行一段候选试验")
    candidate_run.add_argument("--plan", type=Path, required=True)
    candidate_run.add_argument("--stage", type=int, choices=(1,2), required=True)
    return parser


def _confirm(prompt, *, approved=False, is_tty=None, input_fn=input):
    """交互终端要求键入“开始”；无终端时必须有显式 --yes。"""
    if approved:
        return
    if is_tty is None:
        is_tty = sys.stdin.isatty()
    if not is_tty:
        raise ValueError("非交互执行须给全参数并显式加 --yes")
    print(prompt, flush=True)
    if input_fn("确认现场条件后键入“开始”执行；其他输入取消：").strip() != "开始":
        raise InterruptedError("用户取消，未发送本步动作")


def quick_dispatch(args, api, *, is_tty=None, input_fn=input):
    """人用入口：预览和执行在同一命令，动作前保留键盘确认。"""
    tty = sys.stdin.isatty() if is_tty is None else is_tty
    if args.yes and ((args.action in ("teach","demo") and args.max_seconds is None)
                     or (args.action in ("replay","demo")
                         and (args.mode is None
                              or (args.mode == "point" and args.speed_percent is None)
                              or (args.mode == "continuous" and
                                  (args.frequency_hz is None or args.min_time_scale is None))))
                     or (args.action == "replay" and args.recording is None)
                     or (args.action == "init" and args.target is None)
                     or (args.action == "cube-sweep" and
                         (args.duration_s is None or args.rate_hz is None
                          or args.margin_m is None))
                     or (args.action == "cube-faces" and
                         (args.seconds_per_face is None or args.rate_hz is None))
                     or (args.action == "home-demo" and
                         (args.duration_s is None or args.rate_hz is None))
                     or (args.action == "park" and args.target is None)):
        raise ValueError("--yes 须明确指定示教时长、记录、回放模式及速度等本步参数")
    if not tty and not args.yes:
        raise ValueError("非交互执行须给全参数并显式加 --yes")
    result = {}
    if args.action == "home-demo":
        seconds = args.duration_s if args.duration_s is not None else 30
        rate = args.rate_hz if args.rate_hz is not None else 20
        print(f"回零路线演示：拖动录制 {seconds:g} 秒、{rate:g} Hz；"
              "结束后退出拖动并停在实际终点，不自动回 S1 或失能。", flush=True)
        _confirm("请扶稳机械臂、清空连杆与线缆活动范围；"
                 "拖到拟定的实体支撑位并在终点停稳至少 2 秒。",
                 approved=args.yes, is_tty=tty, input_fn=input_fn)
        return api.home_demo(duration_s=seconds, rate_hz=rate)
    if args.action == "cube-faces":
        seconds = args.seconds_per_face if args.seconds_per_face is not None else 12
        rate = args.rate_hz if args.rate_hz is not None else 20
        session_status = api.cube_status(args.session) if args.session else None
        faces_to_record = ([face for face in FACES
                            if session_status["face_sample_counts"][face] < 3]
                           if session_status else list(FACES))
        print(f"本次待录制：{','.join(faces_to_record) or '无'}；"
              f"每面在三个分散位置各停稳至少 0.3 秒。"
              f"每面单独开启/结束拖动，录制约 {len(faces_to_record)*seconds:g} 秒；"
              "每面启动命令内核对实时状态和七轴；"
              "等待准备时不处于拖动模式，完成后不自动回 S1。",flush=True)
        _confirm("请扶稳机械臂并清空所有连杆、法兰和线缆活动范围。",
                 approved=args.yes,is_tty=tty,input_fn=input_fn)

        def phase_gate(face):
            if tty and not args.yes:
                answer=input_fn(f"扶稳机械臂，准备拖动到 {face} 面后按回车；"
                                "输入 q 取消：").strip().lower()
                if answer != "":
                    raise InterruptedError("用户取消六面采样；已采文件保留")
            else:
                print(f"5 秒后进入 {face} 面拖动并立即录制；"
                      "请移到该面并在三个位置各停稳。",
                      flush=True)
                time.sleep(5)

        return api.cube_face_sweep(seconds_per_face=seconds,rate_hz=rate,
                                    phase_gate=phase_gate,
                                    session_path=args.session)
    if args.action == "cube-sweep":
        seconds = args.duration_s if args.duration_s is not None else 30
        rate = args.rate_hz if args.rate_hz is not None else 20
        margin = args.margin_m if args.margin_m is not None else 0.01
        state = api.status()
        if not state["ready_for_motion"]:
            raise RuntimeError("边界拖动预检未通过：" + "；".join(state["reasons"]))
        print("边界录制预检：", json.dumps(display_result(state), ensure_ascii=False),
              flush=True)
        print(f"拟录制 {seconds:g} 秒、{rate:g} Hz；只生成几何候选，"
              "结束时退出拖动，不自动回 S1。", flush=True)
        _confirm("请扶稳机械臂、清空期望边界和线缆活动范围；确认后开始拖动录制。",
                 approved=args.yes, is_tty=tty, input_fn=input_fn)
        return api.cube_sweep(duration_s=seconds, rate_hz=rate, margin_m=margin)
    if args.action in ("init", "park"):
        target = args.target or ("H0" if args.action == "park" else None)
        if target is None:
            catalog=api.starts()
            print("可选起点：",flush=True)
            for item in catalog["starts"]:
                print(item["name"],item["joint_rad"],flush=True)
            target=input_fn("输入目标名称 S1、C2 或 H0：").strip().upper()
            if target not in ("S1","C2","H0"):
                raise ValueError("必须选择已验证的 S1、C2 或 H0")
        planned=api.plan_start(target)
        print("命名目标预检：",json.dumps(display_result(planned),ensure_ascii=False),flush=True)
        if planned["plan"]["already_at_target"]:
            return {"plan_path":planned["plan_path"],"already_at_target":True}
        prompt = ("将从 S1 沿经双向验收的 move_j 路线回 H0，"
                  "最后仅 J1 以 2% 速度靠近桌边；到位后保持使能。"
                  "请确认完整活动范围及终点实体承托。"
                  if target == "H0" else
                  "将用 move_j 到达命名起点；请确认底座、全部连杆、"
                  "桌面、线缆和支撑的完整活动范围。")
        _confirm(prompt,
                 approved=args.yes,is_tty=tty,input_fn=input_fn)
        return {"plan_path":planned["plan_path"],
                "result":api.run_start(planned["plan_path"])}
    if args.action in ("teach","demo"):
        seconds = args.max_seconds if args.max_seconds is not None else 5
        preview = api.teach_preflight(seconds)
        print("示教预检：",json.dumps(display_result(preview),ensure_ascii=False),flush=True)
        _confirm("拖动示教将限时结束并从停稳的实时姿态规划回 S1；请扶稳机械臂并检查完整范围。",
                 approved=args.yes,is_tty=tty,input_fn=input_fn)
        result["teaching"] = api.teach_run(seconds)
        print("示教完成，记录 ID：",result["teaching"]["recording_id"],flush=True)
    if args.action in ("replay","demo"):
        mode = args.mode
        if mode is None:
            print("回放方式：1 连续 move_j（20 Hz，已有 5 秒记录实机验收）；"
                  "2 逐点 move_j（每步到位后再发下一步）。",flush=True)
            answer=input_fn("选择 1/2；直接回车选连续：").strip()
            if answer in ("", "1"):
                mode="continuous"
            elif answer == "2":
                mode="point"
            else:
                raise ValueError("回放方式必须选 1 或 2；未生成或执行回放计划")
        if mode == "continuous" and args.speed_percent is not None:
            raise ValueError("连续回放不能传 --speed-percent；用 --frequency-hz")
        if mode == "point" and (args.frequency_hz is not None
                                or args.min_time_scale is not None):
            raise ValueError("逐点回放不能传 --frequency-hz 或 --min-time-scale；用 --speed-percent")
        if args.action == "replay":
            selector = args.recording
            if selector is None:
                recent = api.recordings()
                if not recent:
                    raise ValueError("当前配置下没有完整有效的新示教记录")
                print("可选示教记录（最近优先）：",flush=True)
                for item in recent[:10]:
                    print(item["recording_id"],item["path"],flush=True)
                selector = input_fn(
                    "记录 ID/CSV 路径；直接回车检查最近一次示教，"
                    "若该次失败则拒绝回放：").strip() or "latest"
        else:
            selector = result["teaching"]["recording_id"]
        if mode == "point":
            speed = args.speed_percent if args.speed_percent is not None else 5
            planned = api.plan_replay(selector,mode=mode,speed_percent=speed)
        else:
            frequency = args.frequency_hz if args.frequency_hz is not None else 20
            min_scale = args.min_time_scale if args.min_time_scale is not None else 6.0
            planned = api.plan_replay(selector,mode=mode,frequency_hz=frequency,
                                      min_time_scale=min_scale)
        print("回放预检：",json.dumps(display_result(planned),ensure_ascii=False),flush=True)
        _confirm("关节回放将沿本次记录正向运行并原路返回；请检查连杆、线缆、桌面和支撑的全程范围。",
                 approved=args.yes,is_tty=tty,input_fn=input_fn)
        result["replay"]={"plan_path":planned["plan_path"],
                          **api.run_replay(planned["plan_path"])}
    return result


def cube_guide(args, api, *, is_tty=None, input_fn=input):
    """由人示教六面点位，程序只读采样并反推立方体几何。"""
    tty = sys.stdin.isatty() if is_tty is None else is_tty
    if not tty:
        raise ValueError("cube guide 需要交互终端；脚本采样请用 cube capture --face")
    session = str(args.session) if args.session else api.cube_new()["session_path"]
    print("标定会话：",session,flush=True)
    print("六个标签表示虚拟立方体自身的相对面；请把法兰中心依次拖到每个面的不同位置。",
          flush=True)
    print("同一面的三个点应大致共面且分散、不共线；对面应大致平行，三方向的间距应接近。",
          flush=True)
    print("每点停稳后按回车采样，输入 q 保存已采点并结束；程序从点位反推边长与位置。",
          flush=True)
    for face in FACES:
        while True:
            status=api.cube_status(session)
            count=status["face_sample_counts"][face]
            if count>=3:
                break
            answer=input_fn(f"{face} 面第 {count+1}/3 点：移动并停稳后按回车采样（q 结束）：").strip().lower()
            if answer == "q":
                return {"session_path":session,"status":api.cube_status(session),
                        "partial":True}
            if answer:
                print("未采样：只接受回车或 q。",flush=True)
                continue
            sample=api.cube_capture(session,face)
            print("已保存：",sample["sample_path"],
                  "法兰 XYZ (m)：",sample.get("flange_xyz_base_m"),flush=True)
    fitted=api.cube_fit(session)
    return {"session_path":session,"status":api.cube_status(session),
            "fit_path":fitted["fit_path"],"fit":fitted["fit"],"partial":False}


def cube_extrema_dispatch(args, api):
    """按输入来源选择证据级别；原始拖动 CSV 不能冒充旧地图。"""
    extra = ({"bottom_margin_m": args.bottom_margin_m}
             if args.bottom_margin_m is not None else {})
    if args.recording:
        if args.level is not None:
            raise ValueError("--recording 自带 new_hand_guided_sweep 级别，不接受 --level")
        return api.cube_extrema_from_recording(
            args.recording, margin_m=args.margin_m, **extra)
    return api.cube_extrema_candidate(
        args.map, level=args.level or "new_full_continuous_path_sample",
        margin_m=args.margin_m, **extra)


def dispatch(args, api):
    if args.group == "status":
        return api.status()
    if args.group == "poses":
        return api.record_poses(duration_s=args.duration, rate_hz=args.rate)
    if args.group == "teach":
        return (api.teach_run(args.max_seconds) if args.run
                else api.teach_preflight(args.max_seconds))
    if args.group == "records":
        return api.validate_recording(args.selector) if args.selector else api.recordings()
    if args.group == "replay":
        return (api.plan_replay(args.recording, mode=args.mode,
                                speed_percent=args.speed_percent,
                                frequency_hz=args.frequency_hz,
                                min_time_scale=args.min_time_scale)
                if args.action == "plan" else api.run_replay(args.plan))
    if args.group == "starts":
        return (api.starts() if args.action == "list" else
                api.plan_start(args.target) if args.action == "plan" else
                api.run_start(args.plan))
    if args.group == "return":
        return (api.plan_return(min_flange_x_m=args.min_flange_x_m)
                if args.action == "plan" else api.run_return(args.plan))
    if args.group == "cube":
        return (api.cube_new() if args.action == "new" else
                api.cube_capture(args.session, args.face) if args.action == "capture" else
                cube_guide(args,api) if args.action == "guide" else
                cube_extrema_dispatch(args, api)
                if args.action == "extrema" else
                api.cube_status(args.session) if args.action == "status" else
                api.cube_fit(args.session))
    if args.group == "probe":
        return (api.plan_probe(axis=args.axis, delta_m=args.delta_m,
                               anchor_recording=args.anchor_recording)
                if args.action == "plan" else api.run_probe(args.plan))
    if args.group == "joint-probe":
        return (api.plan_joint_probe(speed_percent=args.speed_percent)
                if args.action == "plan" else api.run_joint_probe(args.plan))
    if args.group == "map":
        return api.reachability_summary()
    if args.group == "continuous-archive":
        return (api.plan_continuous_archive(frequency_hz=args.frequency_hz)
                if args.action == "plan" else api.run_continuous_archive(args.plan))
    if args.group == "quick":
        return quick_dispatch(args,api)
    if args.group == "pose-candidate":
        return (api.plan_pose_candidate() if args.action == "plan" else
                api.run_pose_candidate(args.plan,args.stage))
    raise ValueError("未知命令")


def display_result(result):
    """终端只显示规划摘要；完整目标序列始终保存在 plan_path。"""
    if not isinstance(result, dict) or not isinstance(result.get("plan"), dict):
        return result
    plan = dict(result["plan"])
    for key in ("route_joint_rad", "waypoints"):
        if isinstance(plan.get(key), list):
            plan[key + "_count"] = len(plan.pop(key))
    return {**result, "plan": plan}


def main(argv=None):
    parser = parser_create()
    args = parser.parse_args(argv)
    try:
        if args.group == "quick" and args.action in ("home-demo", "cube-sweep", "cube-faces"):
            require_drag_start_safe()
        result = dispatch(args, LabAPI(args.config))
        print(json.dumps(display_result(result), ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError,
            KeyError, TypeError) as exc:
        print(f"实验未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
