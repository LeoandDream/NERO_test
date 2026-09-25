#!/usr/bin/env python3
"""离线比较临时锚点至 S1 的两目标关节路线；只读，不连接机械臂。"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.start_transfer.geometry.urdf_origins import (
    DEFAULT_URDF, link_origin_envelopes, link_origins, parse_joint_origins,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config, validate_joints


DEFAULT_REPORT = Path(
    "experiments/single_arm/control_interfaces/data/runs/nero_move_l_20260924T151836Z.json"
)
OUTPUT_DIR = Path("experiments/single_arm/start_transfer/data/plans")
MODEL_SOURCE = (
    "https://github.com/agilexrobotics/agx_arm_urdf/blob/"
    "f6642ce0d7872c686f29c99e9e10cd23d1d49313/nero/urdf/nero_description.urdf"
)


def sampled_joints(stages, samples_per_stage=100):
    for start, target in zip(stages, stages[1:]):
        for step in range(samples_per_stage + 1):
            fraction = step / samples_per_stage
            yield [a + fraction * (b - a) for a, b in zip(start, target)]


def evaluate_route(robot, stages, model, config):
    """比较关节/连杆原点预测，不推断控制器或现场真实轨迹。"""
    min_bend_pair = math.inf
    for joints in sampled_joints(stages):
        validate_joints(joints, config["zero_exclusion_radius_rad"])
        min_bend_pair = min(min_bend_pair, max(abs(joints[1]), abs(joints[3])))
        flange_urdf = link_origins(joints, model)["link7"]
        flange_sdk = robot.fk(joints)[:3]
        if math.dist(flange_urdf, flange_sdk) > 0.001:
            raise ValueError("URDF 与本机 SDK FK 法兰位置不一致；拒绝引用连杆范围")
    return {
        "number_of_controller_targets": len(stages) - 1,
        "joint_distance_per_target_rad": [
            math.dist(start, target) for start, target in zip(stages, stages[1:])
        ],
        "minimum_max_abs_j2_j4_rad": min_bend_pair,
        "link_origin_ranges_m": link_origin_envelopes(stages, model),
    }


def start_from_recording(path, config):
    """使用静止、正常使能后的新记录；历史记录不代表当前实时位置。"""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 5:
        raise ValueError("需要至少 5 条新姿态反馈")
    if any(row.get("control_mode") != "CAN_CTRL"
           or row.get("arm_status") != "NORMAL"
           or row.get("teach_status") != "DISABLED" for row in rows):
        raise ValueError("记录中有非正常或示教状态，不能作为起点")
    joints = [
        validate_joints(
            [float(row[f"joint_{index}_rad"]) for index in range(1, 8)],
            config["zero_exclusion_radius_rad"],
        )
        for row in rows
    ]
    start = joints[-1]
    if max(abs(value - previous[axis])
           for previous in joints for axis, value in enumerate(start)) > 0.002:
        raise ValueError("采样期间关节位置变化超过 0.002 rad")
    return start


def build_comparison(robot, source_report, config, model, source_recording=None):
    if source_recording is None:
        run = json.loads(source_report.read_text(encoding="utf-8"))
        if run.get("control_command") != "move_l" or run.get("completed") is not True:
            raise ValueError("需要已完成的法兰回程运行报告")
        samples = run.get("feedback_samples") or []
        if not samples:
            raise ValueError("运行报告缺少实际关节反馈")
        start = validate_joints(samples[-1]["joint_rad"], config["zero_exclusion_radius_rad"])
    else:
        start = start_from_recording(source_recording, config)
    target = config["safe_start_joint_rad"]
    # 本候选仅分析前次实测的临时锚点。其他姿态必须重新设计，不能套用中间点。
    if not (0.05 <= start[1] <= 0.25 and -0.05 <= start[3] <= 0.12
            and -1.1 <= start[6] <= -0.8):
        raise ValueError("来源不在已分析的临时起点附近姿态区间")
    middle = list(start)
    middle[1] = 0.2     # J4 过零时，先让 J2 保持正向弯曲。
    middle[3] = target[3]
    middle[6] = target[6]
    validate_joints(middle, config["zero_exclusion_radius_rad"])
    return {
        "source_run": str(source_report) if source_recording is None else None,
        "source_recording": str(source_recording) if source_recording is not None else None,
        "model_source": MODEL_SOURCE,
        "historical_s1_name": config["safe_start_name"],
        "historical_s1_joint_rad": target,
        "start_joint_rad_from_recording": start,
        "candidate_intermediate_joint_rad": middle,
        "direct_joint_target_comparison": evaluate_route(robot, [start, target], model, config),
        "two_joint_target_comparison": evaluate_route(robot, [start, middle, target], model, config),
        "live_start_verified": False,
        "collision_meshes_checked": False,
        "environment_and_cables_checked": False,
        "controller_actual_path_verified": False,
        "executable": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--source-report", type=Path, default=None)
    source.add_argument("--source-recording", type=Path, default=None,
                        help="新采集的静止 NORMAL 位姿 CSV；仅做离线比较")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = load_config(args.config)
    model = parse_joint_origins(DEFAULT_URDF)
    # 工厂仅构造本机 SDK 的 FK 计算器；不调用 connect() 或 CAN 写入。
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
        channel=config["channel"],
    ))
    result = build_comparison(robot, args.source_report or DEFAULT_REPORT, config, model,
                              source_recording=args.source_recording)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output = OUTPUT_DIR / (
        "nero_two_target_offline_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("只读几何比较：", output)
    print("记录起点 (rad)：", [round(value, 6) for value in result["start_joint_rad_from_recording"]])
    print("候选中间目标 (rad)：", [round(value, 6) for value in result["candidate_intermediate_joint_rad"]])
    print("S1 目标 (rad)：", [round(value, 6) for value in result["historical_s1_joint_rad"]])
    print("直接/两目标 J2、J4 角度指标 (rad)：",
          round(result["direct_joint_target_comparison"]["minimum_max_abs_j2_j4_rad"], 4),
          round(result["two_joint_target_comparison"]["minimum_max_abs_j2_j4_rad"], 4))
    print("两目标各段关节空间距离 (rad)：", [
        round(value, 4) for value in result["two_joint_target_comparison"]["joint_distance_per_target_rad"]
    ])
    print("只分析记录起点和假定的关节线性插值；未连接机械臂，不能直接执行运动。")


if __name__ == "__main__":
    main()
