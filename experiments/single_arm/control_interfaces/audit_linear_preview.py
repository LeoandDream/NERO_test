#!/usr/bin/env python3
"""离线抽查 S1 附近单条 move_l 预检关节路径的裸臂网格间距。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.control_interfaces.cartesian_linear_session import preview_line
from experiments.single_arm.start_transfer.geometry.audit_self_collision import (
    MESH_SHA256, NAMES, audit_configurations, check_meshes,
)
from experiments.single_arm.start_transfer.geometry.urdf_origins import parse_joint_origins
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


OUTPUT_DIR = Path("experiments/single_arm/control_interfaces/data/diagnostics")


def subdivided(route, intervals):
    yield list(route[0])
    for start, target in zip(route, route[1:]):
        for sample in range(1, intervals + 1):
            fraction = sample / intervals
            yield [a + fraction * (b - a) for a, b in zip(start, target)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dx-m", type=float, default=-0.01)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--samples-per-preview-step", type=int, default=10)
    args = parser.parse_args()
    if not 2 <= args.samples_per_preview_step <= 100:
        parser.error("每段离散抽查数量须为 2～100")
    try:
        import numpy as np
        import trimesh
    except ImportError as exc:
        parser.error(f"需离线安装 numpy、trimesh、python-fcl：{exc}")
    check_meshes(args.mesh_dir)
    config = load_config(args.config)
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
        channel=config["channel"],
    ))
    start = config["safe_start_joint_rad"]
    start_pose = list(robot.fk(start))
    target = start_pose[:]
    target[0] += args.dx_m
    route = preview_line(robot, start, start_pose, target, config)
    meshes = [trimesh.load_mesh(args.mesh_dir / f"{name}.stl", process=False)
              for name in NAMES]
    result = audit_configurations(
        subdivided(route, args.samples_per_preview_step),
        parse_joint_origins(), meshes, np, trimesh,
    )
    result.update({
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_config": str(args.config),
        "mesh_sha256": MESH_SHA256,
        "delta_x_m": args.dx_m,
        "start_joint_rad": start,
        "start_flange_pose": start_pose,
        "target_flange_pose": target,
        "preview_joint_waypoints": route,
        "samples_per_preview_step": args.samples_per_preview_step,
        "environment_checked": False,
        "controller_actual_path_checked": False,
        "continuous_path_proven_clear": False,
        "executable": False,
    })
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output = OUTPUT_DIR / ("nero_linear_mesh_" +
                           datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")
    pair, distance = min(result["minimum_pair_distance_m"].items(),
                         key=lambda item: item[1])
    print("离线网格报告：", output)
    print(f"抽查 {result['sample_count']} 个裸臂姿态；相交连杆对："
          f"{result['intersecting_pairs']}；最近 {pair} = {distance:.5f} m")
    print("仅为本机近邻 IK 的离散路径；未检查控制器实际轨迹或现场环境，不能据此执行运动。")


if __name__ == "__main__":
    main()
