#!/usr/bin/env python3
"""用固定版本 Nero 碰撞网格复核离线关节路线；永不连接或控制机械臂。

运行前另装 numpy、trimesh、python-fcl，并下载厂商 STL 到 --mesh-dir。
只检查离散的裸臂自碰撞，不检查台面、支撑、线缆及实际控制器路径。
"""

import argparse
from datetime import datetime, timezone
import hashlib
from itertools import product
import json
from pathlib import Path

from experiments.single_arm.start_transfer.geometry.urdf_origins import (
    identity, link_transforms, parse_joint_origins,
)


OUTPUT_DIR = Path("experiments/single_arm/start_transfer/data/diagnostics")
MESH_SHA256 = {
    "base_link": "540462fb56e52091daa3cd1ad3acc3c19a43f120b2f11bae5a69b32b08c738b6",
    "link1": "0e38bf33ce41f04d7e70e471768c00fa144ba4016a878b9a6d8303c319736e09",
    "link2": "fc9eb85db9fbd351c9f6e119f5fed33c6f0dd3bc3b11698a0f96aaeb2d36c721",
    "link3": "44c70f731ad5067d608ebebc9e59f3c231988f992dfac6cb01ee7a6bb4b053f1",
    "link4": "47242e9b5d8b06c912261756f24dddb1e7f54ac6e718232a25889115298b966d",
    "link5": "dc497f5a9bab2d0d314250134d4ec86617abea7205d122db41b765acdb86ae7e",
    "link6": "66417d30596e33c890f4f141781023bd3ea15709a502e7f59b30f6676f556022",
    "link7": "70a2f415358f9e46088673c92bfe78ca5b1072dd73b7ea24464ba3ebb4c3e647",
}
NAMES = list(MESH_SHA256)


def check_meshes(mesh_dir):
    """防止把其他版网格和当前固定版 URDF 混用。"""
    for name, expected in MESH_SHA256.items():
        path = mesh_dir / f"{name}.stl"
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f"{path} 与固定版厂商网格的 SHA-256 不一致")


def audit_configurations(configurations, model, meshes, np, trimesh):
    """对给定关节姿态逐个做非相邻裸臂网格距离和相交检查。"""
    managers = {}
    for first in range(len(NAMES)):
        for second in range(first + 2, len(NAMES)):
            manager = trimesh.collision.CollisionManager()
            manager.add_object(NAMES[first], meshes[first])
            manager.add_object(NAMES[second], meshes[second])
            managers[(first, second)] = manager
    minimum = {pair: float("inf") for pair in managers}
    intersections = {pair: 0 for pair in managers}
    count = 0
    for joints in configurations:
        transforms = {"base_link": identity(), **link_transforms(joints, model)}
        for (first, second), manager in managers.items():
            manager.set_transform(NAMES[first], np.array(transforms[NAMES[first]]))
            manager.set_transform(NAMES[second], np.array(transforms[NAMES[second]]))
            pair = (first, second)
            minimum[pair] = min(minimum[pair], float(manager.min_distance_internal()))
            if manager.in_collision_internal():
                intersections[pair] += 1
        count += 1
    return {
        "sample_count": count,
        "nonadjacent_pair_count": len(managers),
        "intersecting_pairs": {
            f"{NAMES[first]}:{NAMES[second]}": hits
            for (first, second), hits in intersections.items() if hits
        },
        "minimum_pair_distance_m": {
            f"{NAMES[first]}:{NAMES[second]}": value
            for (first, second), value in minimum.items()
        },
    }


def audit_route(stages, model, meshes, samples_per_stage, np, trimesh):
    """检查假定的关节线性插值路线。"""
    samples = (
        [a + (sample / samples_per_stage) * (b - a) for a, b in zip(start, target)]
        for start, target in zip(stages, stages[1:])
        for sample in range(samples_per_stage + 1)
    )
    result = audit_configurations(samples, model, meshes, np, trimesh)
    result["samples_per_stage_including_endpoints"] = samples_per_stage + 1
    return result


def audit_joint_box(start, target, grid_intervals, model, meshes, np, trimesh):
    """抽查各轴不同步时的关节范围，离散网格仍不是连续碰撞证明。"""
    varied = [axis for axis in range(7) if abs(target[axis] - start[axis]) > 1e-6]
    count = (grid_intervals + 1) ** len(varied)
    if count > 5000:
        raise ValueError(f"关节盒需要 {count} 个姿态，超过 5000 上限；请调小网格数")

    def configurations():
        for fractions in product(range(grid_intervals + 1), repeat=len(varied)):
            joints = list(start)
            for axis, step in zip(varied, fractions):
                joints[axis] += (target[axis] - start[axis]) * step / grid_intervals
            yield joints

    result = audit_configurations(configurations(), model, meshes, np, trimesh)
    result["varied_joint_numbers"] = [axis + 1 for axis in varied]
    result["grid_intervals_per_joint"] = grid_intervals
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True, help="只读两目标几何分析输出的 JSON")
    parser.add_argument("--mesh-dir", type=Path, required=True, help="固定版厂商 STL 所在目录")
    parser.add_argument("--samples-per-stage", type=int, default=100)
    parser.add_argument("--box-stage", type=int, choices=(1, 2),
                        help="另抽查本段所有变化关节构成的离散关节盒")
    parser.add_argument("--box-grid-intervals", type=int, default=10)
    args = parser.parse_args()
    if not 10 <= args.samples_per_stage <= 1000:
        parser.error("每段采样间隔数须为 10～1000")
    if not 2 <= args.box_grid_intervals <= 20:
        parser.error("关节盒每轴采样间隔数须为 2～20")
    try:
        import numpy as np
        import trimesh
    except ImportError as exc:
        parser.error(f"离线网格分析需要 numpy、trimesh、python-fcl：{exc}")
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    if plan.get("executable") is not False:
        raise ValueError("此工具只接受明确标记为不可执行的离线比较计划")
    check_meshes(args.mesh_dir)
    model = parse_joint_origins()
    meshes = [trimesh.load_mesh(args.mesh_dir / f"{name}.stl", process=False) for name in NAMES]
    start = plan["start_joint_rad_from_recording"]
    middle = plan["candidate_intermediate_joint_rad"]
    target = plan["historical_s1_joint_rad"]
    result = {
        "source_plan": str(args.plan),
        "mesh_revision": "agx_arm_urdf f6642ce0d7872c686f29c99e9e10cd23d1d49313",
        "mesh_sha256": MESH_SHA256,
        "direct": audit_route([start, target], model, meshes, args.samples_per_stage, np, trimesh),
        "two_target": audit_route([start, middle, target], model, meshes,
                                  args.samples_per_stage, np, trimesh),
        "environment_checked": False,
        "controller_actual_path_checked": False,
        "continuous_path_proven_clear": False,
        "executable": False,
    }
    if args.box_stage is not None:
        previous, destination = ((start, middle) if args.box_stage == 1
                                 else (middle, target))
        result["joint_box_stage"] = args.box_stage
        result["joint_box"] = audit_joint_box(
            previous, destination, args.box_grid_intervals, model, meshes, np, trimesh
        )
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output = OUTPUT_DIR / (
        "nero_mesh_audit_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("只读裸臂网格检查：", output)
    for name in ("direct", "two_target"):
        outcome = result[name]
        closest_pair, closest_distance = min(
            outcome["minimum_pair_distance_m"].items(), key=lambda item: item[1]
        )
        print(f"{name}: {outcome['sample_count']} 个姿态，"
              f"相交连杆对 {outcome['intersecting_pairs']}；"
              f"最近 {closest_pair} = {closest_distance:.5f} m")
    if args.box_stage is not None:
        outcome = result["joint_box"]
        pair, distance = min(outcome["minimum_pair_distance_m"].items(),
                             key=lambda item: item[1])
        print(f"joint_box stage {args.box_stage}: {outcome['sample_count']} 个姿态，"
              f"相交连杆对 {outcome['intersecting_pairs']}；"
              f"最近 {pair} = {distance:.5f} m")
    print("离散裸臂模型结果；不能据此发送实机运动。")


if __name__ == "__main__":
    main()
