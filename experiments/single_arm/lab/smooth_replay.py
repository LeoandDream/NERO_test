"""新封存示教的定时 move_j 回放；复用已验证的关节流执行器。

这里仅适配源文件格式并构造计划。实时发布、反馈监测和异常停止由
continuous_replay.run_stream 保持统一实现，避免两份运动循环出现分歧。
"""

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from experiments.single_arm.continuous_replay.continuous_session import (
    geometry_audit, robot_instance,
)
from experiments.single_arm.continuous_replay.trajectory import (
    MAX_SOURCE_GAP_S, make_plan,
)
from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.recordings import RECORDING_DIR, resolve_recording
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


RUN_DIR = Path("experiments/single_arm/lab/data/replays")
START_TOLERANCE_RAD = 0.005
INTRO_DURATION_S = 0.1


def read_sealed_source(path, config, *, config_path=DEFAULT_CONFIG,
                       recording_dir=RECORDING_DIR):
    """从新格式记录读取实测时序，在最前面加入示教前实际起点。

    拖动启动至首个 CSV 样本有短暂延迟，首行最多可偏离起点 0.02 rad。
    插入 0.1 秒的零速起点，让控制器从真实 S1 平滑连接到首个样本。
    停止拖动后的停稳样本保留在末尾，使反向前目标速度降为零。
    """
    info = resolve_recording(str(path), config_path, recording_dir)
    meta = json.loads(info.metadata_path.read_text(encoding="utf-8"))
    with info.path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 10 or meta["post_stop_samples"] < 5:
        raise ValueError("连续回放需要至少 10 条记录及 5 条停稳样本")
    raw_times = [float(row["elapsed_s"]) for row in rows]
    if any(not math.isfinite(value) for value in raw_times):
        raise ValueError("示教源时间包含非有限值")
    recorded_times = [value - raw_times[0] + INTRO_DURATION_S for value in raw_times]
    times = [0.0] + recorded_times
    if not all(0 < b-a <= MAX_SOURCE_GAP_S for a,b in zip(times,times[1:])):
        raise ValueError("连续示教源的样本时间间隔超出 0.15 秒")
    original = [float(value) for value in info.original_joint_rad]
    joints = [original] + [[float(row[f"joint_{i}_rad"]) for i in range(1,8)]
                           for row in rows]
    xyz = [[float(row[key]) for key in ("x_m","y_m","z_m")] for row in rows]
    # 原位与首个样本的关节差已由封存校验限制，首点法兰反馈用于源包络审计。
    return {"path":str(info.path), "sha256":info.sha256,
            "metadata":meta, "original":original,
            "times":times, "joints":joints,
            "teaching_rows":len(rows)-meta["post_stop_samples"]+1,
            "flange_xyz":[list(xyz[0])]+xyz}


def build_continuous_plan(selector="latest", *, config_path=DEFAULT_CONFIG,
                          recording_dir=RECORDING_DIR, frequency_hz=20,
                          min_time_scale=6.0, robot_factory=None):
    """只读生成完整正向及反向定时关节流，并复核源反馈与 FK。"""
    config_path = Path(config_path)
    config = load_config(config_path)
    info = resolve_recording(selector, config_path, recording_dir)
    source = read_sealed_source(info.path, config, config_path=config_path,
                                recording_dir=recording_dir)
    plan = make_plan(source, config, frequency_hz=frequency_hz,
                     min_time_scale=min_time_scale)
    plan.update({"kind":"quick_teach_replay", "replay_mode":"timed_move_j",
                 "recording_id":info.recording_id,
                 "recording_sha256":info.sha256,
                 "metadata":str(info.metadata_path),
                 "config_sha256":hashlib.sha256(config_path.read_bytes()).hexdigest(),
                 "mount":config["mount"],
                 "safe_start_joint_rad":config["safe_start_joint_rad"],
                 "source_sample_count":info.sample_count,
                 "created_at_utc":datetime.now(timezone.utc).isoformat()})
    # SDK FK 不发送控制命令。和固定源连续实验共用包络及反馈一致性审计。
    fk_robot = (robot_factory or robot_instance)(config["channel"])
    plan["geometry_audit"] = geometry_audit(fk_robot, plan, source, config)
    with connected(config, robot_factory) as robot:
        snapshot = require_ready(read_state(robot))
    if max(abs(a-b) for a,b in zip(snapshot["joint_rad"],source["original"])) > START_TOLERANCE_RAD:
        raise ValueError("当前关节角偏离新示教实际起点超过 0.005 rad")
    plan["observed_start_joint_rad"] = snapshot["joint_rad"]
    plan["observed_start_flange_pose_m_rad"] = snapshot["flange_pose_m_rad"]
    return plan


def verify_continuous_plan(plan, *, config_path=DEFAULT_CONFIG,
                           recording_dir=RECORDING_DIR, robot_factory=None):
    """执行前重算源和所有目标，拒绝修改后的计划或配置。"""
    config_path = Path(config_path)
    if (plan.get("schema_version") != 1 or plan.get("kind") != "quick_teach_replay"
            or plan.get("replay_mode") != "timed_move_j"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("recovery_mode") or plan.get("one_way_mode")
            or plan.get("recovery_parent_report") is not None):
        raise ValueError("连续回放计划版本、模式或设备配置不匹配")
    config = load_config(config_path)
    info = resolve_recording(plan["recording"], config_path, recording_dir)
    source = read_sealed_source(info.path, config, config_path=config_path,
                                recording_dir=recording_dir)
    expected = make_plan(source, config, frequency_hz=plan["frequency_hz"],
                         min_time_scale=plan["min_time_scale"])
    expected["geometry_audit"] = geometry_audit(
        (robot_factory or robot_instance)(config["channel"]),expected,source,config)
    if (plan.get("recording_id") != info.recording_id
            or plan.get("recording_sha256") != info.sha256
            or plan.get("metadata") != str(info.metadata_path)
            or plan.get("source_sample_count") != info.sample_count
            or plan.get("mount") != config["mount"]
            or plan.get("safe_start_joint_rad") != config["safe_start_joint_rad"]):
        raise ValueError("连续回放源或设备条件已变化")
    for key, value in expected.items():
        if plan.get(key) != value:
            raise ValueError(f"连续回放计划字段 {key} 与重新计算结果不符")
    if (not isinstance(plan.get("observed_start_joint_rad"),list)
            or len(plan["observed_start_joint_rad"]) != 7
            or max(abs(a-b) for a,b in zip(plan["observed_start_joint_rad"],
                                             source["original"])) > START_TOLERANCE_RAD):
        raise ValueError("连续回放计划起点无效")
    return plan


def execute_continuous_plan(plan_path, *, config_path=DEFAULT_CONFIG,
                            recording_dir=RECORDING_DIR, robot_factory=None,
                            run_dir=RUN_DIR):
    """重新验算后使用原有定时发布器；失败时由原发布器记录并停止。"""
    from experiments.single_arm.continuous_replay.run_stream import run
    path = Path(plan_path)
    plan = verify_continuous_plan(json.loads(path.read_text(encoding="utf-8")),
                                  config_path=config_path,
                                  recording_dir=recording_dir,
                                  robot_factory=robot_factory)
    config = load_config(Path(config_path))

    def source_reader(source_path, current_config):
        return read_sealed_source(source_path,current_config,config_path=config_path,
                                  recording_dir=recording_dir)

    return run(plan,path,config,robot_factory=robot_factory,
               source_reader=source_reader,run_dir=run_dir)
