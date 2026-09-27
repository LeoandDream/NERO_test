"""限时示教记录的发现、封存校验和设备配置匹配。纯离线。"""

from dataclasses import dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
from pathlib import Path
import re

from experiments.single_arm.teaching.return_session import load_recording
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


RECORDING_DIR = Path("experiments/single_arm/teaching/data/recordings")
SESSION_NAME = re.compile(r"nero_session_(\d{8}T\d{6}(?:\d{6})?Z)")


@dataclass(frozen=True)
class RecordingInfo:
    recording_id: str
    path: Path
    metadata_path: Path
    sha256: str
    config_sha256: str
    finished_at_utc: str
    sample_count: int
    duration_s: float
    original_joint_rad: tuple[float, ...]
    replay_mode: str = "point_move_j"
    completion_path: Path | None = None

    def as_dict(self):
        return {**self.__dict__, "path": str(self.path),
                "metadata_path": str(self.metadata_path),
                "completion_path": (str(self.completion_path)
                                    if self.completion_path else None),
                "original_joint_rad": list(self.original_joint_rad)}


def _inside(path, root):
    path, root = Path(path).resolve(), Path(root).resolve()
    if not path.is_relative_to(root) or path.suffix != ".csv":
        raise ValueError("示教记录必须是指定记录目录内的 CSV")
    return path


def validate_recording(path, config_path=DEFAULT_CONFIG, directory=RECORDING_DIR):
    """校验原始哈希、S1 回位证据、轨迹连续性及当前配置。"""
    path = _inside(path, directory)
    config_path = Path(config_path)
    config = load_config(config_path)
    meta_path = path.with_suffix(".json")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    if meta.get("schema_version") != 2 or meta.get("robot_model") != "NERO" or meta.get("firmware_driver") != "V121":
        raise ValueError("记录缺少设备与哈希封存元数据；旧记录需另走已验证入口")
    if meta.get("recording_id") != path.stem or Path(meta.get("recording", "")).resolve() != path:
        raise ValueError("记录 ID、CSV 路径与元数据不一致")
    config_sha = hashlib.sha256(config_path.read_bytes()).hexdigest()
    if meta.get("config_sha256") != config_sha:
        raise ValueError("记录创建时配置与当前配置不一致")
    source_sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if meta.get("recording_sha256") != source_sha:
        raise ValueError("示教 CSV 已变化或缺少原始文件校验和")
    if meta.get("stop_reason") != "time_limit" or meta.get("emergency_stop_requested"):
        raise ValueError("示教不是完整限时录制，或曾请求急停")
    completion_path = None
    if meta.get("return_completed") is True:
        if meta.get("error"):
            raise ValueError("已完成示教的元数据包含错误")
    elif path.with_suffix(".completion.json").exists():
        # 单独回 S1 的成功凭证须绑定原始 CSV/JSON、配置和执行报告。
        # 原始失败元数据保持不变，不能仅靠一个完成布尔值放行回放。
        from experiments.single_arm.lab.complete_recording import validate_completion
        completion_path = validate_completion(
            path, meta, config, config_path=config_path,
            recording_sha256=source_sha,
            metadata_sha256=hashlib.sha256(meta_path.read_bytes()).hexdigest())
    else:
        raise ValueError("示教不是完整限时录制并完成回位；缺少独立回位凭证")
    _, _, original, points = load_recording(path, config, allow_completed=True)
    post_count = meta.get("post_stop_samples")
    if (not isinstance(post_count, int) or post_count < 2
            or post_count >= len(points)):
        raise ValueError("退出示教后的停稳样本数量无效")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    split = len(rows)-post_count
    post_statuses = [row["teach_status"] for row in rows[split:]]
    stop_count = 0
    while (stop_count < len(post_statuses)
           and post_statuses[stop_count] == "STOP_RECORDING"):
        stop_count += 1
    # 停止帧确认与 CSV 采样不同步：刚退出拖动后可能采到短暂的
    # STOP_RECORDING。它只能出现在停稳段最前面，后面必须是 DISABLED。
    if (not {"teach_status", "pose_feedback_unix_s", "x_m", "y_m", "z_m"}.issubset(rows[0])
            or any(row["teach_status"] != "START_RECORDING" for row in rows[:split])
            or stop_count > 5 or len(post_statuses)-stop_count < 2
            or any(status != "DISABLED" for status in post_statuses[stop_count:])):
        raise ValueError("示教段与退出后的停稳段缺失或顺序错误")
    pose_stamps = [float(row["pose_feedback_unix_s"]) for row in rows]
    if (any(not math.isfinite(value) for value in pose_stamps)
            or any(b < a for a,b in zip(pose_stamps, pose_stamps[1:]))):
        raise ValueError("法兰反馈时间无效或倒退")
    for index,(first,second) in enumerate(zip(points,points[1:]),1):
        if math.dist(first,second) > config["max_recorded_step_rad"]:
            raise ValueError(f"示教记录第 {index} 行关节反馈不连续")
    # 进入拖动和写下首行之间会有有限延迟。逐轴 0.02 rad 允许这段
    # 小幅起步，同时与 build_cycle 的起点连续性检查共同约束首段。
    if (max(abs(a-b) for a,b in zip(points[0],original)) > 0.02
            or max(math.dist(point,points[-1]) for point in points[-min(5,post_count):]) > 0.002):
        raise ValueError("示教起点或停稳段与记录不符")
    finished = datetime.fromisoformat(meta["finished_at_utc"])
    if finished.tzinfo is None or finished > datetime.now(timezone.utc):
        raise ValueError("示教结束时间无效")
    return RecordingInfo(
        recording_id=path.stem, path=path, metadata_path=meta_path,
        sha256=source_sha, config_sha256=config_sha,
        finished_at_utc=finished.isoformat(), sample_count=len(points),
        duration_s=float(meta["max_teach_seconds"]),
        original_joint_rad=tuple(original),
        completion_path=completion_path,
    )


def list_recordings(config_path=DEFAULT_CONFIG, directory=RECORDING_DIR):
    """返回已通过严格校验的记录（最近在前）；不连接 CAN。"""
    valid = []
    for path in Path(directory).glob("nero_session_*.csv"):
        try:
            valid.append(validate_recording(path, config_path, directory))
        except (OSError, ValueError, KeyError, TypeError, IndexError):
            continue
    return sorted(valid, key=lambda item: item.finished_at_utc, reverse=True)


def resolve_recording(selector="latest", config_path=DEFAULT_CONFIG,
                      directory=RECORDING_DIR):
    """ID、目录内 CSV 路径或 latest；最近一次失败不能被旧记录掩盖。"""
    if selector == "latest":
        attempts = [path for path in Path(directory).glob("nero_session_*.csv")
                    if SESSION_NAME.fullmatch(path.stem)]
        if not attempts:
            raise ValueError("没有新格式示教记录；请先完成示教")
        newest = max(attempts, key=lambda path: SESSION_NAME.fullmatch(path.stem).group(1))
        try:
            return validate_recording(newest, config_path, directory)
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            detail = ""
            try:
                meta = json.loads(newest.with_suffix(".json").read_text(encoding="utf-8"))
                if meta.get("error"):
                    detail = f"；原始失败原因：{meta['error']}"
            except (OSError, ValueError):
                pass
            raise ValueError(
                f"最近一次示教 {newest.stem} 未形成可回放记录：{exc}{detail}。"
                "先按实时状态恢复到 S1；如要回放更早的完整记录，请显式指定记录 ID"
            ) from exc
    candidate = Path(selector)
    if candidate.suffix != ".csv":
        candidate = Path(directory) / (str(selector) + ".csv")
    return validate_recording(candidate, config_path, directory)
