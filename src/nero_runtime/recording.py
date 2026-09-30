"""独立只读轨迹记录：保存每次 F1 单轮采集，不筛掉坏质量或故障事实。"""

from dataclasses import asdict, dataclass, fields
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time

from .acquisition import AcquisitionResult
from .snapshot import RobotSnapshot


SCHEMA_VERSION = 1
QUALITY_STATES = ("valid", "stale", "missing", "invalid")
_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}\Z")


@dataclass(frozen=True)
class RecordingParameters:
    duration_s: float
    rate_hz: float

    def __post_init__(self):
        for name, upper in (("duration_s", 3600.0), ("rate_hz", 100.0)):
            value = getattr(self, name)
            try:
                valid = (type(value) in (int, float) and math.isfinite(float(value))
                         and 0 < value <= upper)
            except OverflowError:
                valid = False
            if not valid:
                raise ValueError(f"{name} must be finite and within (0, {upper}]")


def recording_paths(output_directory: str | Path, recording_id: str) -> tuple[Path, Path]:
    """校验本次新文件目标；CLI 在连接设备之前调用。"""
    if type(recording_id) is not str or _ID_PATTERN.fullmatch(recording_id) is None:
        raise ValueError("recording_id must be 1..128 ASCII letters/digits/_/-")
    directory = Path(output_directory)
    if directory.exists() and not directory.is_dir():
        raise ValueError(f"recording output is not a directory: {directory}")
    raw = directory / f"{recording_id}.jsonl"
    summary = directory / f"{recording_id}.summary.json"
    for path in (raw, summary):
        if path.exists() or path.is_symlink():
            raise FileExistsError(f"recording file already exists: {path}")
    return raw, summary


def _number(value, label: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{label} must be finite non-negative seconds")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{label} must be finite non-negative seconds") from exc
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"{label} must be finite non-negative seconds")
    return number


def _utc(value) -> str:
    return datetime.fromtimestamp(_number(value, "wall clock"), timezone.utc).isoformat()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _counts() -> dict[str, int]:
    return {state: 0 for state in QUALITY_STATES}


def _quality(snapshot: RobotSnapshot, key: str) -> str:
    state = getattr(snapshot, key).state
    if state not in QUALITY_STATES:
        raise ValueError(f"unknown {key} state")
    return state


def record_trajectory(
    *, recording_id: str, output_directory: str | Path,
    parameters: RecordingParameters, read_snapshot, monotonic_s, sleep_s,
    cancel_requested, finish_requested, wall_clock_s=time.time,
) -> dict:
    """按单调节拍流式保存每轮 AcquisitionResult，异常后保留已写原件。

    调用方持有反馈源；本函数不创建 SDK、不连接设备、不执行停止控制。
    `complete` 只表示录制正常结束，不表示 Replay 资格。
    """
    if type(parameters) is not RecordingParameters:
        raise TypeError("RecordingParameters are required")
    for name, value in (("read_snapshot", read_snapshot),
                        ("monotonic_s", monotonic_s), ("sleep_s", sleep_s),
                        ("cancel_requested", cancel_requested),
                        ("finish_requested", finish_requested),
                        ("wall_clock_s", wall_clock_s)):
        if not callable(value):
            raise TypeError(f"{name} callable is required")
    raw_path, summary_path = recording_paths(output_directory, recording_id)
    start = _number(monotonic_s(), "monotonic clock")
    last_mono = start
    started_at = _utc(wall_clock_s())
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    stream = raw_path.open("xb")
    sample_count = fault_count = 0
    committed_bytes = 0
    quality_counts = {name: _counts() for name in
                      ("joint", "flange", "status")}
    stop_reason = None
    error = None
    next_tick = start
    deadline = start + parameters.duration_s
    interval = 1.0 / parameters.rate_hz

    def now() -> float:
        nonlocal last_mono
        current = _number(monotonic_s(), "monotonic clock")
        if current < last_mono:
            raise ValueError("monotonic clock moved backwards")
        last_mono = current
        return current

    try:
        with stream:
            while True:
                current = now()
                if cancel_requested():
                    stop_reason = "cancelled"
                    break
                if finish_requested():
                    stop_reason = "user_finish"
                    break
                if current >= deadline:
                    stop_reason = "duration_complete"
                    break
                if current < next_tick:
                    sleep_s(min(next_tick - current, deadline - current))
                    continue
                result = read_snapshot()
                if type(result) is not AcquisitionResult or type(result.snapshot) is not RobotSnapshot:
                    raise TypeError("read_snapshot must return AcquisitionResult with RobotSnapshot")
                observed = now()
                states = {
                    "joint": _quality(result.snapshot, "joints_quality"),
                    "flange": _quality(result.snapshot, "flange_quality"),
                    "status": _quality(result.snapshot, "status_quality"),
                }
                row = {
                    "schema_version": SCHEMA_VERSION,
                    "sample_index": sample_count,
                    "elapsed_s": observed - start,
                    "snapshot": asdict(result.snapshot),
                    "read_errors": list(result.read_errors),
                }
                payload = json.dumps(row, ensure_ascii=False, allow_nan=False,
                                     separators=(",", ":")) + "\n"
                stream.write(payload.encode("utf-8"))
                stream.flush()
                os.fsync(stream.fileno())
                committed_bytes = stream.tell()
                sample_count += 1
                for key, state in states.items():
                    quality_counts[key][state] += 1
                if result.snapshot.error_code is not None and result.snapshot.error_code != 0:
                    fault_count += 1
                # 丢弃错过的节拍，不为了追赶而连续突发采样。
                next_tick = start + max(sample_count,
                                        math.floor((observed - start) * parameters.rate_hz) + 1) * interval
    except KeyboardInterrupt:
        stop_reason = "cancelled"
    except Exception as exc:
        stop_reason = "fault"
        error = f"{type(exc).__name__}: {exc}"

    # 仅移除本轮未确认的半行；此前 fsync 确认的样本保持原字节。
    if raw_path.stat().st_size > committed_bytes:
        try:
            with raw_path.open("r+b") as output:
                output.truncate(committed_bytes)
                output.flush()
                os.fsync(output.fileno())
        except OSError as exc:
            stop_reason = "fault"
            suffix = f"raw tail truncate failed: {type(exc).__name__}: {exc}"
            error = f"{error}; {suffix}" if error else suffix

    actual_duration = max(0.0, last_mono - start)
    summary = {
        "schema_version": SCHEMA_VERSION,
        "recording_id": recording_id,
        "started_at": started_at,
        "finished_at": _utc(wall_clock_s()),
        "requested_duration_s": float(parameters.duration_s),
        "requested_rate_hz": float(parameters.rate_hz),
        "actual_duration_s": actual_duration,
        "sample_count": sample_count,
        "effective_rate_hz": sample_count / actual_duration if actual_duration else 0.0,
        "joint_quality_counts": quality_counts["joint"],
        "flange_quality_counts": quality_counts["flange"],
        "status_quality_counts": quality_counts["status"],
        "controller_fault_sample_count": fault_count,
        "stop_reason": stop_reason,
        "complete": stop_reason in ("duration_complete", "user_finish"),
        "raw_path": raw_path.name,
        "raw_sha256": _sha256(raw_path),
        "error": error,
    }
    with summary_path.open("x", encoding="utf-8") as output:
        output.write(json.dumps(summary, ensure_ascii=False, allow_nan=False,
                                indent=2) + "\n")
        output.flush()
        os.fsync(output.fileno())
    return summary


def _unique_pairs(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate recording JSON key: {key}")
        value[key] = item
    return value


def _reject_constant(value):
    raise ValueError(f"nonstandard recording JSON number: {value}")


def load_recording_summary(path: str | Path) -> dict:
    """读取摘要并核对对应 JSONL 的哈希、行序、数量和质量计数。"""
    path = Path(path)
    summary = json.loads(path.read_text(encoding="utf-8"),
                         object_pairs_hook=_unique_pairs,
                         parse_constant=_reject_constant)
    expected = {
        "schema_version", "recording_id", "started_at", "finished_at",
        "requested_duration_s", "requested_rate_hz", "actual_duration_s",
        "sample_count", "effective_rate_hz", "joint_quality_counts",
        "flange_quality_counts", "status_quality_counts",
        "controller_fault_sample_count", "stop_reason", "complete",
        "raw_path", "raw_sha256", "error",
    }
    if type(summary) is not dict or set(summary) != expected:
        raise ValueError("recording summary fields are incomplete")
    recording_id = summary["recording_id"]
    if (type(recording_id) is not str or _ID_PATTERN.fullmatch(recording_id) is None or
            path.name != f"{recording_id}.summary.json" or
            summary["raw_path"] != f"{recording_id}.jsonl"):
        raise ValueError("recording summary identity or raw path is invalid")
    if type(summary["schema_version"]) is not int or summary["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unknown recording schema version")
    if summary["stop_reason"] not in ("duration_complete", "user_finish", "cancelled", "fault"):
        raise ValueError("unknown recording stop reason")
    if type(summary["complete"]) is not bool or summary["complete"] != (
            summary["stop_reason"] in ("duration_complete", "user_finish")):
        raise ValueError("recording completion disagrees with stop reason")
    for key in ("started_at", "finished_at"):
        if type(summary[key]) is not str or not summary[key]:
            raise ValueError(f"recording {key} is invalid")
    for key in ("requested_duration_s", "requested_rate_hz", "actual_duration_s",
                "effective_rate_hz"):
        _number(summary[key], key)
    if (type(summary["sample_count"]) is not int or summary["sample_count"] < 0 or
            type(summary["controller_fault_sample_count"]) is not int or
            summary["controller_fault_sample_count"] < 0):
        raise ValueError("recording summary counts are invalid")
    if summary["error"] is not None and type(summary["error"]) is not str:
        raise ValueError("recording error is invalid")
    raw_path = path.with_name(summary["raw_path"])
    if _sha256(raw_path) != summary["raw_sha256"]:
        raise ValueError("recording raw SHA256 mismatch")
    counts = {name: _counts() for name in ("joint", "flange", "status")}
    fault_count = 0
    previous_elapsed = -1.0
    with raw_path.open("r", encoding="utf-8") as stream:
        for index, line in enumerate(stream):
            row = json.loads(line, object_pairs_hook=_unique_pairs,
                             parse_constant=_reject_constant)
            if (type(row) is not dict or set(row) != {
                    "schema_version", "sample_index", "elapsed_s", "snapshot", "read_errors"}
                    or type(row["schema_version"]) is not int or
                    row["schema_version"] != SCHEMA_VERSION or
                    type(row["sample_index"]) is not int or row["sample_index"] != index):
                raise ValueError("recording raw sample schema or index is invalid")
            elapsed = _number(row["elapsed_s"], "elapsed_s")
            if elapsed < previous_elapsed:
                raise ValueError("recording raw elapsed time moved backwards")
            previous_elapsed = elapsed
            snapshot = row["snapshot"]
            if (type(snapshot) is not dict or
                    set(snapshot) != {field.name for field in fields(RobotSnapshot)} or
                    type(row["read_errors"]) is not list or
                    any(type(item) is not str for item in row["read_errors"])):
                raise ValueError("recording raw snapshot or read errors are invalid")
            for name, field in (("joint", "joints_quality"),
                                ("flange", "flange_quality"),
                                ("status", "status_quality")):
                quality = snapshot[field]
                if type(quality) is not dict or quality.get("state") not in QUALITY_STATES:
                    raise ValueError("recording raw quality is invalid")
                counts[name][quality["state"]] += 1
            if snapshot["error_code"] is not None and snapshot["error_code"] != 0:
                fault_count += 1
        line_count = index + 1 if previous_elapsed >= 0 else 0
    if line_count != summary["sample_count"]:
        raise ValueError("recording raw sample count mismatch")
    for name in ("joint", "flange", "status"):
        if summary[f"{name}_quality_counts"] != counts[name]:
            raise ValueError(f"recording {name} quality count mismatch")
    if summary["controller_fault_sample_count"] != fault_count:
        raise ValueError("recording controller fault count mismatch")
    return summary


def list_recordings(directory: str | Path) -> list[dict]:
    directory = Path(directory)
    if not directory.is_dir():
        raise ValueError(f"recording directory does not exist: {directory}")
    entries = []
    for path in sorted(directory.glob("*.summary.json")):
        try:
            entries.append({"path": path, "summary": load_recording_summary(path), "error": None})
        except (OSError, ValueError) as exc:
            entries.append({"path": path, "summary": None, "error": str(exc)})
    return entries


def format_recording_summary(summary: dict, path: str | Path) -> str:
    raw = Path(path).with_name(summary["raw_path"])
    lines = [
        f"Recording: {summary['recording_id']}",
        f"Started: {summary['started_at']}",
        f"Finished: {summary['finished_at']}",
        f"Duration: {summary['actual_duration_s']:.3f} s (requested {summary['requested_duration_s']:.3f} s)",
        f"Samples: {summary['sample_count']}",
        f"Requested rate: {summary['requested_rate_hz']:.3f} Hz",
        f"Effective rate: {summary['effective_rate_hz']:.3f} Hz (samples / actual duration)",
    ]
    for title, key in (("Joint feedback", "joint_quality_counts"),
                       ("Flange feedback", "flange_quality_counts"),
                       ("Controller feedback", "status_quality_counts")):
        lines.append(f"{title}:")
        for state in QUALITY_STATES:
            lines.append(f"  {state}: {summary[key][state]}")
    lines.extend([
        f"Controller fault samples: {summary['controller_fault_sample_count']}",
        f"Stop reason: {summary['stop_reason']}",
        f"Complete: {summary['complete']} (recording only; no Replay approval)",
        f"Error: {summary['error'] or 'none'}",
        f"Raw file: {raw.resolve()}",
        f"SHA256: {summary['raw_sha256']}",
    ])
    return "\n".join(lines)
