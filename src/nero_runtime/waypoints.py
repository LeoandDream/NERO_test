"""单次七轴观测点位：从现有快照捕获，并以一条普通 JSON 保存。

点位只是带来源的反馈记录；名称、状态和文件内容均不授予运动许可。
"""

from dataclasses import asdict, fields
import json
import math
import os
from pathlib import Path
import tempfile

from .snapshot import RobotSnapshot


SCHEMA_VERSION = 1
SOURCE_LIMITATION = (
    "SDK joint feedback combines multiple CAN frame caches; its aggregate "
    "timestamp does not prove every joint was updated together."
)
_RECORD_KEYS = {
    "schema_version", "kind", "name", "source", "robot_id",
    "captured_at_s", "time_basis", "joint_unit", "flange_position_unit",
    "flange_orientation_unit", "flange_frame", "tool_id",
    "source_limitations", "read_errors", "snapshot",
}


def validate_name(name: str) -> str:
    if (type(name) is not str or not name or name != name.strip() or
            len(name) > 128 or any(ord(char) < 32 for char in name)):
        raise ValueError("point name must be 1..128 visible characters without surrounding space")
    return name


def _finite(value, *, nonnegative=False) -> bool:
    if type(value) not in (int, float):
        return False
    try:
        number = float(value)
    except OverflowError:
        return False
    return math.isfinite(number) and (not nonnegative or number >= 0)


def _quality(data: object, observed_at_s: float) -> None:
    if type(data) is not dict or set(data) != {"state", "timestamp_s", "age_s", "issues"}:
        raise ValueError("snapshot source quality is incomplete")
    if data["state"] not in ("valid", "missing", "invalid", "stale"):
        raise ValueError("snapshot source quality state is unknown")
    timestamp, age = data["timestamp_s"], data["age_s"]
    if timestamp is not None and not _finite(timestamp, nonnegative=True):
        raise ValueError("snapshot source timestamp is invalid")
    if age is not None and not _finite(age):
        raise ValueError("snapshot source age is invalid")
    if data["state"] == "valid":
        if timestamp is None or age is None or not _finite(age, nonnegative=True):
            raise ValueError("valid source requires timestamp and non-negative finite age")
        if timestamp > observed_at_s or not math.isclose(
                age, observed_at_s - timestamp, rel_tol=1e-9, abs_tol=1e-6):
            raise ValueError("valid source age disagrees with observation time")
    if type(data["issues"]) is not list or any(type(x) is not str for x in data["issues"]):
        raise ValueError("snapshot source issues are invalid")


def _vector(value: object, length: int) -> bool:
    return type(value) is list and len(value) == length and all(_finite(x) for x in value)


def _validate_record(record: object) -> dict:
    if type(record) is not dict or set(record) != _RECORD_KEYS:
        raise ValueError("waypoint record fields are incomplete or unknown")
    if type(record["schema_version"]) is not int or record["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unknown waypoint schema version")
    if record["kind"] != "observed_joint_waypoint":
        raise ValueError("unknown waypoint record kind")
    validate_name(record["name"])
    if type(record["source"]) is not str or not record["source"].strip():
        raise ValueError("waypoint source is missing")
    if record["robot_id"] is not None and (type(record["robot_id"]) is not str or
                                        not record["robot_id"].strip()):
        raise ValueError("waypoint robot_id is invalid")
    if not _finite(record["captured_at_s"], nonnegative=True):
        raise ValueError("waypoint capture time is invalid")
    if type(record["time_basis"]) is not str or not record["time_basis"].strip():
        raise ValueError("waypoint time basis is missing")
    if (record["joint_unit"] != "rad" or record["flange_position_unit"] != "m" or
            record["flange_orientation_unit"] != "rad"):
        raise ValueError("waypoint units are invalid")
    if record["flange_frame"] is not None or record["tool_id"] is not None:
        raise ValueError("waypoint frame/tool identity was not established by this schema")
    if record["source_limitations"] != [SOURCE_LIMITATION]:
        raise ValueError("waypoint SDK cache limitation is missing")
    if (type(record["read_errors"]) is not list or
            any(type(error) is not str for error in record["read_errors"])):
        raise ValueError("waypoint read errors are invalid")

    snap = record["snapshot"]
    if type(snap) is not dict or set(snap) != {field.name for field in fields(RobotSnapshot)}:
        raise ValueError("waypoint snapshot is incomplete")
    if (snap["robot_id"] != record["robot_id"] or
            snap["observed_at_s"] != record["captured_at_s"] or
            snap["time_basis"] != record["time_basis"]):
        raise ValueError("waypoint source metadata does not match the snapshot")
    for key in ("status_quality", "joints_quality", "flange_quality"):
        _quality(snap[key], snap["observed_at_s"])
    if snap["joints_quality"]["state"] != "valid" or not _vector(snap["joint_rad"], 7):
        raise ValueError("seven finite joint angles with valid aggregate timestamp are required")
    joint_stamp = snap["joints_quality"]["timestamp_s"]
    if joint_stamp > record["captured_at_s"]:
        raise ValueError("joint feedback timestamp is after capture time")
    flange = snap["flange_pose_m_rad"]
    if (snap["flange_quality"]["state"] == "valid") != _vector(flange, 6):
        raise ValueError("flange quality and pose disagree")
    status_fields = ("control_mode", "arm_status", "teach_status", "motion_status", "error_code")
    for key in status_fields:
        if snap[key] is not None and type(snap[key]) is not int:
            raise ValueError(f"snapshot {key} is invalid")
    if ((snap["status_quality"]["state"] == "valid") !=
            all(type(snap[key]) is int for key in status_fields)):
        raise ValueError("controller quality and status fields disagree")
    for key in ("issues", "uninterpreted_status_fields"):
        if type(snap[key]) is not list or any(type(x) is not str for x in snap[key]):
            raise ValueError(f"snapshot {key} is invalid")
    drivers = snap["drivers"]
    if type(drivers) is not list or len(drivers) != 7:
        raise ValueError("snapshot driver list must contain seven entries")
    for index, driver in enumerate(drivers, 1):
        if type(driver) is not dict or set(driver) != {
                "joint", "enabled", "undervoltage", "driver_error", "voltage_v", "quality"}:
            raise ValueError("snapshot driver entry is incomplete")
        if type(driver["joint"]) is not int or driver["joint"] != index:
            raise ValueError("snapshot driver order is invalid")
        _quality(driver["quality"], snap["observed_at_s"])
        for key in ("enabled", "undervoltage", "driver_error"):
            if driver[key] is not None and type(driver[key]) is not bool:
                raise ValueError("snapshot driver flag is invalid")
        if driver["voltage_v"] is not None and not _finite(driver["voltage_v"]):
            raise ValueError("snapshot driver voltage is invalid")
        complete = (all(type(driver[key]) is bool for key in
                        ("enabled", "undervoltage", "driver_error")) and
                    _finite(driver["voltage_v"]))
        if (driver["quality"]["state"] == "valid") != complete:
            raise ValueError("driver quality and fields disagree")
    return record


def capture_waypoint(name: str, snapshot: RobotSnapshot, *, source: str,
                     read_errors=()) -> dict:
    """捕获调用方已经取得的一次快照；不读取时钟或设备。"""
    if not isinstance(snapshot, RobotSnapshot):
        raise TypeError("snapshot must be a RobotSnapshot")
    record = {
        "schema_version": SCHEMA_VERSION,
        "kind": "observed_joint_waypoint",
        "name": validate_name(name), "source": source,
        "robot_id": snapshot.robot_id,
        "captured_at_s": snapshot.observed_at_s,
        "time_basis": snapshot.time_basis,
        "joint_unit": "rad", "flange_position_unit": "m",
        "flange_orientation_unit": "rad",
        "flange_frame": None, "tool_id": None,
        "source_limitations": [SOURCE_LIMITATION],
        "read_errors": list(read_errors),
        "snapshot": asdict(snapshot),
    }
    # 标准 JSON 往返统一 tuple/list，并拒绝任何 NaN/Inf。
    plain = json.loads(json.dumps(record, allow_nan=False))
    return _validate_record(plain)


def save_waypoint(record: dict, output: str | Path) -> Path:
    """在同目录先写完整临时文件，再原子建立不覆盖的目标文件名。"""
    payload = json.dumps(_validate_record(record), ensure_ascii=False,
                         indent=2, allow_nan=False) + "\n"
    path = Path(output)
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"waypoint file already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".waypoint-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)  # EEXIST 保留已有目标，包括并发创建的文件。
    finally:
        Path(temporary).unlink(missing_ok=True)
    return path


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"nonstandard JSON number: {value}")


def load_waypoint(path: str | Path) -> dict:
    path = Path(path)
    try:
        record = json.loads(path.read_text(encoding="utf-8"),
                            object_pairs_hook=_unique_pairs,
                            parse_constant=_reject_constant)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid waypoint JSON: {path}: {exc}") from exc
    return _validate_record(record)


def list_waypoints(directory: str | Path) -> list[dict]:
    """列出 JSON；损坏文件带错误返回，绝不冒充有效点位。"""
    directory = Path(directory)
    if not directory.is_dir():
        raise ValueError(f"waypoint directory does not exist: {directory}")
    entries = []
    for path in sorted(directory.glob("*.json")):
        try:
            entries.append({"path": path, "record": load_waypoint(path), "error": None})
        except (OSError, ValueError) as exc:
            entries.append({"path": path, "record": None, "error": str(exc)})
    return entries
