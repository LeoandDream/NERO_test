"""版本化环境资产：只核对 F1 点位及命名路线，不证明现场许可。"""

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile

from .waypoints import load_waypoint


SCHEMA_VERSION = 1
STATES = frozenset(("candidate", "validated", "locked", "retired"))
ROLES = frozenset(("ready", "park", "waypoint"))
OPERATIONS = frozenset(("start", "park", "return_ready"))
_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_SHA = re.compile(r"[0-9a-f]{64}\Z")


@dataclass(frozen=True)
class PointAsset:
    name: str
    role: str
    source_waypoint_path: Path
    source_waypoint_sha256: str
    state: str
    joint_rad: tuple[float, ...]


@dataclass(frozen=True)
class RouteAsset:
    route_id: str
    from_point: str
    to_point: str
    ordered_point_refs: tuple[str, ...]
    state: str
    directional: bool
    evidence_ref: str
    allowed_operations: tuple[str, ...]


@dataclass(frozen=True)
class EnvironmentProfile:
    schema_version: int
    environment_id: str
    environment_version: str
    robot_id: str
    mount: str
    tool_id: str
    points: tuple[PointAsset, ...]
    routes: tuple[RouteAsset, ...]
    source_path: Path
    source_sha256: str

    def point(self, name: str) -> PointAsset | None:
        return next((point for point in self.points if point.name == name), None)

    def route(self, route_id: str) -> RouteAsset | None:
        return next((route for route in self.routes if route.route_id == route_id), None)


def _text(value, label: str, *, identifier=False) -> str:
    if type(value) is not str or not value.strip() or value != value.strip():
        raise ValueError(f"{label} must be nonempty text")
    if identifier and _ID.fullmatch(value) is None:
        raise ValueError(f"{label} must be an ASCII identifier")
    return value


def _keys(value, expected: set[str], label: str) -> dict:
    if type(value) is not dict or set(value) != expected:
        raise ValueError(f"{label} fields are incomplete or unknown")
    return value


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate environment JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"nonstandard environment JSON number: {value}")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _parse_profile(data: object, path: Path, source_sha256: str) -> EnvironmentProfile:
    data = _keys(data, {"schema_version", "environment_id", "environment_version",
                        "robot_id", "mount", "tool_id", "points", "routes"}, "profile")
    if type(data["schema_version"]) is not int or data["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unknown environment schema version")
    environment_id = _text(data["environment_id"], "environment_id", identifier=True)
    version = _text(data["environment_version"], "environment_version", identifier=True)
    if (path.name != f"{version}.json" or path.parent.name != environment_id or
            path.parent.parent.name != "environment"):
        raise ValueError("profile path must be environment/<environment_id>/<version>.json")
    robot_id = _text(data["robot_id"], "robot_id")
    mount = _text(data["mount"], "mount")
    tool_id = _text(data["tool_id"], "tool_id")
    if type(data["points"]) is not list or type(data["routes"]) is not list:
        raise ValueError("profile points and routes must be lists")
    points = []
    names = set()
    for item in data["points"]:
        item = _keys(item, {"name", "role", "source_waypoint_path",
                            "source_waypoint_sha256", "state"}, "point")
        name = _text(item["name"], "point name")
        if name in names:
            raise ValueError(f"duplicate point: {name}")
        names.add(name)
        if (type(item["role"]) is not str or item["role"] not in ROLES or
                type(item["state"]) is not str or item["state"] not in STATES):
            raise ValueError(f"point {name}: unknown role or state")
        source = _text(item["source_waypoint_path"], "source_waypoint_path")
        digest = item["source_waypoint_sha256"]
        if type(digest) is not str or _SHA.fullmatch(digest) is None:
            raise ValueError(f"point {name}: invalid SHA256")
        source_path = Path(source)
        if not source_path.is_absolute():
            source_path = (path.parent / source_path).resolve()
        if _sha256(source_path) != digest:
            raise ValueError(f"point {name}: source waypoint SHA256 mismatch")
        record = load_waypoint(source_path)
        if _sha256(source_path) != digest:
            raise ValueError(f"point {name}: source waypoint changed during load")
        if record["name"] != name or record["robot_id"] != robot_id:
            raise ValueError(f"point {name}: waypoint name or robot identity mismatch")
        points.append(PointAsset(name, item["role"], source_path, digest,
                                 item["state"], tuple(record["snapshot"]["joint_rad"])))
    routes = []
    route_ids = set()
    for item in data["routes"]:
        item = _keys(item, {"route_id", "from_point", "to_point", "ordered_point_refs",
                            "state", "directional", "evidence_ref", "allowed_operations"}, "route")
        route_id = _text(item["route_id"], "route_id", identifier=True)
        if route_id in route_ids:
            raise ValueError(f"duplicate route: {route_id}")
        route_ids.add(route_id)
        origin = _text(item["from_point"], "from_point")
        destination = _text(item["to_point"], "to_point")
        refs = item["ordered_point_refs"]
        if (type(refs) is not list or len(refs) < 2 or
                any(type(ref) is not str or ref not in names for ref in refs) or
                refs[0] != origin or refs[-1] != destination):
            raise ValueError(f"route {route_id}: references or direction are invalid")
        if (type(item["state"]) is not str or item["state"] not in STATES or
                type(item["directional"]) is not bool):
            raise ValueError(f"route {route_id}: state or directional flag is invalid")
        operations = item["allowed_operations"]
        if (type(operations) is not list or not operations or
                any(type(op) is not str or op not in OPERATIONS for op in operations) or
                len(operations) != len(set(operations))):
            raise ValueError(f"route {route_id}: allowed_operations are invalid")
        evidence = _text(item["evidence_ref"], "evidence_ref")
        routes.append(RouteAsset(route_id, origin, destination, tuple(refs),
                                 item["state"], item["directional"], evidence,
                                 tuple(operations)))
    return EnvironmentProfile(SCHEMA_VERSION, environment_id, version, robot_id,
                              mount, tool_id, tuple(points), tuple(routes), path,
                              source_sha256)


def load_environment_profile(path: str | Path) -> EnvironmentProfile:
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_pairs,
                      parse_constant=_reject_constant)
    profile = _parse_profile(data, path, hashlib.sha256(raw).hexdigest())
    if _sha256(path) != profile.source_sha256:
        raise ValueError("environment profile changed during load")
    return profile


def save_environment_profile(data: dict, path: str | Path) -> Path:
    """校验原件后建立新版本；绝不覆盖已有版本文件。"""
    path = Path(path)
    payload = (json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode()
    _parse_profile(data, path, hashlib.sha256(payload).hexdigest())
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"environment profile already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".environment-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return path


def format_environment_profile(profile: EnvironmentProfile) -> str:
    lines = [f"Environment: {profile.environment_id}/{profile.environment_version}",
             f"Robot / mount / tool: {profile.robot_id} / {profile.mount} / {profile.tool_id}",
             f"Source SHA256: {profile.source_sha256}"]
    lines += [f"Point {point.name}: role={point.role}; state={point.state}; SHA256={point.source_waypoint_sha256}"
              for point in profile.points]
    lines += [f"Route {route.route_id}: {route.from_point} → {route.to_point}; "
              f"state={route.state}; operations={','.join(route.allowed_operations)}; "
              f"evidence={route.evidence_ref}" for route in profile.routes]
    lines.append("Profile asset state does not grant current-site motion permission.")
    return "\n".join(lines)
