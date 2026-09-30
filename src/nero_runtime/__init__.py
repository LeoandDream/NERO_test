"""NERO 单臂 Runtime 的纯 Python 使用面。

导出在首次访问时才加载；仅 `import nero_runtime` 不读取现场或创建设备。
执行内核须从子模块显式导入并注入经现场核实的依赖。
"""

from importlib import import_module


_EXPORTS = {
    "RobotSnapshot": "snapshot",
    "SourceQuality": "snapshot",
    "capture_waypoint": "waypoints",
    "save_waypoint": "waypoints",
    "load_waypoint": "waypoints",
    "list_waypoints": "waypoints",
    "RoutePoint": "motion",
    "JointRoute": "motion",
    "RouteParameters": "motion",
    "MotionResult": "motion",
    "StopEvidence": "motion",
    "prepare_joint_route": "motion",
    "RecordingParameters": "recording",
    "load_recording_summary": "recording",
    "list_recordings": "recording",
    "ReplayPlan": "replay",
    "ReplayResult": "replay",
    "prepare_replay": "replay",
    "replay_route": "replay",
    "PointAsset": "environment",
    "RouteAsset": "environment",
    "EnvironmentProfile": "environment",
    "load_environment_profile": "environment",
    "save_environment_profile": "environment",
    "EnvironmentIdentity": "workflows",
    "OperationPlan": "workflows",
    "OperationResult": "workflows",
    "TeachReturnPlan": "workflows",
    "TeachV1Result": "workflows",
    "prepare_start": "workflows",
    "prepare_park": "workflows",
    "prepare_return_ready": "workflows",
    "prepare_teach_return": "workflows",
}

__all__ = tuple(_EXPORTS)


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(f".{module_name}", __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))
