"""复用受限 J4 小行程路线的关节探测 API，显式计划后执行。"""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.local_probe.local_joint_probe import plan_probe
from experiments.single_arm.start_transfer.go_to_start import execute_route, flange_envelope
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_route_workspace,
)


PLAN_DIR = Path("experiments/single_arm/lab/data/joint_probe_plans")
RUN_DIR = Path("experiments/single_arm/lab/data/joint_probes")


def _new_path(directory, prefix):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    return directory / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")


def _write(path, value, exclusive=False):
    with Path(path).open("x" if exclusive else "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def prepare(*, config_path=DEFAULT_CONFIG, robot_factory=None, speed_percent=5):
    if speed_percent not in (5,10):
        raise ValueError("J4 探测仅允许 5% 或 10%")
    config_path = Path(config_path)
    config = load_config(config_path)
    with connected(config, robot_factory) as robot:
        state = require_ready(read_state(robot))
        route = plan_probe(state["joint_rad"], config)
        validate_route_workspace(robot, route, config)
        envelope = flange_envelope(robot, route, config)
    return {"schema_version":1, "kind":"j4_local_probe",
            "created_at_utc":datetime.now(timezone.utc).isoformat(),
            "config_sha256":hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "start_joint_rad":route[0], "route_joint_rad":route,
            "flange_xyz_envelope_m":envelope, "speed_percent":speed_percent,
            "description":"J4 +0.05 rad 后返回本次实测起点；逐点 move_j"}


def save(plan, directory=PLAN_DIR):
    path = _new_path(directory, "nero_j4_probe_")
    _write(path, plan, exclusive=True)
    return path


def verify(plan, config_path=DEFAULT_CONFIG):
    config_path = Path(config_path)
    config = load_config(config_path)
    if (plan.get("schema_version") != 1 or plan.get("kind") != "j4_local_probe"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("speed_percent") not in (5,10)):
        raise ValueError("J4 探测计划版本、速度或配置不符")
    expected = plan_probe(plan["start_joint_rad"], config)
    if plan.get("route_joint_rad") != expected:
        raise ValueError("J4 探测路线被修改")
    return plan


def run(plan_path, *, config_path=DEFAULT_CONFIG, robot_factory=None,
        run_dir=RUN_DIR, countdown_s=5):
    plan_path = Path(plan_path)
    plan = verify(json.loads(plan_path.read_text(encoding="utf-8")), config_path)
    config = load_config(Path(config_path))
    report_path = _new_path(run_dir, "nero_j4_probe_")
    report = {"schema_version":1,"kind":"j4_local_probe",
              "started_at_utc":datetime.now(timezone.utc).isoformat(),
              "plan":str(plan_path),"completed":False,
              "last_attempted_step":0,"last_published_step":0,"last_reached_step":0,
              "feedback_samples":[]}
    _write(report_path, report, exclusive=True)
    old_handlers = None
    interrupted = False

    def stop(_number, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            state = require_ready(read_state(robot))
            if max(abs(a-b) for a,b in zip(state["joint_rad"],plan["start_joint_rad"])) > .002:
                raise RuntimeError("实时姿态偏离 J4 探测计划起点")
            route = plan["route_joint_rad"]
            validate_route_workspace(robot, route, config)
            flange_envelope(robot, route, config)
            old_handlers = (signal.signal(signal.SIGINT, stop),
                            signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s,0,-1):
                if interrupted: raise InterruptedError("倒计时取消")
                print(f"{remaining} 秒后 J4 小行程；按 Ctrl+C 取消。",flush=True)
                time.sleep(1)
            require_ready(read_state(robot))
            started=time.monotonic()
            execute_route(robot,config,route,lambda:interrupted,
                          on_command=lambda index,_:report.update(last_attempted_step=index),
                          on_command_published=lambda index,_:report.update(last_published_step=index),
                          on_waypoint=lambda index,_:report.update(last_reached_step=index),
                          on_feedback=lambda joints:report["feedback_samples"].append({
                              "elapsed_s":round(time.monotonic()-started,3),
                              "joint_rad":list(joints)}),
                          speed_percent=plan["speed_percent"],
                          target_tolerance_rad=.005,waypoint_timeout_s=5,
                          waypoint_max_timeout_s=15,pause_on_stationary_timeout=True)
            final=require_ready(read_state(robot))
            error=max(abs(a-b) for a,b in zip(final["joint_rad"],route[0]))
            if error>.005: raise RuntimeError("J4 探测未返回本次起点")
            report.update(completed=True,final_status=final,
                          final_max_joint_error_rad=error)
    except Exception as exc:
        report.update(error=str(exc),error_type=type(exc).__name__)
        raise RuntimeError(f"J4 小行程未完成：{exc}；报告 {report_path}") from exc
    finally:
        if old_handlers is not None:
            signal.signal(signal.SIGINT,old_handlers[0])
            signal.signal(signal.SIGTERM,old_handlers[1])
        report["finished_at_utc"]=datetime.now(timezone.utc).isoformat()
        _write(report_path,report)
    return report_path
