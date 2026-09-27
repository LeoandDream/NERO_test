"""六个有界拖动阶段采面；从停留段提取真实法兰点并尝试几何拟合。

每段对应操作者指定的一个虚拟立方体面。程序只采集反馈，绝不根据
极值臆造面点，也不发布回位目标或把拟合结果写进运动许可配置。
"""

import csv
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys

from experiments.single_arm.lab import cube
from experiments.single_arm.lab.cube_sweep import _command
from experiments.single_arm.teaching.teach_session import load_config


REPORT_DIR = Path("experiments/single_arm/workspace_cube/data/face_sweeps")


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _distance(a, b):
    return math.dist(a, b)


def _triangle_area(a, b, c):
    u = [x-y for x, y in zip(b, a)]
    v = [x-y for x, y in zip(c, a)]
    cross = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2],
             u[0]*v[1]-u[1]*v[0]]
    return math.sqrt(sum(x*x for x in cross))/2


def select_dwell_triplet(rows, *, min_dwell_s=0.25, radius_m=0.003,
                         separation_m=0.02):
    """只选该面录制段内三个分散且停稳的实测点，保留源行索引。

    单段内若没有足够的停留点，就返回空列表，让拟合以缺面状态结束。
    路过某点时的一帧极值无法成为面样本。
    """
    if not rows:
        return []
    points = []
    for row in rows:
        xyz = [float(row[key]) for key in ("x_m", "y_m", "z_m")]
        elapsed = float(row["elapsed_s"])
        if not all(math.isfinite(value) for value in xyz+[elapsed]):
            raise ValueError("六面录制含非有限法兰反馈或时间")
        lag = abs(float(row["joint_feedback_unix_s"])
                  - float(row["pose_feedback_unix_s"]))
        if not math.isfinite(lag) or lag > 0.05:
            # 原始行仍在 CSV；不同步的行不得成为几何面点。
            continue
        points.append((xyz, elapsed, row))
    if not points:
        return []
    dwell = []
    first = 0
    while first < len(points):
        end = first+1
        while end < len(points) and _distance(points[first][0], points[end][0]) <= radius_m:
            end += 1
        if points[end-1][1]-points[first][1] >= min_dwell_s:
            middle = (first+end-1)//2
            dwell.append({"xyz":points[middle][0], "row":points[middle][2],
                          "source_start_index":first,
                          "source_end_index":end-1,
                          "dwell_s":points[end-1][1]-points[first][1]})
        first = end
    if len(dwell) < 3:
        return []
    # 六段通常各只有数十个停留候选；去掉同一物理停留被拆成的近邻。
    distinct = []
    for item in dwell:
        if all(_distance(item["xyz"], other["xyz"]) >= separation_m
               for other in distinct):
            distinct.append(item)
    if len(distinct) < 3:
        return []
    triplet = max(itertools.combinations(distinct, 3),
                  key=lambda group: _triangle_area(*(item["xyz"] for item in group)))
    if _triangle_area(*(item["xyz"] for item in triplet)) < 1e-6:
        return []
    return list(triplet)


def run_face_sweep(api, *, seconds_per_face=12, rate_hz=20, phase_gate=None,
                   command_runner=None, report_dir=None, session_dir=None,
                   session_path=None):
    """每面准备时保持非拖动，采样时才进入拖动，结束时总是退出。"""
    if not (math.isfinite(seconds_per_face) and 3 <= seconds_per_face <= 60):
        raise ValueError("每面录制时长须在 3～60 秒")
    if not (math.isfinite(rate_hz) and 5 <= rate_hz <= 20):
        raise ValueError("六面录制频率须在 5～20 Hz")
    config_path = Path(api.config_path)
    config = load_config(config_path)
    if session_path is None:
        existing_counts = {face: 0 for face in cube.FACES}
    else:
        session = Path(session_path)
        # 复用时核对安装/零点配置和已有点的来源；不能只凭面点数量跳过一面。
        existing_counts = cube.session_status(session, config_path)["face_sample_counts"]
        for source in session.glob("sample_*.json"):
            sample = json.loads(source.read_text(encoding="utf-8"))
            recording = sample.get("source_recording")
            if recording is not None:
                path = Path(recording)
                if (not path.is_file() or
                        hashlib.sha256(path.read_bytes()).hexdigest()
                        != sample.get("source_sha256")):
                    raise ValueError("已有面点的原始录制缺失或哈希不符，拒绝续录")
    reused_faces = [face for face in cube.FACES if existing_counts[face] >= 3]
    if session_path is None:
        session = cube.create_session(config_path, directory=session_dir or cube.SESSION_DIR)
    report_dir = Path(report_dir or REPORT_DIR)
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / ("nero_cube_faces_"+datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ")+".json")
    report = {"schema_version":2, "kind":"six_face_guided_drag_sweep",
              "started_at_utc":_utc_now(), "completed":False,
              "session_path":str(session), "config_path":str(config_path),
              "config_sha256":hashlib.sha256(config_path.read_bytes()).hexdigest(),
              "mount":config["mount"], "coordinate_frame":"robot_base",
              "reference_point":"flange_center", "tool_offset_m":[0,0,0],
              "seconds_per_face":seconds_per_face, "rate_hz":rate_hz,
              "drag_start_check":"can_drag_teach --start 内部核对实时状态和七轴驱动",
              "faces":{}, "face_attempts":[],
              "continued_session":session_path is not None,
              "existing_sample_counts":existing_counts,
              "reused_faces":reused_faces,
              "automatic_return":False,
              "geometry_valid":False, "motion_region_verified":False}
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",
                           encoding="utf-8")
    runner = command_runner or subprocess.run
    base = [sys.executable,"-m","experiments.single_arm.can_setup.can_drag_teach"]
    failure = None
    try:
        for face in cube.FACES:
            if face in reused_faces:
                print(f"{face} 面已有 {existing_counts[face]} 个点，跳过录制；"
                      "最终几何拟合仍会复核。", flush=True)
                continue
            attempt = {"face":face,"started_at_utc":_utc_now()}
            report["face_attempts"].append(attempt)
            phase_failure = None
            start_attempted = False
            try:
                # 操作者准备或交互等待必须在拖动开始之前；等待回车没有时限。
                # 开始后立即进入有时限的反馈录制，不在拖动状态等待下一面。
                if phase_gate is not None:
                    phase_gate(face)
                start_attempted = True
                attempt["start"] = _command(
                    runner,base+["--start","--mount",config["mount"],
                                 "--channel",config["channel"]])
                if attempt["start"]["returncode"] != 0:
                    raise RuntimeError(f"{face} 面 CAN 拖动开始命令未确认")
                print(f"开始 {face} 面 {seconds_per_face:g} 秒录制；"
                      "在该面三个分散位置各停稳至少 0.3 秒。",flush=True)
                recording = api.record_poses(duration_s=seconds_per_face,
                                              rate_hz=rate_hz,required_state="drag")
                attempt["recording"] = recording
            except (Exception,KeyboardInterrupt) as exc:
                phase_failure = exc
            finally:
                if start_attempted:
                    try:
                        attempt["stop"] = _command(
                            runner,base+["--stop","--channel",config["channel"]])
                        if attempt["stop"]["returncode"] != 0:
                            raise RuntimeError(f"{face} 面 CAN 拖动结束命令未确认")
                    except (Exception,KeyboardInterrupt) as exc:
                        attempt["stop_error"] = str(exc)
                        phase_failure = phase_failure or exc
                attempt["finished_at_utc"] = _utc_now()
                report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",
                                       encoding="utf-8")
            if phase_failure is not None:
                raise phase_failure
            source = Path(recording["recording_path"])
            with source.open(newline="",encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            selected = select_dwell_triplet(rows)
            face_entry = {"recording":recording, "selected_dwell_count":len(selected),
                          "source_rows":len(rows), "sample_paths":[]}
            for item in selected:
                row = item["row"]
                pose = [float(row[key]) for key in ("x_m","y_m","z_m",
                                                  "roll_rad","pitch_rad","yaw_rad")]
                sample = {"schema_version":cube.FIT_VERSION,
                          "captured_at_utc":row["captured_at_utc"], "face":face,
                          "joint_rad":[float(row[f"joint_{i}_rad"]) for i in range(1,8)],
                          "flange_pose_m_rad":pose,
                          "joint_feedback_unix_s":float(row["joint_feedback_unix_s"]),
                          "pose_feedback_unix_s":float(row["pose_feedback_unix_s"]),
                          "source_recording":str(source),
                          "source_sha256":recording["sha256"],
                          "source_start_index":item["source_start_index"],
                          "source_end_index":item["source_end_index"],
                          "source_sample_index":int(row["sample_index"]),
                          "dwell_s":item["dwell_s"],
                          "config_sha256":report["config_sha256"],
                          "mount":config["mount"],
                          "coordinate_frame":"robot_base",
                          "reference_point":"flange_center"}
                sample_path = session / ("sample_"+datetime.now(timezone.utc).strftime(
                    "%Y%m%dT%H%M%S%fZ")+".json")
                with sample_path.open("x",encoding="utf-8") as stream:
                    json.dump(sample,stream,ensure_ascii=False,indent=2)
                    stream.write("\n")
                face_entry["sample_paths"].append(str(sample_path))
            report["faces"][face] = face_entry
            report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",
                                   encoding="utf-8")
            print(f"{face} 面保存 {len(selected)} 个停留样本。",flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        failure = exc
    if failure is None:
        try:
            fit_path, fit = cube.fit_session(session,config_path)
            report["fit_path"] = str(fit_path)
            report["geometry_valid"] = fit["geometry_valid"]
            report["fit_reason"] = fit["reason"]
            report["completed"] = True
        except (Exception,KeyboardInterrupt) as exc:
            failure = exc
    if failure is not None:
        report["error_type"] = type(failure).__name__
        report["error"] = str(failure)
    report["finished_at_utc"] = _utc_now()
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",
                           encoding="utf-8")
    if failure is not None:
        raise RuntimeError(f"六面分段录制未完成：{failure}；报告 {report_path}") from failure
    return {"report_path":str(report_path), "session_path":str(session),
            "fit_path":report["fit_path"], "geometry_valid":fit["geometry_valid"],
            "fit_reason":fit["reason"], "automatic_return":False}
