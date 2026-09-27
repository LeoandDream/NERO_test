"""六面法兰采样拟合基坐标系中的几何立方体；不授予运动许可。"""

from datetime import datetime, timezone
import csv
import hashlib
import json
import math
from pathlib import Path
import time

from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


FACES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
SESSION_DIR = Path("experiments/single_arm/workspace_cube/data/sessions")
CANDIDATE_DIR = Path("experiments/single_arm/workspace_cube/data/candidates")
FIT_VERSION = 1


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def sub(a, b):
    return [x-y for x, y in zip(a, b)]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0]]


def unit(a):
    length = math.sqrt(dot(a, a))
    if length < 1e-12:
        raise ValueError("面点没有足够的空间跨度")
    return [x/length for x in a]


def _smallest_eigenvector(matrix):
    """对称 3×3 协方差的 Jacobi 迭代，避免引入重型依赖。"""
    a = [row[:] for row in matrix]
    v = [[float(i == j) for j in range(3)] for i in range(3)]
    for _ in range(32):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(a[pair[0]][pair[1]]))
        if abs(a[p][q]) < 1e-15:
            break
        angle = 0.5*math.atan2(2*a[p][q], a[q][q]-a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        rotation = [[float(i == j) for j in range(3)] for i in range(3)]
        rotation[p][p], rotation[q][q] = c, c
        rotation[p][q], rotation[q][p] = s, -s
        a = [[sum(rotation[k][i]*a[k][l]*rotation[l][j]
                  for k in range(3) for l in range(3))
              for j in range(3)] for i in range(3)]
        v = [[sum(v[i][k]*rotation[k][j] for k in range(3))
              for j in range(3)] for i in range(3)]
    index = min(range(3), key=lambda i: a[i][i])
    return unit([v[i][index] for i in range(3)])


def _plane(points):
    if len(points) < 3:
        raise ValueError("每个面至少需要 3 个不同位置的样本")
    centroid = [sum(point[i] for point in points)/len(points) for i in range(3)]
    vectors = [sub(point, centroid) for point in points]
    max_area = max((math.sqrt(dot(cross(sub(b,a), sub(c,a)),
                                  cross(sub(b,a), sub(c,a))))/2
                    for i,a in enumerate(points) for j,b in enumerate(points)
                    for c in points[j+1:] if j > i), default=0.0)
    if max_area < 1e-6:
        raise ValueError("单面的三个采样点几乎共线或过于集中")
    cov = [[sum(row[i]*row[j] for row in vectors) for j in range(3)]
           for i in range(3)]
    normal = _smallest_eigenvector(cov)
    residual = max(abs(dot(normal, sub(point, centroid))) for point in points)
    return centroid, normal, residual, max_area


def fit_cube(samples):
    """返回几何结果与失败原因；不会把立方体写成安全工作空间。"""
    grouped = {face: [] for face in FACES}
    for sample in samples:
        face = sample.get("face")
        pose = sample.get("flange_pose_m_rad")
        if face not in grouped or not isinstance(pose, list) or len(pose) != 6:
            raise ValueError("样本面标签或法兰六维位姿无效")
        point = [float(value) for value in pose[:3]]
        if not all(math.isfinite(value) for value in point):
            raise ValueError("样本含非有限位置")
        grouped[face].append(point)
    counts = {face: len(points) for face, points in grouped.items()}
    missing = [face for face in FACES if counts[face] < 3]
    base = {"schema_version": FIT_VERSION, "coordinate_frame": "robot_base",
            "reference_point": "flange_center", "face_sample_counts": counts,
            "geometry_valid": False, "motion_region_verified": False,
            "missing_or_undercovered_faces": missing}
    if missing:
        return {**base, "reason": "六个面尚未各采集至少三个点"}
    try:
        planes = {face: _plane(points) for face, points in grouped.items()}
        raw_axes, separations = [], []
        for axis in "XYZ":
            pos, neg = planes["+"+axis], planes["-"+axis]
            between = sub(pos[0], neg[0])
            npos = pos[1] if dot(pos[1], between) > 0 else [-x for x in pos[1]]
            nneg = neg[1] if dot(neg[1], between) < 0 else [-x for x in neg[1]]
            if dot(npos, nneg) > -math.cos(math.radians(8)):
                raise ValueError(f"{axis} 轴对面不平行")
            direction = unit(sub(npos, nneg))
            length = dot(direction, between)
            if length <= 0.02:
                raise ValueError("立方体边长须大于 20 mm")
            raw_axes.append(direction)
            separations.append(length)
        for i in range(3):
            for j in range(i+1, 3):
                if abs(dot(raw_axes[i], raw_axes[j])) > math.sin(math.radians(8)):
                    raise ValueError("相邻面法向不正交")
        ex = raw_axes[0]
        ey = unit([a-dot(raw_axes[1], ex)*b for a,b in zip(raw_axes[1], ex)])
        ez = unit(cross(ex, ey))
        if dot(ez, raw_axes[2]) < math.cos(math.radians(8)):
            raise ValueError("六面标签不是一致的右手坐标系")
        axes = (ex, ey, ez)
        side = sum(separations)/3
        if max(abs(length-side)/side for length in separations) > 0.05:
            raise ValueError("三边不等长，只能称长方体采样，不能称立方体")
        mid_offsets = [(dot(axis, planes["+"+label][0])
                        + dot(axis, planes["-"+label][0]))/2
                       for axis,label in zip(axes, "XYZ")]
        center = [sum(mid_offsets[i]*axes[i][coordinate] for i in range(3))
                  for coordinate in range(3)]
        residuals = {}
        coverage = {}
        for axis,label in zip(axes, "XYZ"):
            for sign in (1, -1):
                face = ("+" if sign == 1 else "-")+label
                residuals[face] = max(abs(dot(axis, sub(point, center))-sign*side/2)
                                      for point in grouped[face])
                coverage[face] = planes[face][3]/(side*side)
        if max(residuals.values()) > 0.005:
            raise ValueError("面点偏离等边立方体拟合面超过 5 mm")
        if min(coverage.values()) < 0.02:
            raise ValueError("某个面的采样三角形覆盖不足")
        corners = [[center[i]+sum(signs[j]*side/2*axes[j][i] for j in range(3))
                    for i in range(3)]
                   for signs in ((sx,sy,sz) for sx in (-1,1)
                                 for sy in (-1,1) for sz in (-1,1))]
        return {**base, "geometry_valid": True, "reason": None,
                "center_base_m": center, "side_length_m": side,
                "measured_side_lengths_m": dict(zip("xyz", separations)),
                "cube_axes_in_base": dict(zip("xyz", axes)),
                "corners_base_m": corners, "max_face_residual_m": residuals,
                "face_triangle_area_fraction": coverage,
                "status": "GEOMETRY_ONLY_UNVERIFIED_MOTION"}
    except ValueError as exc:
        return {**base, "reason": str(exc)}


def create_session(config_path=DEFAULT_CONFIG, directory=SESSION_DIR):
    config_path = Path(config_path)
    config = load_config(config_path)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = Path(directory) / f"nero_cube_{stamp}"
    path.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema_version": FIT_VERSION, "calibration_version": "cube_faces_v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "config_path": str(config_path), "mount": config["mount"],
        "zero_reference": config["safe_start_source"], "robot_model": "NERO",
        "firmware_driver": "V121", "coordinate_frame": "robot_base",
        "reference_point": "flange_center", "tool_offset_m": [0,0,0],
        "note": "六面为立方体局部 ±X/±Y/±Z；每面至少三个不共线的法兰点。",
    }
    with (path / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def _manifest(path, config_path):
    path = Path(path)
    manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
    config = load_config(config_path)
    if (manifest.get("config_sha256") != hashlib.sha256(Path(config_path).read_bytes()).hexdigest()
            or manifest.get("mount") != config["mount"]):
        raise ValueError("标定会话与当前安装配置不符")
    return manifest


def capture_face(session, face, config_path=DEFAULT_CONFIG, robot_factory=None):
    """只读采集一个已停稳的法兰点；每次存独立原始 JSON。"""
    if face not in FACES:
        raise ValueError("面标签须为 +X/-X/+Y/-Y/+Z/-Z")
    manifest = _manifest(session, config_path)
    if robot_factory is None:
        from experiments.single_arm.continuous_replay.continuous_session import robot_instance
        robot_factory = robot_instance
    from experiments.single_arm.can_setup.can_drag_teach import fresh
    config = load_config(config_path)
    robot = robot_factory(config["channel"])
    robot.connect()
    try:
        snapshots = []
        deadline = time.monotonic() + 3.0
        for index in range(2):
            while True:
                status, joints, pose = (robot.get_arm_status(), robot.get_joint_angles(),
                                        robot.get_flange_pose())
                if all(fresh(item, 0.25) for item in (status, joints, pose)):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("CAN 状态、关节或法兰反馈不新鲜")
                time.sleep(0.05)
            mode = int(status.msg.ctrl_mode)
            arm = int(status.msg.arm_status)
            teach = int(status.msg.teach_status)
            # CAN 拖动示教在实机上报告 ctrl_mode=2、arm_status=0、teach_status=1。
            # 不能把它误当作仅 ctrl_mode=1 的空闲状态；两种状态都只读采样。
            idle = mode == 1 and arm == 0 and teach in (0, 2, 6)
            drag = mode == 2 and arm in (0, 11) and teach == 1
            if not (idle or drag) or status.msg.err_code:
                raise RuntimeError("当前状态不允许只读标定采样")
            snapshots.append(([float(x) for x in joints.msg],
                              [float(x) for x in pose.msg], status,
                              float(joints.timestamp)))
            if index == 0:
                time.sleep(0.25)
        first, second = snapshots
        if max(abs(a-b) for a,b in zip(first[0], second[0])) > 0.002 or math.dist(first[1][:3], second[1][:3]) > 0.002:
            raise RuntimeError("法兰尚未停稳，请固定在面上后重试")
        if math.dist(robot.fk(second[0])[:3], second[1][:3]) > 0.02:
            raise RuntimeError("法兰反馈与正运动学不一致")
        data = {"schema_version": FIT_VERSION, "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "face": face, "joint_rad": second[0], "flange_pose_m_rad": second[1],
                "joint_feedback_unix_s": second[3],
                "config_sha256": manifest["config_sha256"],
                "mount": manifest["mount"], "coordinate_frame": "robot_base",
                "reference_point": "flange_center", "arm_status": int(second[2].msg.arm_status),
                "teach_status": int(second[2].msg.teach_status)}
        path = Path(session) / ("sample_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
        with path.open("x", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        return path
    finally:
        robot.disconnect()


def fit_session(session, config_path=DEFAULT_CONFIG):
    manifest = _manifest(session, config_path)
    files = sorted(Path(session).glob("sample_*.json"))
    samples = [json.loads(path.read_text(encoding="utf-8")) for path in files]
    for sample in samples:
        if sample.get("config_sha256") != manifest["config_sha256"]:
            raise ValueError("原始样本与标定会话配置不匹配")
    result = fit_cube(samples)
    result.update({"created_at_utc": datetime.now(timezone.utc).isoformat(),
                   "session": str(session), "manifest": manifest,
                   "raw_sample_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                         for path in files},
                   "raw_sample_count": len(samples)})
    output = Path(session) / ("fit_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return output, result


def session_status(session, config_path=DEFAULT_CONFIG):
    """只读显示六面原始采样覆盖；不能把数量当作几何或运动验收。"""
    manifest = _manifest(session, config_path)
    files = sorted(Path(session).glob("sample_*.json"))
    samples = [json.loads(path.read_text(encoding="utf-8")) for path in files]
    if any(sample.get("config_sha256") != manifest["config_sha256"] for sample in samples):
        raise ValueError("采样与当前标定配置不符")
    counts = {face:sum(sample.get("face") == face for sample in samples)
              for face in FACES}
    return {"session":str(session), "sample_count":len(samples),
            "face_sample_counts":counts,
            "undercovered_faces":[face for face,count in counts.items() if count < 3],
            "geometry_status":"UNFIT_OR_REQUIRES_REVIEW",
            "motion_region_verified":False}


def extrema_candidate(map_csv, *, level="new_full_continuous_path_sample",
                      margin_m=0.01, bottom_margin_m=None):
    """从一种实测轨迹提取六个单轴极值，生成轴对齐立方体候选。

    六个极值可能来自六个互不连通的位置；候选面中心和角点未必被
    轨迹经过。返回值明确不是六面标定，也不授予任何运动许可。
    """
    path = Path(map_csv)
    if not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError("内缩余量须为非负有限米数")
    if bottom_margin_m is None:
        bottom_margin_m = margin_m
    if not math.isfinite(bottom_margin_m) or bottom_margin_m < margin_m:
        raise ValueError("离桌底部余量须不小于普通内缩余量")
    metadata_path = path.with_suffix(".json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if level not in metadata.get("levels", {}):
        raise ValueError(f"实测地图没有证据级别 {level}")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row.get("level") == level]
    if len(rows) != metadata["levels"][level]["samples"] or len(rows) < 6:
        raise ValueError("实测轨迹样本数量与地图元数据不符")
    points = []
    for row in rows:
        xyz = [float(row[f"{axis}_m"]) for axis in "xyz"]
        joints = json.loads(row["joint_rad"])
        if (not all(math.isfinite(value) for value in xyz)
                or len(joints) != 7
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in joints)):
            raise ValueError("实测轨迹包含无效法兰位置或七轴反馈")
        points.append((xyz, row, joints))
    extrema = {}
    mins, maxs = [], []
    for index, axis in enumerate("xyz"):
        lower = min(points, key=lambda item: item[0][index])
        upper = max(points, key=lambda item: item[0][index])
        mins.append(lower[0][index])
        maxs.append(upper[0][index])
        extrema[axis] = {
            bound: {"flange_xyz_base_m": item[0], "joint_rad": item[2],
                    "source": item[1]["source"],
                    "source_index": item[1]["source_index"],
                    "time_utc": item[1]["time_utc"]}
            for bound, item in (("min", lower), ("max", upper))}
    spans = [high-low for low, high in zip(mins, maxs)]
    # 左侧安装现场已确认基座 X 正向离桌，因此额外的底部余量加在 -X。
    lower_inset = [mins[0]+bottom_margin_m, mins[1]+margin_m,
                   mins[2]+margin_m]
    upper_inset = [value-margin_m for value in maxs]
    inset_spans = [high-low for low,high in zip(lower_inset,upper_inset)]
    side = min(inset_spans)
    if side <= 0.02:
        raise ValueError("最窄轴实测跨度扣除余量后不足 20 mm，不能提出立方体候选")
    center = [(low+high)/2 for low, high in zip(lower_inset, upper_inset)]
    cube_bounds = {axis:[center[i]-side/2, center[i]+side/2]
                   for i,axis in enumerate("xyz")}
    vertices = [
        {"corner":("+" if sx > 0 else "-") + ("+" if sy > 0 else "-")
                   + ("+" if sz > 0 else "-"),
         "xyz_base_m":[center[i]+sign*side/2
                       for i,sign in enumerate((sx,sy,sz))],
         "observed_point":False}
        for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)
    ]
    interior = [item for item in points if all(
        lower_inset[i] <= item[0][i] <= upper_inset[i] for i in range(3))]
    if not interior:
        raise ValueError("内缩后没有任何实测姿态，不能推荐极值点")
    measured_targets = {}
    for index, axis in enumerate("XYZ"):
        for sign, bound in ((-1,"min"),(1,"max")):
            item = (min(interior,key=lambda candidate:candidate[0][index])
                    if sign < 0 else
                    max(interior,key=lambda candidate:candidate[0][index]))
            measured_targets[("-" if sign < 0 else "+")+axis] = {
                "flange_xyz_base_m": item[0], "joint_rad": item[2],
                "source": item[1]["source"],
                "source_index": item[1]["source_index"],
                "time_utc": item[1]["time_utc"],
                "coordinate_offset_from_raw_extremum_m":
                    (item[0][index]-mins[index] if sign < 0 else
                     maxs[index]-item[0][index]),
                "observed_point_only": True,
            }
    faces = {}
    for index, axis in enumerate("XYZ"):
        for sign, symbol in ((1, "+"), (-1, "-")):
            point = center.copy()
            point[index] += sign*side/2
            faces[symbol+axis] = {
                "candidate_center_xyz_base_m": point,
                "nearest_recorded_distance_m": min(math.dist(point, xyz)
                                                   for xyz, _, _ in points),
                "plane_in_base": {"X": "Y-Z", "Y": "X-Z", "Z": "X-Y"}[axis],
            }
    return {
        "schema_version": 1,
        "status": "EXTREMA_CANDIDATE_ONLY",
        "geometry_valid": False,
        "motion_region_verified": False,
        "six_faces_sampled": False,
        "coordinate_frame": "robot_base",
        "reference_point": "flange_center",
        "source_map_csv": str(path),
        "source_map_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_recording_csv":metadata.get("source_recording_csv"),
        "source_recording_sha256":metadata.get("source_recording_sha256"),
        "evidence_level": level,
        "sample_count": len(points),
        "axis_ranges_m": {axis: [mins[i], maxs[i]] for i, axis in enumerate("xyz")},
        "axis_spans_m": dict(zip("xyz", spans)),
        "axis_extrema": extrema,
        "axis_extrema_distinct_source_rows":len({
            (item["source"],item["source_index"])
            for axis in extrema.values() for item in axis.values()}),
        "inset_axis_ranges_m": {axis:[lower_inset[i],upper_inset[i]]
                                for i,axis in enumerate("xyz")},
        "physical_bottom_in_base": "-X (left mount, on-site confirmed)",
        "bottom_inset_m": bottom_margin_m,
        "recommended_measured_extrema": measured_targets,
        "candidate_center_base_m": center,
        "candidate_side_length_m": side,
        "candidate_cube_bounds_base_m":cube_bounds,
        "candidate_vertices_base_m":vertices,
        "construction_method":"一条实测轨迹的 XYZ 六个单轴极值；按余量内缩后取最窄轴边长，构造基座轴对齐等边立方体",
        "candidate_vertices_observed":False,
        "inset_from_narrowest_extrema_m": (
            margin_m if bottom_margin_m == margin_m else None),
        "ordinary_inset_m": margin_m,
        "candidate_face_centers": faces,
        "interpretation": "六个单轴极值来自已记录轨迹；未证明立方体六面或内部、角点、连杆和线缆可达/无碰撞。",
    }


def save_extrema_candidate(candidate, directory=CANDIDATE_DIR):
    path = Path(directory) / ("nero_cube_extrema_" +
                              datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(candidate, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def map_from_drag_recording(recording_csv, directory=CANDIDATE_DIR):
    """把完整的拖动位姿 CSV 转成带来源的离线极值地图。

    只接受每一行均处于正常拖动状态的连续反馈；不推断接触边界。
    原始 CSV 原样保留，地图与元数据始终创建新文件。
    """
    source = Path(recording_csv)
    with source.open(newline="", encoding="utf-8") as stream:
        raw = list(csv.DictReader(stream))
    if len(raw) < 20:
        raise ValueError("拖动记录不足 20 条，不能提取轨迹极值")
    rows = []
    for index, row in enumerate(raw):
        if (int(row["sample_index"]) != index
                or row["control_mode"] != "TEACHING_MODE"
                or row["arm_status"] != "NORMAL"
                or row["teach_status"] != "START_RECORDING"):
            raise ValueError(f"第 {index} 条不是连续的正常拖动反馈")
        xyz = [float(row[f"{axis}_m"]) for axis in "xyz"]
        joints = [float(row[f"joint_{joint}_rad"]) for joint in range(1, 8)]
        if not all(math.isfinite(value) for value in xyz+joints):
            raise ValueError(f"第 {index} 条含无效坐标或关节角")
        rows.append({"level": "new_hand_guided_sweep", "source": str(source),
                     "source_index": index, "time_utc": row["captured_at_utc"],
                     **{f"{axis}_m": xyz[i] for i, axis in enumerate("xyz")},
                     "joint_rad": json.dumps(joints)})
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = Path(directory) / f"nero_drag_map_{stamp}.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("level", "source", "source_index",
                                                   "time_utc", "x_m", "y_m", "z_m",
                                                   "joint_rad"))
        writer.writeheader()
        writer.writerows(rows)
    metadata = {"schema_version": 1, "source_recording_csv": str(source),
                "source_recording_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "levels": {"new_hand_guided_sweep": {"samples": len(rows)}},
                "interpretation": "仅是拖动时经过的法兰位置，不证明包围盒或立方体内部安全。"}
    with path.with_suffix(".json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path
