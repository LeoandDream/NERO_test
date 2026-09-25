"""把完整示教反馈转成带时间、连续一阶导数的关节目标。纯离线计算。"""

import bisect
import csv
import hashlib
import math
from pathlib import Path

from experiments.single_arm.teaching.return_session import load_recording
from experiments.single_arm.teaching.teach_session import validate_joints


DEFAULT_SOURCE = Path(
    "experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv"
)
EXPECTED_SOURCE_SHA256 = "90eadd27a19b86e7b32305c3e27b503fd6d5d073a0f834181b10773d8548dba5"
FILTER_WEIGHTS = (1, 4, 6, 4, 1)
MAX_SOURCE_SMOOTH_DEVIATION_RAD = 0.01
MAX_SOURCE_GAP_S = 0.15


def read_source(path, config):
    """验证同名元数据、全部 CSV 行和时间连续性；保留停稳补录。"""
    path = Path(path)
    _, metadata, original, recorded = load_recording(path, config, allow_completed=True)
    if not metadata.get("return_completed") or metadata.get("stop_reason") != "time_limit":
        raise ValueError("连续回放只接受完成原路回位的完整限时记录")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != len(recorded):
        raise ValueError("示教源记录行数发生变化")
    if rows[0]["teach_status"] != "START_RECORDING":
        raise ValueError("示教源缺少拖动起点")
    recorded_count = sum(row["teach_status"] == "START_RECORDING" for row in rows)
    if recorded_count < 2 or any(row["teach_status"] != "DISABLED" for row in rows[recorded_count:]):
        raise ValueError("示教与退出后停稳样本顺序无效")
    raw_times = [float(row["elapsed_s"]) for row in rows]
    times = [value - raw_times[0] for value in raw_times]
    gaps = [b - a for a, b in zip(times, times[1:])]
    if not all(0 < gap <= MAX_SOURCE_GAP_S for gap in gaps):
        raise ValueError("示教源时间间隙或顺序异常")
    if not 19.0 <= times[recorded_count - 1] <= 21.0:
        raise ValueError("拖动源时长不是约 20 秒")
    if math.dist(recorded[0], original) > 0.01:
        raise ValueError("源首点与示教前实际起点不一致")
    return {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "metadata": metadata,
        "original": original,
        "times": times,
        "joints": recorded,
        "teaching_rows": recorded_count,
        "flange_xyz": [[float(row[key]) for key in ("x_m", "y_m", "z_m")] for row in rows],
    }


def _smoothed(joints):
    radius = len(FILTER_WEIGHTS) // 2
    weight_sum = sum(FILTER_WEIGHTS)
    result = []
    for index in range(len(joints)):
        result.append([
            sum(weight * joints[min(max(index + offset - radius, 0), len(joints) - 1)][joint]
                for offset, weight in enumerate(FILTER_WEIGHTS)) / weight_sum
            for joint in range(7)
        ])
    result[0] = list(joints[0])
    result[-1] = list(joints[-1])
    return result


class JointCurve:
    """五点平滑后按真实采样时间构造分段三次 Hermite 曲线。"""

    def __init__(self, times, joints):
        self.times = list(times)
        self.raw = [list(point) for point in joints]
        self.points = _smoothed(self.raw)
        self.tangents = [[0.0] * 7]
        for index in range(1, len(times) - 1):
            width = times[index + 1] - times[index - 1]
            self.tangents.append([
                (self.points[index + 1][joint] - self.points[index - 1][joint]) / width
                for joint in range(7)
            ])
        self.tangents.append([0.0] * 7)
        self.max_source_deviation_rad = max(
            max(abs(a - b) for a, b in zip(raw, smooth))
            for raw, smooth in zip(self.raw, self.points)
        )

    def evaluate(self, second):
        """返回 q、dq/dt、d²q/dt²；t 为源记录秒数。"""
        if not 0 <= second <= self.times[-1]:
            raise ValueError("请求的源时间超出记录范围")
        index = min(bisect.bisect_right(self.times, second) - 1, len(self.times) - 2)
        width = self.times[index + 1] - self.times[index]
        u = (second - self.times[index]) / width
        u2, u3 = u * u, u * u * u
        h00, h10, h01, h11 = 2*u3-3*u2+1, u3-2*u2+u, -2*u3+3*u2, u3-u2
        d00, d10, d01, d11 = 6*u2-6*u, 3*u2-4*u+1, -6*u2+6*u, 3*u2-2*u
        a00, a10, a01, a11 = 12*u-6, 6*u-4, -12*u+6, 6*u-2
        q, velocity, acceleration = [], [], []
        for joint in range(7):
            p0, p1 = self.points[index][joint], self.points[index + 1][joint]
            v0, v1 = self.tangents[index][joint], self.tangents[index + 1][joint]
            q.append(h00*p0 + h10*width*v0 + h01*p1 + h11*width*v1)
            velocity.append((d00*p0 + d10*width*v0 + d01*p1 + d11*width*v1)/width)
            acceleration.append((a00*p0 + a10*width*v0 + a01*p1 + a11*width*v1)/(width*width))
        return q, velocity, acceleration


def make_plan(source, config, *, until_source_s=None, frequency_hz=20,
              max_velocity_rad_s=0.20, max_acceleration_rad_s2=0.40,
              min_time_scale=6.0, return_to_start=True,
              turnaround_hold_s=0.0):
    """从全部或前缀源样本生成定时目标；完整方案必须包含末尾停稳样本。"""
    if frequency_hz not in (10, 20):
        raise ValueError("首次流式试验只允许 10 或 20 Hz")
    if not 0 < max_velocity_rad_s <= 0.20 or not 0 < max_acceleration_rad_s2 <= 0.40:
        raise ValueError("目标速度/加速度超过首次试验上限")
    if min_time_scale < 6 or min_time_scale > 30:
        raise ValueError("时间伸缩下限必须在 6～30 倍")
    if (not 0 <= turnaround_hold_s <= 5
            or abs(turnaround_hold_s * frequency_hz
                   - round(turnaround_hold_s * frequency_hz)) > 1e-9):
        raise ValueError("转向停留须在 0～5 秒，且为发布周期的整数倍")
    if turnaround_hold_s and (until_source_s is None or not return_to_start):
        raise ValueError("转向停留仅用于沿源前缀到点保持并原路返回")
    times, joints = source["times"], source["joints"]
    if until_source_s is not None:
        if not 1.7 <= until_source_s < times[-1] - 0.1:
            raise ValueError("前缀源时长须在 1.7 秒与完整源时长之间")
        end = bisect.bisect_right(times, until_source_s)
        if end < 3:
            raise ValueError("前缀样本不足")
        times, joints = times[:end], joints[:end]
    curve = JointCurve(times, joints)
    if curve.max_source_deviation_rad > MAX_SOURCE_SMOOTH_DEVIATION_RAD:
        raise ValueError("平滑轨迹偏离原始关节反馈过大")
    source_v = source_a = 0.0
    max_path_deviation = curve.max_source_deviation_rad
    # 每段细采样导数，包含转折和端点；时间伸缩将速度按 1/S、加速度按 1/S² 缩放。
    for first, second in zip(times, times[1:]):
        for part in range(5):
            fraction = part / 4
            target, velocity, acceleration = curve.evaluate(first + (second-first)*fraction)
            source_v = max(source_v, *(abs(value) for value in velocity))
            source_a = max(source_a, *(abs(value) for value in acceleration))
            left = bisect.bisect_right(times, first) - 1
            raw_linear = [
                a + fraction * (b-a)
                for a, b in zip(joints[left], joints[left + 1])
            ]
            max_path_deviation = max(max_path_deviation, max(
                abs(a-b) for a, b in zip(target, raw_linear)
            ))
    if max_path_deviation > MAX_SOURCE_SMOOTH_DEVIATION_RAD:
        raise ValueError("Hermite 轨迹偏离原记录折线超过 0.01 rad")
    scale = math.ceil(max(
        min_time_scale,
        source_v / max_velocity_rad_s,
        math.sqrt(source_a / max_acceleration_rad_s2),
    ) * 10) / 10
    if scale > 30:
        raise ValueError("源轨迹要求时间伸缩超过 30 倍；需重新评估数据或控制方案")
    duration = times[-1] * scale
    period = 1 / frequency_hz
    out_times = [index * period for index in range(math.floor(duration / period) + 1)]
    if duration - out_times[-1] > 1e-9:
        out_times.append(duration)
    forward = []
    for t in out_times:
        q, _, _ = curve.evaluate(min(t / scale, times[-1]))
        forward.append({"t_s": t, "source_t_s": min(t / scale, times[-1]),
                        "phase": "forward", "joint_rad": validate_joints(
                            q, config["zero_exclusion_radius_rad"]
                        )})
    waypoints = list(forward)
    for index in range(1, round(turnaround_hold_s * frequency_hz) + 1):
        waypoints.append({"t_s": duration + index * period,
                          "source_t_s": times[-1], "phase": "hold",
                          "joint_rad": list(forward[-1]["joint_rad"])})
    if return_to_start:
        for item in reversed(forward[:-1]):
            waypoints.append({"t_s": 2*duration + turnaround_hold_s - item["t_s"],
                              "source_t_s": item["source_t_s"], "phase": "return",
                              "joint_rad": list(item["joint_rad"])})
    return {
        "schema_version": 1,
        "recording": source["path"],
        "source_sha256": source["sha256"],
        "source_total_rows": len(source["joints"]),
        "source_teaching_rows": source["teaching_rows"],
        "source_used_rows": len(joints),
        "source_full_duration_s": source["times"][-1],
        "source_used_duration_s": times[-1],
        "until_source_s": until_source_s,
        "frequency_hz": frequency_hz,
        "max_velocity_rad_s": max_velocity_rad_s,
        "max_acceleration_rad_s2": max_acceleration_rad_s2,
        "min_time_scale": min_time_scale,
        "time_scale": scale,
        "turnaround_hold_s": turnaround_hold_s,
        "max_source_velocity_rad_s": source_v,
        "max_source_acceleration_rad_s2": source_a,
        "max_smoothing_deviation_rad": curve.max_source_deviation_rad,
        "max_path_deviation_rad": max_path_deviation,
        "return_to_start": return_to_start,
        "forward_duration_s": duration,
        "total_duration_s": waypoints[-1]["t_s"],
        "waypoints": waypoints,
    }
