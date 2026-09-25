#!/usr/bin/env python3
"""汇总历史实测点和受控连接路径，生成可追溯 CSV/JSON/SVG；不连接 CAN。"""

from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from experiments.single_arm.continuous_replay.trajectory import DEFAULT_SOURCE


ROOT = Path("experiments/single_arm/reachability/data/maps")
OLD_POINT_RUNS = Path("experiments/single_arm/control_interfaces/data/runs")
CONTACT_RUN = OLD_POINT_RUNS / "nero_point_demo_leg7_20260925T142300Z.json"


def historic_rows():
    with DEFAULT_SOURCE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        yield {
            "level": "historical_hand_guided",
            "source": str(DEFAULT_SOURCE),
            "source_index": row["sample_index"],
            "time_utc": row["captured_at_utc"],
            "x_m": float(row["x_m"]), "y_m": float(row["y_m"]),
            "z_m": float(row["z_m"]),
            "joint_rad": [float(row[f"joint_{number}_rad"]) for number in range(1, 8)],
        }


def old_controlled_rows():
    for path in sorted(OLD_POINT_RUNS.glob("nero_point_demo_leg*_20260925T*.json")):
        if path == CONTACT_RUN:
            continue
        report = json.loads(path.read_text(encoding="utf-8"))
        if not report.get("completed") or not 1 <= report.get("leg", 0) <= 6:
            continue
        for index, sample in enumerate(report.get("feedback_samples", [])):
            pose = sample["flange_pose"]
            yield {
                "level": "historical_controlled_path_sample",
                "source": str(path),
                "source_index": index,
                "time_utc": report.get("command_sent_at_utc", ""),
                "x_m": float(pose[0]), "y_m": float(pose[1]),
                "z_m": float(pose[2]),
                "joint_rad": sample["joint_rad"],
            }


def new_probe_rows():
    analyses = sorted(Path("experiments/single_arm/reachability/data/analysis").glob(
        "nero_probe_series_*.json"))
    accepted_sources = set()
    if analyses:
        analysis = json.loads(analyses[-1].read_text(encoding="utf-8"))
        accepted_sources = {
            item["report"] for item in analysis.get("segments", [])
            if item.get("accepted")
        }
    for path in sorted(Path("experiments/single_arm/reachability/data/runs").glob("nero_probe_*.json")):
        if str(path) not in accepted_sources:
            continue
        report = json.loads(path.read_text(encoding="utf-8"))
        if not report.get("completed"):
            continue
        for index, sample in enumerate(report.get("feedback_samples", [])):
            pose = sample["flange_pose"]
            yield {
                "level": "new_controlled_path_sample",
                "source": str(path), "source_index": index,
                "time_utc": report.get("command_sent_at_utc", ""),
                "x_m": float(pose[0]), "y_m": float(pose[1]),
                "z_m": float(pose[2]), "joint_rad": sample["joint_rad"],
            }


def new_continuous_rows():
    """只收录独立验收通过的完整回放反馈，不把离线目标当实测位置。"""
    run_dir = Path("experiments/single_arm/continuous_replay/data/runs")
    for path in sorted(run_dir.glob("nero_continuous_*.json")):
        if path.stem.endswith("_analysis"):
            continue
        analysis_path = path.with_name(path.stem + "_analysis.json")
        if not analysis_path.exists():
            continue
        report = json.loads(path.read_text(encoding="utf-8"))
        analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
        if not (report.get("completed") and analysis.get("acceptance_pass")
                and analysis.get("full_source_used")
                and analysis.get("source_sha256") == report.get("source_sha256")):
            continue
        with Path(report["samples_csv"]).open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                pose = json.loads(row["flange_feedback_xyz_m"])
                yield {
                    "level": "new_full_continuous_path_sample",
                    "source": str(path), "source_index": int(row["index"]),
                    "time_utc": row["command_returned_utc"],
                    "x_m": float(pose[0]), "y_m": float(pose[1]),
                    "z_m": float(pose[2]),
                    "joint_rad": json.loads(row["joint_feedback_rad"]),
                }


def summary(rows):
    levels = {}
    for level in sorted(set(row["level"] for row in rows)):
        group = [row for row in rows if row["level"] == level]
        levels[level] = {
            "samples": len(group),
            "sources": sorted(set(row["source"] for row in group)),
            "coordinate_ranges_m": {
                axis: [min(row[f"{axis}_m"] for row in group),
                       max(row[f"{axis}_m"] for row in group)]
                for axis in "xyz"
            },
            "joint_ranges_rad": [
                [min(row["joint_rad"][index] for row in group),
                 max(row["joint_rad"][index] for row in group)]
                for index in range(7)
            ],
        }
    contact = json.loads(CONTACT_RUN.read_text(encoding="utf-8"))
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "installation": "left mount; current table, base, tool and cable layout only",
        "source_sha256": hashlib.sha256(DEFAULT_SOURCE.read_bytes()).hexdigest(),
        "levels": levels,
        "contact_event": {
            "source": str(CONTACT_RUN),
            "user_observation": "2026-09-25 move_p X-negative leg7: flange contacted table during motion",
            "last_logged_flange_xyz_m": contact["feedback_samples"][-1]["flange_pose"][:3],
            "last_logged_point_is_not_exact_contact_location": True,
            "result": contact.get("error"),
        },
        "verified_connections": [
            {"source": source, "level": level, "sample_count": count}
            for (level, source), count in sorted(Counter(
                (row["level"], row["source"]) for row in rows
                if row["level"] in ("new_controlled_path_sample",
                                    "new_full_continuous_path_sample")
            ).items())
        ],
        "interpretation": "Separate observations by level. Verified connections refer only to recorded paths. Axis-aligned bounding boxes are summaries, not collision-free workspace.",
    }


def write_svg(path, rows):
    # 顶视 X/Y 投影，仅画有来源的实测样本，不在样本之间填充可达区域。
    width, height, margin = 780, 600, 65
    xs = [row["x_m"] for row in rows]
    ys = [row["y_m"] for row in rows]
    xmin, xmax = min(xs)-0.015, max(xs)+0.015
    ymin, ymax = min(ys)-0.015, max(ys)+0.015

    def pos(row):
        x = margin + (row["x_m"]-xmin)/(xmax-xmin)*(width-2*margin)
        y = height-margin-(row["y_m"]-ymin)/(ymax-ymin)*(height-2*margin)
        return x, y

    colors = {"historical_hand_guided": "#9ca3af",
              "historical_controlled_path_sample": "#1d4ed8",
              "new_controlled_path_sample": "#059669",
              "new_full_continuous_path_sample": "#ea580c"}
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="65" y="32" font-family="sans-serif" font-size="18">Nero 法兰实测 X/Y 投影（左侧装）</text>',
             f'<path d="M {margin} {height-margin} H {width-margin} M {margin} {height-margin} V {margin}" stroke="#333" fill="none"/>',
             f'<text x="{width/2}" y="{height-15}" font-family="sans-serif" font-size="13">基座 X (m): {xmin:.2f} 至 {xmax:.2f}</text>',
             f'<text x="8" y="{height/2}" font-family="sans-serif" font-size="13">Y (m)</text>']
    for row in rows:
        x, y = pos(row)
        radius = 1.2 if row["level"] == "historical_hand_guided" else 2.1
        parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{colors[row["level"]]}"/>')
    for index, (label, color) in enumerate((
            ("历史手动示教样本", colors["historical_hand_guided"]),
            ("旧受控路径反馈", colors["historical_controlled_path_sample"]),
            ("本次新探测反馈", colors["new_controlled_path_sample"]),
            ("本次完整连续回放", colors["new_full_continuous_path_sample"]))):
        y = 54 + 18*index
        parts.append(f'<circle cx="550" cy="{y}" r="4" fill="{color}"/>')
        parts.append(f'<text x="565" y="{y+4}" font-family="sans-serif" font-size="12">{label}</text>')
    parts.append('<text x="65" y="580" font-family="sans-serif" font-size="11">点仅表示采样位置；未填充部分和点间路径均不代表安全可达。</text>')
    parts.append('</svg>')
    path.write_text("\n".join(parts)+"\n", encoding="utf-8")


def main():
    rows = [*historic_rows(), *old_controlled_rows(),
            *new_probe_rows(), *new_continuous_rows()]
    if not rows:
        raise RuntimeError("没有可汇总的原始样本")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    root = ROOT / f"nero_workspace_{stamp}"
    root.parent.mkdir(parents=True, exist_ok=True)
    with root.with_suffix(".csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("level", "source", "source_index", "time_utc",
                                                     "x_m", "y_m", "z_m", "joint_rad"))
        writer.writeheader()
        writer.writerows(rows)
    with root.with_suffix(".json").open("x", encoding="utf-8") as stream:
        json.dump(summary(rows), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    write_svg(root.with_suffix(".svg"), rows)
    print("实测空间目录：", root.with_suffix(".json"))
    print("样本与来源：", root.with_suffix(".csv"))
    print("X/Y 投影图：", root.with_suffix(".svg"))
    print("本次新受控探测样本数：", sum(row["level"] == "new_controlled_path_sample" for row in rows))
    print("包围盒仅供描述；不证明内部点或连线安全。")


if __name__ == "__main__":
    main()
