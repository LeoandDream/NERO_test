#!/usr/bin/env python3
"""只读记录 CAN 网卡计数器及工作区进程，辅助定位突然失能。"""

import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import time


OUTPUT_DIR = Path("experiments/single_arm/can_setup/data/monitoring")
NET_ROOT = Path("/sys/class/net")
PROC_ROOT = Path("/proc")
COUNTERS = ("rx_packets", "rx_errors", "rx_dropped", "tx_packets", "tx_errors", "tx_dropped")


def project_processes():
    """只记录工作区模块名和 PID，不保存完整命令行或环境变量。"""
    found = []
    for entry in PROC_ROOT.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            parts = (entry / "cmdline").read_bytes().split(b"\0")
        except (OSError, PermissionError):
            continue  # 进程可能恰在扫描期间退出。
        for index, part in enumerate(parts[:-1]):
            if part == b"-m" and parts[index + 1].startswith(b"experiments.single_arm."):
                module = parts[index + 1].decode("utf-8", errors="replace")
                found.append(f"{entry.name}:{module}")
                break
    return ";".join(sorted(found))


def sample(channel):
    path = NET_ROOT / channel
    row = {"can_present": path.exists(), "project_processes": project_processes()}
    if path.exists():
        try:
            row["can_operstate"] = (path / "operstate").read_text().strip()
            for name in COUNTERS:
                row[name] = int((path / "statistics" / name).read_text())
        except OSError:
            # USB-CAN 恰在读取多个文件时消失：保留本次空字段并继续采样。
            row = {"can_present": False, "project_processes": row["project_processes"]}
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--interval", type=float, default=2.0)
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not math.isfinite(args.interval)
            or not 1 <= args.duration <= 43200 or not 0.5 <= args.interval <= 60):
        parser.error("duration 必须为 1～43200 秒，interval 必须为 0.5～60 秒")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = OUTPUT_DIR / f"nero_host_{session}.csv"
    fields = ("time_utc", "elapsed_s", "can_present", "can_operstate", *COUNTERS,
              "project_processes")
    started = time.monotonic()
    try:
        with output.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            stream.flush()
            print(f"只读主机记录：{output}；按 Ctrl+C 提前结束。", flush=True)
            next_sample = started
            while time.monotonic() - started < args.duration:
                time.sleep(max(0, next_sample - time.monotonic()))
                next_sample += args.interval
                writer.writerow({"time_utc": datetime.now(timezone.utc).isoformat(),
                                 "elapsed_s": round(time.monotonic() - started, 3),
                                 **sample(args.channel)})
                stream.flush()
    except KeyboardInterrupt:
        print("收到 Ctrl+C，结束只读主机记录。", flush=True)
    print(f"主机记录文件：{output}", flush=True)


if __name__ == "__main__":
    main()
