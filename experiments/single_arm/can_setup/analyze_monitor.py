#!/usr/bin/env python3
"""只读汇总状态跳变及附近 CAN 控制帧；不连接机械臂。"""

import argparse
import csv
from datetime import datetime
from pathlib import Path


def timestamp(row):
    return datetime.fromisoformat(row["time_utc"])


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def state_key(row):
    enabled = tuple(row.get(f"joint_{index}_enabled", "") for index in range(1, 8))
    if not row.get("arm_status") or not all(enabled):
        return None
    return row["arm_status"], enabled


def transitions(rows):
    """跳过启动时尚未收到新鲜反馈的空行。"""
    previous = None
    changes = []
    for row in rows:
        current = state_key(row)
        if current is None:
            continue
        if previous is not None and current != previous[1]:
            changes.append((previous[0], row))
        previous = (row, current)
    return changes


def voltage_summary(rows):
    """仅统计 CSV 中已有的驱动电压反馈，不推测适配器输出。"""
    observations = []
    for row in rows:
        for index in range(1, 8):
            raw = row.get(f"joint_{index}_voltage_v", "")
            if raw:
                observations.append((float(raw), row["time_utc"], index))
    if not observations:
        return None
    return min(observations), max(observations)


def largest_sample_gap(rows):
    """找出状态反馈之间最长的采样空档。"""
    if len(rows) < 2:
        return None
    pairs = ((timestamp(before), timestamp(after))
             for before, after in zip(rows, rows[1:]))
    start, end = max(pairs, key=lambda pair: pair[1] - pair[0])
    return (end - start).total_seconds(), start.isoformat(), end.isoformat()


def surrounding_host_samples(rows, when, window_seconds):
    """取状态跳变前后的主机记录；过远的样本不用于解释该事件。"""
    before = None
    after = None
    for row in rows:
        at = timestamp(row)
        if at <= when and (before is None or at > timestamp(before)):
            before = row
        if at >= when and (after is None or at < timestamp(after)):
            after = row
    if before and (when - timestamp(before)).total_seconds() > window_seconds:
        before = None
    if after and (timestamp(after) - when).total_seconds() > window_seconds:
        after = None
    return before, after


def print_host_sample(label, row):
    if row is None:
        print(f"  {label}：该时间附近无主机记录。")
        return
    print(f"  {label}：{row['time_utc']}；can0={row.get('can_present', '')}/"
          f"{row.get('can_operstate', '')}；RX 丢包={row.get('rx_dropped', '')}；"
          f"RX/TX 错误={row.get('rx_errors', '')}/{row.get('tx_errors', '')}；"
          f"工作区进程={row.get('project_processes', '') or '无'}")


def summarize(state_path, frame_path, window_seconds=5.0, host_path=None):
    states = read_csv(state_path)
    frames = read_csv(frame_path)
    hosts = read_csv(host_path) if host_path else []
    print(f"状态采样：{len(states)} 条；控制帧：{len(frames)} 条。")
    if host_path:
        print(f"主机采样：{len(hosts)} 条。")
    if not states:
        print("状态文件为空，不能分析失能。")
        return
    valid = [row for row in states if state_key(row) is not None]
    if not valid:
        print("没有完整的整机状态与七轴使能反馈，不能分析失能。")
        return
    print("有效区间：", valid[0]["time_utc"], "至", valid[-1]["time_utc"])
    voltage = voltage_summary(valid)
    if voltage:
        low, high = voltage
        print(f"采样到的驱动电压范围：{low[0]:.1f}～{high[0]:.1f} V；"
              f"最低值在 {low[1]} 的关节 {low[2]}。")
    gap = largest_sample_gap(valid)
    if gap:
        print(f"最长状态采样间隔：{gap[0]:.3f} 秒（{gap[1]} 至 {gap[2]}）。")
    changes = transitions(states)
    if not changes:
        print("有效采样中未见整机状态或七轴使能位跳变。")
    for before, after in changes:
        when = timestamp(after)
        nearby = [frame for frame in frames
                  if abs((timestamp(frame) - when).total_seconds()) <= window_seconds]
        print("状态跳变：", before["time_utc"], before["arm_status"],
              "使能", state_key(before)[1], "→", after["time_utc"],
              after["arm_status"], "使能", state_key(after)[1])
        if nearby:
            for frame in nearby:
                print("  附近控制帧：", frame["time_utc"], frame["can_id"],
                      frame["data_hex"], frame["event"])
        else:
            print(f"  前后 {window_seconds:g} 秒的记录中未见所监听控制帧。")
        if host_path:
            host_before, host_after = surrounding_host_samples(hosts, when, window_seconds)
            print_host_sample("跳变前主机", host_before)
            print_host_sample("跳变后主机", host_after)
    print("解释边界：驱动电压反馈不是适配器电压测量，采样间隔内的供电瞬变可能漏掉；"
          "未见帧不能排除未开启本机回环的发送端、漏帧、外部控制器或供电事件。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state_csv", type=Path)
    parser.add_argument("control_frames_csv", type=Path)
    parser.add_argument("--host-csv", type=Path, help="可选：monitor_host 生成的主机侧 CSV")
    parser.add_argument("--window-seconds", type=float, default=5.0)
    args = parser.parse_args()
    if not 0 < args.window_seconds <= 60:
        parser.error("--window-seconds 必须大于 0 且不超过 60")
    summarize(args.state_csv, args.control_frames_csv, args.window_seconds, args.host_csv)


if __name__ == "__main__":
    main()
