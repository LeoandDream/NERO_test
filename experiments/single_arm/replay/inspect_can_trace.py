#!/usr/bin/env python3
"""只读汇总回放最后几次目标发送前后的原始 CAN 帧。"""

import argparse
import csv
from datetime import datetime, timedelta
import json
from pathlib import Path


JOINT_FRAME_IDS = ("0x155", "0x156", "0x157", "0x170")


def inspect(report_path):
    report = json.loads(report_path.read_text(encoding="utf-8"))
    trace_path = Path(report["can_trace_csv"])
    if not trace_path.is_absolute():
        trace_path = Path.cwd() / trace_path
    with trace_path.open(newline="", encoding="utf-8") as stream:
        frames = [
            (datetime.fromisoformat(row["time_utc"]), row["can_id"], row["data_hex"])
            for row in csv.DictReader(stream)
        ]
    for cycle in report["cycles"]:
        print(f"第 {cycle['cycle']} 轮：到位 {cycle['last_reached_step']}/{cycle['total_steps']}，"
              f"发送尝试 {cycle['last_sent_step']}，SDK 返回 {cycle['last_sdk_publish_completed_step']}")
        print("脚本主动急停：", cycle.get("emergency_stop_requested", "旧报告未记录"))
        events = cycle.get("recent_command_events", [])
        for event_index, event in enumerate(events):
            started = datetime.fromisoformat(event["started_at_utc"])
            ended = datetime.fromisoformat(event.get("sdk_returned_at_utc", event["started_at_utc"]))
            # 给内核时间戳留少量余量，同时在下一次发送前截止，避免把相邻
            # 小步的四组目标帧重复统计进同一个事件。
            window_start = started - timedelta(milliseconds=5)
            window_end = max(ended, started) + timedelta(milliseconds=50)
            if event_index + 1 < len(events):
                next_started = datetime.fromisoformat(
                    events[event_index + 1]["started_at_utc"]
                )
                window_end = min(window_end, next_started - timedelta(milliseconds=5))
            selected = [(stamp, frame_id, data) for stamp, frame_id, data in frames
                        if window_start <= stamp <= window_end]
            counts = {frame_id: sum(item[1] == frame_id for item in selected)
                      for frame_id in JOINT_FRAME_IDS}
            stop_frames = [item for item in selected if item[1] == "0x150"]
            print(f"  小步 {event['step']} {event['kind']}，SDK 返回："
                  f"{'sdk_returned_at_utc' in event}；关节目标帧数：{counts}；"
                  f"运动控制帧数：{len(stop_frames)}")
            if event["step"] == cycle["last_sent_step"]:
                for stamp, frame_id, data in selected:
                    if frame_id in JOINT_FRAME_IDS or frame_id == "0x150":
                        print(f"    {stamp.isoformat()} {frame_id} {data}")
    print("上述帧表示旁路监听实际看到的数据；不能单独证明控制器已接收或执行。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="新版回放生成的 JSON 报告")
    args = parser.parse_args()
    inspect(args.report)


if __name__ == "__main__":
    main()
