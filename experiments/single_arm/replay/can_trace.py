"""回放期间旁路记录 CAN 指令和整机状态；只接收，不发送帧。"""

import csv
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import can


TRACE_IDS = (0x150, 0x151, 0x155, 0x156, 0x157, 0x170, 0x2A1)


class ReplayCanTrace:
    """把实际总线帧即时写入 CSV，保留目标发送与状态反馈的时序。"""

    def __init__(self, channel, path, frame_ids=TRACE_IDS, flush_every=1):
        self.channel = channel
        self.path = Path(path)
        self.frame_ids = tuple(frame_ids)
        if not self.frame_ids or flush_every < 1:
            raise ValueError("CAN 旁路记录筛选器或刷新间隔无效")
        self.flush_every = flush_every
        self.bus = None
        self.stream = None
        self.notifier = None
        self.count = 0
        self.id_counts = Counter()

    def start(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        filters = [
            {"can_id": frame_id, "can_mask": 0x7FF, "extended": False}
            for frame_id in self.frame_ids
        ]
        try:
            self.bus = can.Bus(
                interface="socketcan", channel=self.channel,
                receive_own_messages=False, can_filters=filters,
            )
            self.stream = self.path.open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(self.stream, fieldnames=(
                "time_utc", "can_id", "data_hex",
            ))
            writer.writeheader()
            self.stream.flush()

            def record(message):
                if (message.is_error_frame or message.is_remote_frame
                        or message.is_extended_id
                        or message.arbitration_id not in self.frame_ids):
                    return
                writer.writerow({
                    "time_utc": datetime.fromtimestamp(
                        message.timestamp, timezone.utc
                    ).isoformat(timespec="microseconds"),
                    "can_id": f"0x{message.arbitration_id:03X}",
                    "data_hex": bytes(message.data).hex().upper(),
                })
                self.count += 1
                self.id_counts[message.arbitration_id] += 1
                if self.count % self.flush_every == 0:
                    self.stream.flush()

            self.notifier = can.Notifier(self.bus, [record], timeout=0.2)
        except Exception:
            self.stop()
            raise

    def check(self):
        if self.notifier is None or self.notifier.exception is not None:
            raise RuntimeError("回放 CAN 旁路监听已停止；禁止继续发送目标")

    def stop(self):
        if self.notifier is not None:
            self.notifier.stop()
            self.notifier = None
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
