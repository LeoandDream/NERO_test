# 限时拖动示教实验

自动回位的总时限现在仅在小步到位后暂停；目标未执行、连续反馈确认仍停在上一步时暂停并保存未完成状态，不由软件超时主动请求电子急停。真实状态异常、移动中偏离或反馈缺失仍会触发急停。会话元数据中的 `pause_reason` 和 `emergency_stop_requested` 可区分这两类退出；暂停后不会自动回 S1，须先只读核对当前位置。

在工作区根目录、`nero-py310` 环境中运行。默认配置为 [S1](../config/nero_teach.json)，原 P4 配置单独保留。脚本要求机械臂在 S1 附近；它不会从任意位置自动移动到 S1。

先执行只读检查：

```bash
python -m experiments.single_arm.teaching.teach_session
```

现场确认底座固定、安装方向和关节零点正确，拖动范围清空，准备扶稳机械臂后，再显式运行，例如最长 5 秒：

```bash
python -m experiments.single_arm.teaching.teach_session --run --max-seconds 5
```

会话记录起点、拖动轨迹、退出拖动后的停稳过程，并尝试沿本次记录倒序返回示教前的实际位置。默认最长 120 秒；命令行可设 1–300 秒。CSV 和同名 JSON 写入 `experiments/single_arm/teaching/data/recordings/`。反馈断档、关节或法兰边界、起点不符、未停稳时会拒绝自动回位。若运动途中检测到异常，会发电子急停；不能假设失败后已自动回到起点。

完整记录在退出拖动后被中断时，可先只读复核，现场确认后再加 `--run`：

```bash
python -m experiments.single_arm.teaching.return_session experiments/single_arm/teaching/data/recordings/记录名.csv --config experiments/single_arm/config/nero_teach.json
```

历史过程、失败样本和回位结果见[实验报告](report.md)。20 秒中断记录缺少可用的停稳轨迹，不得用于自动回位。

## 使用已完成的示教记录

2026-09-24 的[20 秒完整记录](data/recordings/nero_session_20260924T101130Z.csv)有 998 条拖动样本和 24 条退出拖动后的停稳样本；[同名 JSON](data/recordings/nero_session_20260924T101130Z.json)标记 `return_completed=true`。CSV 的七个 `joint_*_rad` 列是实际关节轨迹，`x_m/y_m/z_m` 是法兰中心位置，后三个姿态列为 rad。原始 CSV 适合轨迹回放；筛选后的拖动样本适合找人工停留点。

先离线提取示教阶段和停留候选点：

```bash
python -m experiments.single_arm.pose_recording.extract_pose_candidates experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv
```

提取工具会生成本目录下的 `*_teaching_only.csv`，以及位姿采集实验目录中的 `*_held_candidates.csv`。候选点不是已验证的自动运动目标。完整轨迹可交给[回放脚本](../replay/README.md)先只读规划：

本次提取得到两个候选点：[P1、P2 数据](../pose_recording/data/poses/nero_session_20260924T101130Z_held_candidates.csv)。P1 停留约 1.38 秒，位置基本是 S1；P2 的判定时长仅 0.08 秒，只能视作经过点，不能直接作为已确认的停留目标。

```bash
python -m experiments.single_arm.replay.replay_session --recording experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv --cycles 1
```

该 20 秒轨迹比此前验证的五秒记录更大，J4 曾距脚本关节边界约 0.049 rad；需要先查看规划、确认整条连杆活动范围，再考虑实机回放。已完成回位的记录不要交给 `return_session.py` 重新执行。
