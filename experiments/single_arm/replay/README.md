# 回放 2026-09-23 的 S1 示教

[回放脚本](replay_session.py)默认读取[已完成回位的五秒示教记录](../teaching/data/recordings/nero_session_20260923T145952Z.csv)。该记录有 280 条关节反馈，其中 35 条是退出拖动后的停稳补录。脚本沿关节记录正向回放，再沿同一条小步路线反向返回**本轮开始时的实际关节位置**；不做末端逆运动学求解。

先在 `nero-py310` 环境运行只读规划：

```bash
python -m experiments.single_arm.replay.replay_session --cycles 3
```

2026-09-24 在 S1 的只读规划结果为：每轮正向 22 小步、往返 44 小步，速度 5%；法兰中心 FK 范围约 `x=0.3787～0.3823 m`、`y=−0.0141～0.0450 m`、`z=0.5595～0.5747 m`。这些是路径上的法兰计算范围，不能排除其他连杆或现场物体的碰撞。默认 `flange_workspace_m=null`，法兰空间边界尚未启用。

现场确认从 S1 到记录末端再返回的整条路径清空、底座固定、有人监护后，可先执行一轮：

```bash
python -m experiments.single_arm.replay.replay_session --cycles 1 --run
```

确认第一轮成功回到 S1 后，再执行后两轮：

```bash
python -m experiments.single_arm.replay.replay_session --cycles 2 --run
```

脚本开始前有 5 秒倒计时。每轮重新核对状态、驱动、当前位置和路线；每个小步核对实际关节反馈。回放默认速度 5%，也可在只读规划时指定 `--speed-percent 10`，经现场核对后将相同参数用于 `--run`。不会自动提速。每步到位误差要求 `≤0.005 rad`；基础等待 5 秒，仅在关节反馈持续接近目标时继续等待，最多 15 秒。整轮使用配置的 600 秒时限；到期后在**当前小步确认到位**时暂停，报告为未完成，机械臂停留在中途目标，必须现场核对后再规划恢复。报告中的 `last_sent_step` 是 SDK 调用前记录的发送尝试；新版另存 `last_sdk_publish_completed_step`，表示 SDK 调用已返回，但仍不能代替 CAN 原始帧或控制器到位反馈。

2026-09-25 修订：若单步超时，连续约 0.8 秒确认整机正常、七轴反馈仍在上一目标附近且几乎静止；控制器仍报告正常到位状态时，只重发**同一个目标一次**。若仍未动，或控制器报告到位失败，则暂停回放且不主动请求电子急停。状态异常、位置漂移、路径越界或反馈缺失仍走故障急停。新报告字段 `stationary_target_retries`、`pause_reason` 和 `emergency_stop_requested` 用于区分重发、暂停与脚本主动急停。执行时会生成同名 `.can.csv`，旁路记录 CAN `0x150/0x151/0x155/0x156/0x157/0x170/0x2A1` 帧；JSON 的 `recent_command_events` 记录最后八次发送尝试与 SDK 返回时刻。用下列**只读**命令对齐失败小步与总线帧：

```bash
python -m experiments.single_arm.replay.inspect_can_trace experiments/single_arm/replay/data/replays/本次报告.json
```

四组关节目标帧都出现在旁路记录中，仍不能单独证明控制器接收或执行。修订通过离线测试，并在下述 2026-09-25 长回放中完成一轮实机验证。

路径偏离、关节在上一目标之外停住、控制器故障或收到 `Ctrl+C` 时，已发送运动指令的情况下会调用 SDK 电子急停，停止后续轮次。**该电子急停是阻尼停止，抬起的关节可能下落，不能视为位置保持或自动回到起点。**每轮返回起点的最大单关节误差须不超过 `0.005 rad`。报告记录最后发送的小步目标、最后反馈关节角、已确认到位的小步号及暂停或错误原因，保存在 `experiments/single_arm/replay/data/replays/`。

2026-09-24 的 20 秒完整示教记录可用 `--recording experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv` 指定。只读规划得到 322 小步去程和 322 小步回程；法兰沿途范围为 `x=0.2429～0.4406 m`、`y=−0.1333～0.0058 m`、`z=0.4343～0.6108 m`。首次实机回放在 558/644 步后中止并触发阻尼急停；失败报告显示该轮恰好运行约 120 秒。旧报告没有急停前的目标和关节反馈，无法断言是整轮时限还是单步运动异常。

随后用旧版改进代码以 10% 速度复测长记录，**在 422/644 步后再次中止，脚本主动发送阻尼急停**；[本次报告](data/replays/nero_replay_20260924T131725Z.json)显示第 423 步目标下发后，关节反馈几乎保持在第 422 步，5 秒后 J4 仍相差约 `0.024 rad`。当时远未到 600 秒整轮时限，因此至少本次不是“运动太慢碰到整轮时限”。现场确认停稳且无接触或异响；只读状态为 `EMERGENCY_STOP`，随后 5 秒检查稳定，但急停后姿态已明显改变。**不要直接重复长记录回放或自动回 S1。**复位和后续实验见[电子急停与实验恢复](../../../docs/emergency_stop_recovery.md)。

此前长回放两次在不同位置中止，仍未查明旧版第 423 步未执行目标的原因。新版加入停在上一点时的重发与暂停，以及 CAN 原始帧旁路记录；2026-09-25 在 S1 完成了下述一轮实机回放。再次运行前仍须现场确认全路线清空并全程监护，先只读检查 10% 方案：

```bash
python -m experiments.single_arm.replay.replay_session --recording experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv --cycles 1 --speed-percent 10
```

现场确认状态、全路线范围和规划后，用同样参数加 `--run` 才会运动。每次先做 1 轮；若报告显示 `pause_reason=cycle_time_limit` 或 `stationary_waypoint_timeout`，机械臂可能仍在中途，不能当作已回 S1；若出现 `EMERGENCY_STOP`，按[恢复流程](../../../docs/emergency_stop_recovery.md)处理。不要在暂停后直接重启原回放命令。

2026-09-25 的[20 秒记录完整回放报告](data/replays/nero_replay_20260925T152748Z.json)：10% 速度完成 644/644 小步，回到本轮实际起点，最大单关节误差 `0.000209 rad`，无重发、无脚本主动急停；事后独立只读检查为 `NORMAL`、七轴使能且仍接近 S1。旁路 CAN 文件四组目标帧各有 644 条。整轮约 132.4 秒，平均约 4.86 步/秒；逐点 `move_j` 加每步等待反馈会造成步间顿挫，详见[实验报告](report.md#2026-09-25-长记录完整回放)。

此前[20 秒中断记录](../teaching/data/recordings/nero_session_20260923T151241Z.csv)缺少停稳补录，不允许用于回放。

## 2026-09-24 三轮测试结果

现场确认底座固定、整条路径清空并监护后，先执行 1 轮，再独立核对 S1、控制器和七个驱动，随后执行 2 轮。三轮均完成正向 22 小步和反向 22 小步，按各轮开始时的实际位置计算，最大单关节返回误差依次为 `0.000140 rad`、`0.000140 rad`、`0.000052 rad`。执行报告分别为[第一轮](data/replays/nero_replay_20260924T085003Z.json)和[后两轮](data/replays/nero_replay_20260924T085053Z.json)。

最后独立只读检查显示机械臂仍接近 S1，控制器 `CAN_CTRL`、状态 `NORMAL`、示教 `DISABLED`、错误码 `0x0000`，七个驱动均使能。这验证了**这份五秒示教记录在当前现场条件下连续三次正向回放并回到起点**；不代表其他记录或其他现场布置同样安全。
