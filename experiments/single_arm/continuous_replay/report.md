# 连续回放实验报告

状态：**实机遥测验收通过；现场目视记录未采集**。用户本轮明确免除重复现场核对，因此以下结论只覆盖控制器反馈、CAN 帧和独立状态检查，不声称已目视确认桌面、连杆或线缆间隙。

## 输入、配置与对照

- 固定源：[20 秒示教 CSV](../teaching/data/recordings/nero_session_20260924T101130Z.csv)，SHA-256 `90eadd27a19b86e7b32305c3e27b503fd6d5d073a0f834181b10773d8548dba5`。完整使用 1022/1022 行，其中 998 行拖动反馈、24 行停稳补录；源时长 20.527 秒。
- 环境：`nero-py310`、Python 3.10、`pyAgxArm 1.0.0`、固件 1.21、左侧安装、`can0` 1 Mbps。目标每 50 ms 更新一次；关节目标速度上限 0.20 rad/s、加速度上限 0.40 rad/s²；完整计划时间伸缩 11.5 倍。预设门槛见 [控制计划](plan.md)。
- [旧逐点基线](../replay/data/replays/nero_replay_20260925T152748Z.json)为 322 步正向、322 步反向，约 132.4 秒。每步等待到位；其时间较短，但逐点停顿和本实验的定时连续发布是不同指标。

## 逐级实机验证

| 源片段 | 有效源行 | 目标数 | 独立分析 | 证据 |
| --- | ---: | ---: | --- | --- |
| 1.8 秒 | 91 | 1255 | 通过 | [运行](data/runs/nero_continuous_20260925T162238850869Z.json) · [分析](data/runs/nero_continuous_20260925T162238850869Z_analysis.json) |
| 2.2 秒 | 111 | 1937 | 通过 | [运行](data/runs/nero_continuous_20260925T162704079071Z.json) · [分析](data/runs/nero_continuous_20260925T162704079071Z_analysis.json) |
| 5.0 秒 | 251 | 2161 | 通过 | [运行](data/runs/nero_continuous_20260925T162909105981Z.json) · [分析](data/runs/nero_continuous_20260925T162909105981Z_analysis.json) |
| 完整 20.527 秒 | **1022** | **9445** | **通过** | [运行](data/runs/nero_continuous_20260925T163204971088Z.json) · [分析](data/runs/nero_continuous_20260925T163204971088Z_analysis.json) · [计划](data/plans/nero_continuous_20260925T155533Z.json) |

首次 2.2 秒尝试的[独立分析](data/runs/nero_continuous_20260925T162408162868Z_analysis.json)发现旁路监听少记录最后一组 CAN 指令；末次 SDK 发送已记录、S1 和整机状态正常。修复监听线程关闭前的追帧等待后重跑，1937 组目标和四类关节 CAN 指令帧全部齐全。旧失败文件保留，不能算作通过。

完整实机运行时间：2026-09-25 16:32:10 至 16:40:02 UTC。去程实际 236.071 秒，回程实际 236.055 秒，总运动 472.127 秒。独立分析得到：

| 指标 | 结果 | 预设判定 |
| --- | ---: | --- |
| 顺序记录的目标 | 9445/9445 | 全部 |
| `0x155/0x156/0x157/0x170` 关节指令帧 | 各 9445 | 每目标各一组 |
| 目标周期 p99 / 最大值 | 51.56 / 52.97 ms | p99 ≤80 ms |
| 目标变化、目标运动、反馈运动周期 | 8680 / 8254 / 8254 | 反馈不能持续停顿 |
| 目标运动时 ≥0.3 秒停顿 | 0 次 | 0 次 |
| 最大单关节跟踪误差 | 0.008396 rad | 即时停止门槛 0.10 rad |
| 最大法兰中心位置误差 | 0.003043 m | 即时停止门槛 0.05 m |
| 回本轮实际 S1 最大单关节误差 | 0.000087 rad | ≤0.005 rad |
| 驱动快照 | 456 组 | 全程七轴使能、无欠压或驱动错误 |

计划、[逐周期目标与反馈 CSV](data/runs/nero_continuous_20260925T163204971088Z.csv)、[CAN 旁路帧](data/runs/nero_continuous_20260925T163204971088Z.can.csv)及运行 JSON 均已保留。完成后另行只读检查 `CAN_CTRL`、`NORMAL`、示教停用、错误码零、七轴使能；`teach_session` 判定当前位置接近 S1。这些数据证明目标在运动中持续发布并得到跟踪，不能单独证明人的主观流畅感或外部碰撞余量。

## 重现与限制

从工作区根目录按 [README 的分级命令](README.md)生成新计划、只读复核、显式 `--run`，随后执行 `analyze_run` 和独立状态/S1 检查。本轮完整源所用命令为：

```bash
python -m experiments.single_arm.continuous_replay.continuous_session --plan experiments/single_arm/continuous_replay/data/plans/nero_continuous_20260925T155533Z.json
python -m experiments.single_arm.continuous_replay.continuous_session --plan experiments/single_arm/continuous_replay/data/plans/nero_continuous_20260925T155533Z.json --run
python -m experiments.single_arm.continuous_replay.analyze_run experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T163204971088Z.json
```

历史 JSON 只用于追溯；任何新一轮均从实时状态重新预检。完整计划实际约 8 分钟，源记录的“20 秒”不是要求机器人 20 秒走完。脚本在反馈、周期或误差异常时保留最后目标与实测姿态；不会把中断报告写成成功。

## C2 候选停留及一次误判恢复

使用[源前 3.2 秒计划](data/plans/nero_continuous_20260925T165935418493Z.json)在示教原始第 160 行 C2 处停留 3 秒，随后沿同一路径返回 S1。第一次运行中，实测关节已稳定在 C2 附近，但控制器的 `motion_status` 在 0/1 间切换，旧脚本误判“不连续停稳”，在反向前停止；[失败报告](data/runs/nero_continuous_20260925T170135755958Z.json)中 `stop_action=move_j_hold_current_confirmed`，没有发送电子急停。另行[3 秒静止采样](../pose_recording/data/recordings/nero_poses_20260925T170322Z.csv)确认位置稳定，再按[原轨迹倒序计划](data/plans/nero_recover_prefix_20260925T170523573300Z.json)回 S1；[恢复报告](data/runs/nero_continuous_20260925T170523580058Z.json)为 705/705 个目标、最终误差 0.000035 rad。

停稳判据改为连续至少 0.5 秒的关节目标误差 ≤0.005 rad、反馈变化量 ≤0.002 rad，不以交替的运动状态码作唯一判据。复测的[完整运行](data/runs/nero_continuous_20260925T170640155991Z.json)和[独立分析](data/runs/nero_continuous_20260925T170640155991Z_analysis.json)通过：1469/1469 个目标，其中 705 个去程、60 个保持、704 个回程；四组关节 CAN 指令各 1469 帧；停留反馈最大变化 0.000244 rad；周期 p99 51.76 ms，最大跟踪误差 0.008819 rad，回 S1 误差 0.000070 rad。独立只读状态仍是 `NORMAL`、七轴使能，`teach_session` 判定近 S1。[C2 候选核验](../reachability/data/visits/nero_candidate_C2_20260925T170852026380Z.json)表明 3 秒内实测七轴与原始 C2 的最大误差 0.000436 rad。C2 是否美观仍待用户判断，未改公共起点配置。
