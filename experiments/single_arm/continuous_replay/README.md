# 连续示教回放实验

本实验把 2026-09-24 的完整 20 秒示教记录转换成带时标的关节目标。旧的 `replay_session` 每到一个小步才发布下一步；这里按单调时钟每 50 ms 发布一次 `move_j` 目标，即使上一目标尚未到达也继续更新。能否形成平滑运动由实测反馈判断，不能由命令名称推定。

## 输入与设计

- 原始 CSV：`experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv`
- SHA-256：`90eadd27a19b86e7b32305c3e27b503fd6d5d073a0f834181b10773d8548dba5`
- 全部 1022 行，其中 998 行拖动反馈、24 行退出示教后停稳反馈；不裁掉首尾。
- 五点加权平滑与三次 Hermite 插值；规划要求相对原始记录的最大关节偏差 ≤0.01 rad。正向和反向共用同一条曲线，并回到本轮实际起点。
- 目标最大关节速度 0.20 rad/s、加速度 0.40 rad/s²；按源时间等比例放慢。具体的放慢倍数、总时长与法兰预测范围由离线计划输出。
- 1.8 秒前缀的源记录开头有停留，放慢后前约 27 秒几乎没有位移；脚本仍每 50 ms 发布并记录目标。这段等待属于计划时间，不应误判为进程卡住。完整往返计划约 472 秒。
- 运行时检查七组关节/法兰反馈、整机状态、七轴驱动和 CAN 旁路监听；关节误差 ≥0.10 rad 或法兰位置误差 ≥0.05 m 即停止更新目标。较低门槛持续 0.30 s 也停止。异常后先尝试保持当前实测关节位置，只有无法确认保持或控制器故障时才发送阻尼电子急停。**急停可能导致机械臂下垂。**

详细依据和预设验收门槛见 [plan.md](plan.md)。这套检查无法证明连杆、线缆和桌面之间没有碰撞；实机运行前须现场核对整片活动范围。

## 历史离线规划入口

此前使用 `continuous_replay/tests` 的 `unittest discover` 和 `continuous_session` 的 1.8 秒前缀/完整源规划生成计划。它们会导入项目与 SDK 代码；本轮尚未核定整个导入、构造和异常链的零设备副作用，**不能仅凭本节当作已审定的离线命令运行**。先按[测试分级与静态候选清单](../../../docs/test_tiers_audit.md)审定选中入口，再在相应授权范围内运行并记录解释器、依赖与结果。历史计划文件名带 UTC 时间戳；`--plan <计划路径>` 的历史说明也不替代执行前的调用链审核。

## 分级实机步骤

每轮先确认现场有人监护、底座固定、S1 到该段终点及回程的法兰、连杆、线缆和桌面间隙均已核对；检查没有其它运动命令源。先运行只读状态检查：

```bash
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.teaching.teach_session
```

只有两项显示 `NORMAL`、七轴使能、关节角接近 S1 时，才按顺序分别为源前缀 `1.8`、`2.2`、`5.0` 秒及完整记录创建计划，检查目标范围，再逐轮执行：

```bash
python -m experiments.single_arm.continuous_replay.continuous_session --source-until-s 1.8
python -m experiments.single_arm.continuous_replay.continuous_session --plan <刚生成的1.8秒计划.json>
python -m experiments.single_arm.continuous_replay.continuous_session --plan <刚生成的1.8秒计划.json> --run
python -m experiments.single_arm.continuous_replay.analyze_run <刚生成的运行报告.json>
```

每一轮独立分析为通过、现场运动无异常、再次只读核对 S1 和整机状态后，才能把 `--source-until-s` 依次改为 `2.2` 和 `5.0`，最后省略该参数生成完整记录计划。后续计划必须重新生成；脚本也要求前一规模的独立分析通过。运行时按 Ctrl+C 会停止后续目标并保存实际姿态和失败报告；不要假定自动回到 S1。

## 证据与结果解释

- `data/plans/*.json`：完整源映射、时间参数、全部正反向目标。
- `data/runs/*.json`：状态、七轴驱动快照、开始和结束关节角、异常处理、最终误差。
- 同名前缀 `.csv`：每周期计划与实际目标、关节/法兰反馈、时间戳、跟踪误差。
- 同名前缀 `.can.csv`：CAN 控制帧与反馈帧的旁路记录。
- 同名前缀 `_analysis.json`：独立检查源覆盖、目标顺序、发送周期、CAN 四组关节控制帧、运动中的反馈停顿及回位误差。`acceptance_pass=true` 才表示本轮达到预设量化门槛；还需独立状态检查和现场观察。

2026-09-25 UTC 已按 1.8、2.2、5.0 秒前缀逐级验证，再完成 1022/1022 行完整源记录往返。[实机报告](report.md)和[独立分析](data/runs/nero_continuous_20260925T163204971088Z_analysis.json)记录了 9445/9445 目标、每目标四组 CAN 关节指令、0 次持续 0.3 秒的运动停顿、回 S1 误差 0.000087 rad；独立只读状态检查为 NORMAL、七轴使能。用户本轮明确免除重复现场核对，因此没有目视接触/间隙观察；该缺项在报告中单列。

## 候选位点到达、保持和回程

从已验收的源轨迹前缀生成到 C2 的计划，在源前缀末端连续发布同一关节目标 3 秒，再沿原路径返回：

```bash
python -m experiments.single_arm.continuous_replay.continuous_session --source-until-s 3.2 --turnaround-hold-s 3.0
python -m experiments.single_arm.continuous_replay.continuous_session --plan <刚生成的计划.json>
python -m experiments.single_arm.continuous_replay.continuous_session --plan <刚生成的计划.json> --run
python -m experiments.single_arm.continuous_replay.analyze_run <刚生成的运行报告.json>
```

停稳判据是连续至少 0.5 秒的关节反馈距目标 ≤0.005 rad 且变化量 ≤0.002 rad。持续发布静止目标时，控制器的运动状态码可能在 0/1 间切换，因此不把单一状态码 0 当成停稳的必要条件；整机仍须 `NORMAL`、七轴使能。C2 的[通过报告](data/runs/nero_continuous_20260925T170640155991Z.json)与[独立分析](data/runs/nero_continuous_20260925T170640155991Z_analysis.json)记录 1469/1469 个目标、3 秒保持及回 S1 误差 0.000070 rad。

首轮试验曾因旧停稳判据误判而在 C2 停止，[失败报告](data/runs/nero_continuous_20260925T170135755958Z.json)显示当前姿态已保持、整机没有急停。此类**明确报告为静止保持成功且状态正常**的前缀终点，可使用下列专用命令从该失败报告重建倒序源路径；其它故障不能套用此命令：

```bash
python -m experiments.single_arm.continuous_replay.recover_prefix_return --failed-report <失败报告.json>
python -m experiments.single_arm.continuous_replay.recover_prefix_return --failed-report <失败报告.json> --run
```

历史[倒序恢复报告](data/runs/nero_continuous_20260925T170523580058Z.json)为 705/705 个目标、回 S1 误差 0.000035 rad。脚本在运动前仍检查实时起点与已保存目标相符，异常会停止并保存报告，不自动复位或使能。
