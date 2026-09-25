# Nero 连续回放与可达空间实验检查点

状态：**连续回放、已执行可达路径及 C2 初始化候选精确到位/保持的遥测验收通过；外观判断为 AWAITING_USER**。本轮用户明确表示无需重复核对现场，因此没有目视碰撞/间隙观察记录。不可把遥测通过写成现场全空间无碰撞证明。

## 已完成

- 完整 20.527 秒、1022 行源记录：20 Hz 定时 `move_j`，9445/9445 个目标，往返实际 472.127 秒，0 次目标运动期间持续 0.3 秒的反馈停顿，回 S1 最大单关节误差 0.000087 rad。完整[计划](experiments/single_arm/continuous_replay/data/plans/nero_continuous_20260925T155533Z.json)、[运行](experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T163204971088Z.json)、[独立分析](experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T163204971088Z_analysis.json)、[报告](experiments/single_arm/continuous_replay/report.md)均已保存。源 SHA-256：`90eadd27a19b86e7b32305c3e27b503fd6d5d073a0f834181b10773d8548dba5`。
- 逐段可达空间：16 段新 `move_l`、647 条反馈、15 条段间连接通过[独立验收](experiments/single_arm/reachability/data/analysis/nero_probe_series_20260925T164924139042Z.json)。先做 X/Y/Z 小范围探测，随后参考历史受控点沿 Y 正向累计 37 mm 到 Y≈+0.02291 m，静止 3 秒，再沿相同段返回。完整[地图](experiments/single_arm/reachability/data/maps/nero_workspace_20260925T164948469376Z.json)和[报告](experiments/single_arm/reachability/report.md)按证据等级保存。物理极限及未执行路径仍未知。
- C1～C5 为非零关节角较齐整的历史实测候选；完整回放正向反馈与这些角度最近差 0.00026～0.00138 rad。[候选数据](experiments/single_arm/reachability/data/candidates/nero_init_candidates_20260925T164019191778Z.json)保留生成当时的状态。C2 另已沿已验收的示教轨迹到精确关节目标、保持 3 秒并原路回 S1：[完整运行](experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T170640155991Z.json)、[独立分析](experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T170640155991Z_analysis.json)、[候选核验](experiments/single_arm/reachability/data/visits/nero_candidate_C2_20260925T170852026380Z.json)。保持期相对原始 C2 关节角最大误差 0.000436 rad，变化 0.000244 rad；回 S1 误差 0.000070 rad。C2 可作为遥测验证的初始化候选，主观外观仍待用户选择，公共 S1 未改；C1、C3～C5 尚无精确停留验收。
- C2 首轮试验因状态码 0/1 交替而误判未停稳；[失败报告](experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T170135755958Z.json)没有急停，已按[源轨迹倒序](experiments/single_arm/continuous_replay/data/plans/nero_recover_prefix_20260925T170523573300Z.json)安全回 S1，随后修正关节反馈停稳判据并完整复测通过。两次原始记录均保留。
- 本轮最后只读状态：`CAN_CTRL`、`NORMAL`、示教停用、错误码零、七轴使能；`teach_session` 判定接近 S1，关节反馈约 `[0.017366, -0.578472, 0.027908, -0.299673, 0.010856, 0.205530, -0.473316]` rad。最后运动目标与反馈见 C2 完整运行报告。**该读数会过期，新运动前必须重新读实时状态。**

## 后续若继续

1. 首先重新运行 `python -m experiments.single_arm.can_setup.nero_can_activate` 和 `python -m experiments.single_arm.teaching.teach_session`，确认当前 CAN、七轴和 S1；不要假定机械臂仍在本次结束位置。
2. 若考虑把 C2 作为正式初始位，先由用户判断外观和实际间隙；若选择 C1、C3～C5，还要沿已验证路径在精确七轴目标到位并保持后才能推荐。没有用户外观选择前不修改共同 S1 配置。
3. 若要越过 Y≈+0.023 m 或其它历史边缘进入未评估区域，先补足桌面、支撑、线缆和连杆的环境约束。旧 `point_workspace_demo` 第 7 段曾发生法兰碰桌面，禁止复用该计划。

异常停止时保留最后目标与实时反馈；脚本优先尝试保持当前姿态，无法确认时可能发阻尼电子急停。参见[急停恢复说明](docs/emergency_stop_recovery.md)。
