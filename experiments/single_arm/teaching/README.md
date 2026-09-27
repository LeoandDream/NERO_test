# 限时拖动示教实验

**S1 限时示教有条件解锁：**2026-09-26 从桌边低位切入拖动发生突然大幅上抬。`--run` 仅在实时 S1、七轴使能、状态正常且停稳时发送模式帧；安装方向帧后再次核对姿态，启动后前 0.5 秒监测关节突变并在异常时请求退出。桌边低位和无时限手动拖动仍锁定。S1 的限时切换和零目标收尾已实机平稳完成；**真正拖动后的自动回位尚未由这次试验验证**。运行时应扶稳机械臂，切入模式后先保持静止 0.5 秒。见[CAN 拖动事故说明](../can_setup/README.md#四通过-can-拖动示教)。

2026-09-26 14:11Z 已从 S1 做一次 1 秒切换试验，现场平稳且首 0.5 秒反馈未突变。由于没有拖动，自动回位规划为 0 个目标；原代码未切回 CAN 导致会话最终验收失败。零目标分支现已修复，本次用独立命令切回 CAN、未发送关节目标；修复后的完整会话仍待实机复测。失败 CSV 的 `return_completed=false`，不能用于正式回放。

随后 [5 秒完整会话](data/recordings/nero_session_20260926T164431516657Z.json)完成：225 条拖动期采样、27 条退出后采样，现场确认平稳且基本没有手动移动；法兰相对起点最大位移约 1.6 mm。程序规划 0 个 `move_j` 回位目标，成功切回 CAN，最终状态 `NORMAL`、七轴使能，元数据 `return_completed=true`。这验证了零目标分支，**不代表一条实际拖动轨迹的自动回位已经验收**。

新录制完成后，脚本现在打印记录 ID、CSV 路径和快速回放的只读命令；同名 JSON 包含 CSV/配置 SHA-256。统一入口使用 `python -m experiments.single_arm.lab.cli teach --max-seconds 5` 先预检，再用相同命令加 `--run` 显式示教；随后 `python -m experiments.single_arm.lab.cli records latest` 校验最近一次示教尝试。若最近一次失败，`latest` 会报告原始错误并拒绝回放；要用更早的完整记录，必须显式写记录 ID。完整步骤见[新手指南](../../../docs/quickstart_cli.md#b-新示教与快速回放)。旧记录缺少新封存字段，不能冒充新记录。

2026-09-26 一次 20 秒拖动在 J4 接近 SDK 下限时中断，记录 `nero_session_20260926T183143020488Z` 的 `return_completed=false`、`post_stop_samples=0`，退出拖动后仍为控制模式 2。该 CSV 不能作为回放源。现场使用绑定这次事故的 `lab.recover_teach_limit` 一次性脚本先切回 CAN、低速让 J4 离开下限，再用实时规划的 4 个 `move_j` 目标回 S1；独立状态为 NORMAL、七轴使能，现场反馈全程平稳。通用示教失败仍须先读状态并根据实时姿态处理，不能套用这条一次性命令。

随后一次 5 秒示教正常结束并补录了停稳反馈，但自动回位预检报告“当前未停稳或控制器/七轴状态不允许回位”，因此没有发布回位目标，控制器留在空闲示教模式 2。旧报告没有保存该次预检快照，无法确定是哪一项瞬时状态触发。现已将回位预检改为逐项报告并保存实时状态；仅当短暂关节变化超过 0.002 rad、其余状态正常且累计姿态变化不超过 0.01 rad 时，最多进行三次只读复测。驱动失能、故障、错误码或仍在拖动时立即退出。此失败记录仍标记 `return_completed=false`；只有按[新手指南](../../../docs/quickstart_cli.md#b-新示教与快速回放)独立回 S1 并生成可核验的完成凭证后，`latest` 才可将其用于回放预检。

新版示教结束后先退出拖动、补录停稳反馈，再从**实时七轴角**生成少量 `move_j` 回 S1 目标；原始示教 CSV 不参与回位规划。若起点、状态或关节盒法兰 X 门槛不满足，停止回位并保留 CSV/JSON 及失败原因。目标未执行而机械臂稳定停在上一点时暂停；真实运动异常仍可能触发电子急停。失败后须从实时姿态重新规划，不复用旧计划。

在工作区根目录、`nero-py310` 环境中运行。默认配置为 [S1](../config/nero_teach.json)，原 P4 配置单独保留。脚本要求机械臂在 S1 附近；它不会从任意位置自动移动到 S1。

先执行只读检查：

```bash
python -m experiments.single_arm.teaching.teach_session
```

现场确认底座固定、安装方向和关节零点正确，拖动范围清空，准备扶稳机械臂后，再显式运行，例如最长 5 秒：

```bash
python -m experiments.single_arm.teaching.teach_session --run --max-seconds 5
```

会话记录起点、拖动轨迹和退出拖动后的停稳过程，并独立规划回配置中的 S1。默认最长 120 秒；命令行可设 1–300 秒。CSV 和同名 JSON 写入 `experiments/single_arm/teaching/data/recordings/`；元数据中的 `return_plan` 记录实时起点、关节目标及预测法兰范围。当前左侧安装的自动回位要求关节盒预测法兰基座 X 不低于 0.15 m；起点更低或规划不通过时保留原始文件、拒绝运动。软件预测不覆盖桌面、支撑和线缆，现场仍须核对完整扫过空间。

示教结束未回位时，从当前姿态重新规划；默认只读，确认现场范围后再执行：

```bash
python -m experiments.single_arm.lab.cli return plan
python -m experiments.single_arm.lab.cli return run --plan 上一步输出的计划.json
```

`teaching.return_session` 只保留历史记录的只读检查；`--run` 已停用，不能倒放运动。停机支撑位目前没有经过现场选定的关节目标，不能把 S1 当成断电支撑位；确定目标后方可加入命名位置与现场限制。历史过程、失败样本和回位结果见[实验报告](report.md)。

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
