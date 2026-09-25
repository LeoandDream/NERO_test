# 单臂可达空间与初始化位点

这个实验把历史手动示教、历史受控动作和本次新探测分开保存。法兰 XYZ 的每轴最大最小值只是数据摘要；其内部点、两个样本之间的连线，以及工具和连杆的空间都不能因此视为安全。

## 离线地图和候选

在仓库根目录运行，不连接 CAN：

```bash
python -m experiments.single_arm.reachability.build_map
python -m experiments.single_arm.reachability.initial_candidates
```

第一条命令读取完整 20 秒原始示教 CSV、历史完成的 `move_p` 反馈、独立验收通过的本轮完整连续回放、独立验收通过的新探测及桌面接触报告，输出带来源级别的样本 CSV、范围 JSON 和 X/Y 投影 SVG。灰色是手动示教，蓝色是旧受控路径，橙色是本轮完整连续回放，绿色是本轮逐段 `move_l` 探测。颜色不能互换；包围盒也不能当作已验证的内部可达空间。

第二条命令只在**实际记录过**的七轴样本中选 5 个相互分开的非零候选。评分为七轴各自偏离最近 5° 整数倍的平均绝对角度，越小越齐整。JSON 提供七轴 rad/degree、法兰六维位姿、SDK 限位余量、距 S1 的最大关节差、旧逐点回放近邻，以及本轮完整连续回放的实测路径近邻。原始候选 JSON 保留生成当时的验证状态；后续实机到位与保持证据另存 `data/visits/`。外观是否美观由用户判断。

## 新的逐段实机探测

`controlled_probe` 每次只允许一个坐标轴移动 2～10 mm，默认只读创建计划。实际运动使用单次 `move_l`，控制器负责 IK 与直线插补；本机近邻 IK 只用于预检。每段保存计划、逐次反馈、驱动状态和 CAN 旁路帧，不自动返回。新点继续扩展前，要从**实时停稳姿态**重新建计划；若已远离 S1，先用 `pose_recording.record_poses` 生成稳定锚点 CSV，传入 `--anchor-recording`。这次 Z 回程第一次只读规划因缺少远端锚点而被拒绝；补录锚点后才生成新计划并执行。

首次建议从 S1 以 X 正向（安装现场确认过远离桌面的方向）做 2 mm 试探：

```bash
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.teaching.teach_session
python -m experiments.single_arm.reachability.controlled_probe --axis x --delta-m 0.002
python -m experiments.single_arm.reachability.controlled_probe --plan <刚生成的nero_probe计划.json>
```

前两项只读核对 `NORMAL`、七轴使能、S1。计划输出包含实际起点、法兰目标和关节预测变化。实机前须现场确认底座、法兰、整条连杆、腕部、线缆、桌面和支撑间隙，以及唯一运动命令源和可用的现场停止方式；再对**刚刚复核的计划**执行：

```bash
python -m experiments.single_arm.reachability.controlled_probe --plan <刚复核的nero_probe计划.json> --run
```

脚本要求实时起点与计划吻合，实际法兰距计划直线 ≤5 mm、相对起点单关节变化 ≤0.25 rad、法兰姿态变化 ≤0.03 rad；越界会停止，先尝试保持当前实测姿态，无法确认时才发阻尼电子急停。急停可能使机械臂下垂。每段只有到位并连续采样确认、`NORMAL`、现场无异常后才考虑新的方向或更远目标；不得执行已发生桌面接触的旧 `point_workspace_demo` 第 7 段。

每轮若干段完成后执行独立复核，再刷新地图：

```bash
python -m experiments.single_arm.reachability.analyze_probes
python -m experiments.single_arm.reachability.build_map
```

`analyze_probes` 核对每段完成标记、五次连续到位反馈、法兰偏线/关节/姿态门槛、七轴使能快照、三类 `move_l` CAN 指令帧及段间实际位置连接。只有其接受的探测报告才进入绿色地图。每次计划和运行报告都使用 UTC 微秒文件名，不覆盖原始数据。

本轮从 S1 做了三轴小范围探测及逆向返回，又参考历史成功的侧向受控点，以 Y 正向 10+10+10+7 mm 到达 Y≈+0.0229 m，静止保持 3 秒，再沿原段返回。16/16 段和 15/15 条连接通过[独立分析](data/analysis/nero_probe_series_20260925T164924139042Z.json)，647 条反馈进入[新地图](data/maps/nero_workspace_20260925T164948469376Z.json)。详情见[报告](report.md)。

## 边界与初始化位点

本安装曾在 `move_p` X 负向第 7 段发生法兰桌面接触，见[原始故障报告](../control_interfaces/data/runs/nero_point_demo_leg7_20260925T142300Z.json)。故障后的 x≈−0.021 m 是急停后位置，**不是**精确碰撞边界；不能拿单一 x 值切出安全工作空间。移动了底座、桌面、工具或线缆后，地图与候选证据要重新核验。

本轮新探测及历史边缘点有实机到位和停稳证据；它们证明已执行的具体连接路径，不证明任意点可达。C2 已沿示教源前 3.2 秒轨迹在精确七轴目标附近保持 3 秒，再原路回到 S1；[候选独立核验](data/visits/nero_candidate_C2_20260925T170852026380Z.json)通过。可复核命令：

```bash
python -m experiments.single_arm.reachability.analyze_candidate_visit \
  --candidates experiments/single_arm/reachability/data/candidates/nero_init_candidates_20260925T164019191778Z.json \
  --name C2 \
  --run-report experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T170640155991Z.json
```

C2 现为遥测验证过的非零初始化候选，外观判定 `AWAITING_USER`；公共 S1 配置未改。C1、C3～C5 仍只有近邻路径证据，未在精确候选关节角处停稳。详见[实验报告](report.md)。
