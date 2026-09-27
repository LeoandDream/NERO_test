> **历史续接材料（2026-09-26）**：以下正文按当时会话保留，含已过期的“最新状态”和设备姿态，不能当作当前许可。当前项目入口为 [CURRENT_STATE.md](CURRENT_STATE.md)；正式契约、实验记录和测试等级分别见 [PROJECT_DESIGN.md](PROJECT_DESIGN.md)、[实验规范](docs/experiment_run_standard.md)、[测试审计](docs/test_tiers_audit.md)。后续不再向本文件追加检查点。

# 当前检查点：统一接口、示教立方体与初始化（2026-09-26）

## 08:21Z 最新选择与检查点（以本节为准）

- 用户决定不继续逐面六面采样，改从一条完整的 30 秒、599 行正常拖动轨迹的六个单轴极值构建基座轴对齐的等边立方体**候选**。最新[候选 JSON](experiments/single_arm/workspace_cube/data/candidates/nero_cube_extrema_20260926T082126485145Z.json)包含原始 CSV 哈希、六个极值源行、普通方向 10 mm 与桌面侧 `−X` 30 mm 余量、中心 `[0.3635725,0.0657490,0.4886975] m`、边长 `0.321257 m`、六面中心及八角。状态 `EXTREMA_CANDIDATE_ONLY`、`geometry_valid=false`、`motion_region_verified=false`；面中心距实测轨迹 21～146 mm，不能当运动许可。
- 第二次[六面失败报告](experiments/single_arm/workspace_cube/data/face_sweeps/nero_cube_faces_20260926T081700410097Z.json)记录 `+X=3`、`−X=3`、`+Y=0`，`−Y` 启动因驱动反馈不完整而拒绝，底层结束拖动成功。08:18Z 只读 `lab.cli status` 为模式 2、NORMAL、七轴使能、错误码零、停稳；当前关节姿态与 S1 不同，法兰约 `[0.425719,-0.048180,0.528317] m`。任何后续回位须从届时实时姿态重规划 `move_j`，不要复用旧计划。用户没有要求当前立即回位。
- 随后离线完善回放入口：交互 `quick replay/demo` 会列出连续或逐点选项，回车选连续；高级 `replay plan` 与 Python `plan_replay()/quick_replay()` 必须显式给 `mode`，省略时不连接 CAN。示教结束提示现在给可直接调用的交互快速回放命令；`quick demo` 的示教→刚录记录→二次确认回放流程由替身验证。单臂全套 **110 项测试通过**、`compileall`、`git diff --check` 通过；这轮没有新的实机动作。接口报告与零基础手册已同步。

## 08:14Z 后续检查点（以本节为准）

- 用户确认单轴离桌和三个实时规划 `move_j` 目标回 S1 全程平稳、无异常。最后实机[回位报告](experiments/single_arm/lab/data/planned_return_runs/nero_planned_return_run_20260926T080144527216Z.json)为 3/3 到位、`NORMAL`、七轴使能、误差 0.000140 rad；状态会过期，后续动作仍需在命令内读取实时反馈。
- 六面失败会话只含 `+X` 三个停留点；新增 `quick cube-faces --session <目录>` 断点续录，核对源 CSV 哈希后只补缺面。移除会误拒绝合法空闲模式 2 的重复公共状态预检，真实拖动启动仍由 `can_drag_teach --start` 在同一命令内核对故障和七轴。43 项 `lab` 离线测试、编译和差异检查通过；续录尚未实机验证。旧 `+X` 三点的共面含义尚未现场确认。
- 六面几何仍 **UNVERIFIED**；[额外桌面余量的极值候选](experiments/single_arm/workspace_cube/data/candidates/nero_cube_extrema_20260926T075952478126Z.json)仅为 `EXTREMA_CANDIDATE_ONLY`，不得作为完整几何立方体或运动许可。参见[标定报告](experiments/single_arm/workspace_cube/report.md)。

## 08:02Z 最新检查点（以本节为准）

- `quick cube-faces` 首次实机尝试只完成 `+X` 239 条/3 个停留点；等待下一面期间设备自行退出拖动、反馈中断，失败报告见[六面报告](experiments/single_arm/workspace_cube/data/face_sweeps/nero_cube_faces_20260926T074556653540Z.json)。脚本没有主动发送电子急停帧，根因仍未确认。设备后来显示刹车未释放、七轴失能；现场确认可靠支撑后，单次使能成功。使能前后关节角有显著变化，但现场未注意到异常运动，不能套用旧姿态或旧计划。
- 已修复六面流程在面与面交互等待时仍保持拖动的问题：每面等待确认时不拖动，进入后立即限时采样，结束后退出；离线替身验证通过，**该修订尚未实机复测**。六面拟合仍为 `UNVERIFIED`。
- 用户要求快速利用既有实测极值，桌面侧多留余量。新[候选](experiments/single_arm/workspace_cube/data/candidates/nero_cube_extrema_20260926T075952478126Z.json)基于 599 条正常拖动记录，普通方向内缩 10 mm、左侧装桌面侧 `−X` 内缩 30 mm；含六个**实测到达过**的内缩极值姿态和来源行。它仅是探索参考，面中心距轨迹 21～146 mm，`geometry_valid=false`、`motion_region_verified=false`；不能作为整块立方体/路径运动许可。
- 从异常使能后的新低位实时姿态，现场确认离桌范围后执行一次 J2 `move_j`：[离桌报告](experiments/single_arm/lab/data/low_pose_escapes/nero_low_pose_escape_run_20260926T080036914087Z.json)完成，误差 0.000119 rad、法兰 X 到 +0.167 m、`NORMAL` 七轴使能。随后现场确认整片三目标回位范围，`quick init --target S1 --yes` 在单次入口内规划和执行 3 个 `move_j` 目标：[回位报告](experiments/single_arm/lab/data/planned_return_runs/nero_planned_return_run_20260926T080144527216Z.json)3/3、最终误差 0.000140 rad、`NORMAL` 七轴使能，法兰 `[0.382038,-0.014074,0.572437] m`。用户确认两段全程平稳、无碰桌或支撑、异响或线缆受拉。该反馈会过期，下次实机动作仍从实时状态读取。
- 用户偏好：必要校验并入执行命令，失败立即退出，不要求先运行另一条预检脚本；回位用实时规划的 `move_j`，不倒放轨迹。当前 `low_pose_escape --run` 与 `quick init --target S1 --yes` 均同次完成预检和动作。尚未选定断电支撑位，公共 S1 仍未修改。

## 07:29Z 后最新检查点

- 新 5 秒封存示教已用统一 `lab.cli replay plan/run` 做 20 Hz 定时 `move_j` 完整往返：[报告](experiments/single_arm/lab/data/replays/nero_continuous_20260926T072120577424Z.json)为 2589/2589 个目标、最终误差 0.000122 rad、无停止动作；独立状态 NORMAL、七轴使能，现场认为比逐点回放平稳。之后 CLI 加入显式 `--min-time-scale`；默认最低 6 倍对应此次实机已用参数，12 倍只做过离线计划与重算测试。
- `quick cube-sweep` 已完成一次新的 30 秒实机拖动：[报告](experiments/single_arm/workspace_cube/data/sweeps/nero_cube_sweep_20260926T072610405628Z.json)保留 599 条原始反馈。XYZ 极值边长候选 335 mm，但多个候选面中心距实测点 68～155 mm，`geometry_valid=false`；不能称六面标定或运动许可。
- 录制后从实时七轴姿态生成[新 5 目标 `move_j` 计划](experiments/single_arm/lab/data/planned_returns/nero_planned_return_plan_20260926T072812088999Z.json)并回 S1：[运行报告](experiments/single_arm/lab/data/planned_return_runs/nero_planned_return_run_20260926T072847168319Z.json)为 5/5 到位、误差 0.000105 rad。独立状态 NORMAL、七轴使能；用户现场反馈平稳无异常。原始拖动轨迹未倒放。
- 因现有极值轨迹缺乏真实面点，新增 `quick cube-faces`：一轮拖动、六面分段录制、每面自动从三个分散停留位置取点拟合；**尚未实机执行**。入口和初学者步骤见[说明](experiments/single_arm/workspace_cube/README.md)与[命令行指南](docs/quickstart_cli.md)。最新单臂 103 项测试、`compileall`、`git diff --check` 通过。若继续实机，先重读当前状态并由现场准备六面拖动范围；执行后仍需另用实时 `move_j` 计划回 S1，不能复用上述旧计划。
- 公共默认仍是 S1；整数度 X–Z 平面候选仅保留供比较。断电支撑位尚未实测选定，不能自动停机回支撑。任务强制条件中真实六面局部标定仍为 `UNVERIFIED`，详见[报告](experiments/single_arm/workspace_cube/report.md)。

## 最新离线增量（未发送运动）

用户确认：位置复位、示教后回位使用实时关节规划的 `move_j`；`move_p` 留给 VLA 法兰点位运行。旧命名起点 S1 反向流式路线及法兰 `--to-s1/--to-anchor --run` 已禁用；控制器电子 `reset` 仍只用于解除急停。新增 `lab.cli quick cube-sweep`，一个命令内实时预检、要求键入“开始”、调用现有 CAN 拖动开始/结束、录制 30 秒边界轨迹、离线提取极值候选。无论录制成功与否均尝试退出拖动，报告和原始 CSV 保留；不自动回 S1，也不把候选当运动许可。完整离线单元测试 97 项通过，`git diff --check` 通过。**新边界录制入口尚未实机运行；真实六面立方体面点仍未采齐。**最新已知实机状态仍是三目标 `move_j` 回到旧 S1 并独立确认；该状态会过期，新动作前须重读。

## 06:50Z 最新现场状态与用户决策

用户明确要求：示教结束、从任意正常姿态回安全位置，以及将来停机回支撑，都应**从实时关节反馈重新规划**，不倒放示教轨迹。30 秒边界拖动[原始 CSV](experiments/single_arm/pose_recording/data/recordings/nero_poses_20260926T062951Z.csv)保留；极值候选只用于几何分析，未验证内部/六面。机械臂此前停在该记录终点。单次 `move_p` 到 S1 法兰的只读预检显示近邻 IK 七轴与 S1 最大差约 0.87 rad，因此未执行该方案。现场确认范围清空后，改由实时姿态生成[三目标计划](experiments/single_arm/lab/data/planned_returns/nero_planned_return_plan_20260926T064307080816Z.json)，不读取或倒放 CSV；[执行报告](experiments/single_arm/lab/data/planned_return_runs/nero_planned_return_run_20260926T064431575064Z.json)记录 3/3 到位、S1 最大关节误差 0.000105 rad、`NORMAL`、七轴使能。独立 `teach_session` 只读确认 S1，用户现场确认平稳无异常。

新 `lab.planned_return`、`lab.cli return plan/run`、`LabAPI.plan_return/run_return` 提供从实时姿态重规划 S1 的入口。公共 `quick init --target S1` 也已改走此入口。默认左侧装法兰基座 X 关节盒门槛 0.15 m；状态异常、低位或无法规划时拒绝运动。`teaching.teach_session` 的**新会话**默认在退出拖动并停稳后用同一规划器回 S1，录制轨迹不再充当回位路线；这条自动链路已通过离线替身测试，尚未单独实机验证。历史 `teaching.return_session` 只保留只读检查，其 `--run` 已停用。**断电支撑位尚无用户现场选定的七轴目标，不能把 S1 冒充支撑位或自动失能。**当前机械臂在旧 S1；下次运动仍须重读状态。

本轮任务来自用户粘贴的“单臂实验接口、示教立方体标定与初始化”和后续四点目标：开发者 Python API、用户一键命令行实验、六面示教反推虚拟立方体、从实时状态初始化到命名起点。新入口在 `experiments/single_arm/lab/`，交付要求和证据见[实施报告](experiments/single_arm/lab/report.md)与[接口说明](experiments/single_arm/lab/README.md)。工作区 `TASK_PROMPT.md` 保留用户已有修改，不由本轮重写。

已实现统一 `LabAPI`/CLI、严格封存的新示教记录、逐点快速回放、S1/C2 命名起点、六面立方体几何拟合、状态/记录/探测/地图接口与新手文档。用户提出不想在多条命令间手工传计划，现新增 `lab.cli quick init/teach/replay/demo`：同一调用内预检、显示摘要、键入“开始”后执行；非交互须全参数加 `--yes`。`cube guide` 在一个终端按六面标签引导回车采点，不预填边长。完整测试要在最终改动后重跑。立方体几何不构成运动许可，公共 S1 未改。

本轮已完成专用四段低位恢复到 S1、5 秒新示教及自动回位、132 小步逐点回放、S1→C2→S1 两次命名起点转移，均有原始报告和现场平稳观察。2026-09-26T05:48:29Z 又到达整数度平面候选 `[0,−33,0,−17,0,12,−27]°`，[运行报告](experiments/single_arm/lab/data/pose_candidate_runs/nero_pose_candidate_stage1_20260926T054823337582Z.json)记录 1/1 到位、误差 0.000192 rad、`NORMAL` 与七轴使能；用户现场确认平稳、外观满意，并决定先保留候选、不更改公共 S1。05:54Z 显式进入左侧装 CAN 拖动模式，06:04Z 用户扶稳停稳后显式退出；独立状态无错误、七轴使能，但控制模式仍为 2，实际关节姿态并未回 S1。任何新运动先重新读取状态并规划，不能复用旧计划。之前的四段恢复脚本 `lab.site_recovery` 是一次性现场辅助，不能重跑旧计划。

统一 `poses` 入口已在 `NORMAL` 状态只读采集 3 秒、14 条稳定记录：`experiments/single_arm/pose_recording/data/recordings/nero_poses_lab_20260926T051530860336Z.csv`。用户指出不必先逐个标虚拟面，应先从历史轨迹极值推算。新增只离线运行的 `lab.cli cube extrema --map ... --level ...`，已从完整连续回放的 9445 条反馈产生[候选 JSON](experiments/single_arm/workspace_cube/data/candidates/nero_cube_extrema_20260926T060633142691Z.json)：X/Y/Z 跨度 197/140/176 mm，候选边长约 120 mm；多处候选面中心离旧轨迹 25～47 mm，因此结果明确 `EXTREMA_CANDIDATE_ONLY`，不能当作六面标定或安全运动区域。05:54Z 建立的[会话](experiments/single_arm/workspace_cube/data/sessions/nero_cube_20260926T055434969074Z/manifest.json)仍为零点，没有把移动中/未标注点写成面点。后续若要六面拟合，再用 `cube guide` 每面至少三个非共线点；先完成最终测试与文档走查。任何急停、接触或丢失反馈时先现场支撑、只读复核，旧计划禁复用。

## 上一轮历史检查点

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
