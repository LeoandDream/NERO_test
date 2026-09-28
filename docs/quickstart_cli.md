# Nero 单臂实验：从零开始的命令行步骤

> **历史步骤，勿按本页顺序操作。** 当前完整命令行流程已集中到[根 README](../README.md)。本页保留旧实验过程供追溯；`quick cube-sweep`、`quick cube-faces`、`quick home-demo` 和手动 `can_drag_teach --start` 仍锁定。旧 `supported_home` H0 路线仍锁定，`quick park` 现已改用新的统一命名目标规划。新手请从根 README 第 1 节开始。

**Step 3C 现行边界：**通用回 S1 规划不再默认采用法兰基座 X≥0.15 m 或 X 非负。它只生成候选；缺少有来源、适用于本次整条路线的环境与执行依据时，`return run`、通用 `quick init --target S1`、`starts run` 对该候选和 Teach 启动会在控制动作前拒绝。下文旧执行命令与历史成功描述只供追溯。已标定 H0↔S1 专用路径单独适用，事故锁定入口未解锁。

**历史示教门槛：**2026-09-26 从桌边低位切入拖动后机械臂突然大幅移动。旧 `quick teach/demo` 与 `teach --run` 曾要求实测 S1、正常状态及七轴使能，并核对模式帧后的姿态。Step 3C 起自动回位缺路线依据，Teach 在拖动前拒绝；这些旧条件仍不能单独授权示教。`quick home-demo/cube-sweep/cube-faces`、`can_drag_teach --start` 继续锁定。详见[事故说明](../experiments/single_arm/can_setup/README.md#四通过-can-拖动示教)。

所有命令由现场人员在终端执行。先读每一步的输出，再决定是否进入下一步；旧运动入口须显式加 `--run`，统一 `quick` 入口则要求在终端键入 `开始` 或用完整参数加 `--yes`；使能命令须显式加 `--enable`。当前设备为 Nero 固件 1.21、左侧装、CAN `can0`，安全起点 S1 已记录在[配置](../experiments/single_arm/config/nero_teach.json)。更换安装方向、机械臂或零点后，不能直接复用 S1。

## 新入口：状态、示教、快速回放、起点和立方体

以下命令均从 `/home/leo/nero_dh116_ws`、已激活 `nero-py310` 的终端运行。第一次使用先按下方第 1、2 节检查环境与 CAN。统一入口详情及 Python 调用见[单臂实验接口](../experiments/single_arm/lab/README.md)。带日期的旧计划文件只属于当时姿态，不要复用。

### 最短交互流程

下面是面向现场人员的入口。每条动作命令在**同一次调用**中先显示实时预检和目标；在终端键入 `开始` 后才执行，其他输入取消。它不要求先运行另一个规划脚本。

```bash
# 查看状态；状态异常时先处理，不直接发送运动命令
python -m experiments.single_arm.lab.cli status
# 选择 S1、C2 或桌边 H0；会显示选项
python -m experiments.single_arm.lab.cli quick init
# 从 S1 做一次 5 秒拖动示教，完成后询问是否回放刚生成的记录
python -m experiments.single_arm.lab.cli quick demo
# 日后只回放最近一份完整、同配置的新示教
python -m experiments.single_arm.lab.cli quick replay
```

`quick demo` 会先提示确认示教开始，结束拖动并停稳后从实时姿态重新规划回 S1，再显示记录 ID 和回放计划，再次提示确认回放。若第二次取消，示教原始文件仍保留。`quick replay` 会列出可选的完整记录；`latest` 指**最近一次示教尝试**，若该次失败就拒绝回放，不会悄悄改用较早记录。要用较早的完整记录，须明确输入它的记录 ID 或 CSV 路径。回放会重新校验 CSV/元数据、状态、实时起点、路线与配置，通过后才运行。交互终端会显示 `1 连续 move_j` 与 `2 逐点 move_j`，直接回车选已有一份 5 秒记录实机验收过的连续方式（20 Hz、最低时间倍率 6）。脚本调用必须写明 `--mode`，连续方式还须给 `--frequency-hz` 与 `--min-time-scale`。其他记录仍须逐次核对。

非交互脚本须写全参数并显式授权：

```bash
python -m experiments.single_arm.lab.cli quick init --target S1 --yes
python -m experiments.single_arm.lab.cli quick teach --max-seconds 5 --yes
python -m experiments.single_arm.lab.cli quick replay --recording latest --mode point --speed-percent 5 --yes
# 定时连续关节流，先查看预检给出的总时长和法兰范围
python -m experiments.single_arm.lab.cli quick replay --recording latest --mode continuous --frequency-hz 20 --min-time-scale 6 --yes
```

这些命令仍执行与交互模式相同的实时预检。`--yes` 是本次动作授权，不能作为忽略故障、过期反馈或未知路径的开关。使能/急停复位仍是独立的显式步骤；状态不正常时按第 2 节处理。示教结束若回位失败，不要用 `quick replay` 猜测当前位置。

### A. 状态与初始化

```bash
python -m experiments.single_arm.lab.cli status
python -m experiments.single_arm.lab.cli starts list
python -m experiments.single_arm.lab.cli starts plan --target S1
```

第一条会给出 `ready_for_motion`、`arm_status`、七轴 `enabled`、当前 `joint_rad` 和 `flange_pose_m_rad`。`ready_for_motion: false` 时先看原因：急停、故障或七轴失能按第 2 节或[急停恢复](emergency_stop_recovery.md)处理；已标定桌边 H0 的失能状态可由 `quick init --target S1` 的专用路线处理。不要绕过执行入口的判断。`starts list` 会列出 S1、C2、H0。`starts plan --target S1` 在精确匹配 H0 时也会选择专用离桌路线；`--target H0` 从 S1 生成桌边绕行路线。未知低位仍拒绝。若已在目标，`already_at_target: true`，无需运动。

现场确认底座、左侧安装、裸法兰、支撑/桌面/线缆及整条连杆扫过范围后，可用**单条执行命令**完成实时预检、规划和回 S1；预检失败会退出，不发送运动指令：

```bash
python -m experiments.single_arm.lab.cli quick init --target S1
# 非交互且本次范围已由现场确认时：
python -m experiments.single_arm.lab.cli quick init --target S1 --yes
```

若想先单独查看计划，也可使用上面的 `starts plan`，再把实际输出路径填入：

```bash
python -m experiments.single_arm.lab.cli starts run --plan <上一步的plan_path>
python -m experiments.single_arm.lab.cli status
```

历史执行会重验实时起点，保存动作报告与最终关节误差。C2 外观仍待用户决定；公共默认起点保持 S1。H0↔S1 使用独立的已标定路线；其他正常姿态→S1 的通用规划现只产生候选。候选的法兰 X 检查若未显式给出有来源的条件，会标记为未评估；即使给出条件，也不能代替连杆、工具、线缆和整条路线的环境依据。

### B. 新示教与快速回放

以下是历史示教命令示例。当前 Teach 在缺少可适用的整条回位路线依据时，会在进入拖动/录制前拒绝；不能把这些命令作为当前现场操作步骤：

```bash
python -m experiments.single_arm.lab.cli teach --max-seconds 5
python -m experiments.single_arm.lab.cli teach --max-seconds 5 --run
python -m experiments.single_arm.lab.cli records
python -m experiments.single_arm.lab.cli records latest
```

完整示教会输出新记录 ID、CSV 与 JSON 元数据；`records` 列出当前配置下哈希匹配、限时结束且已验收回 S1 的记录。`records latest` 检查最近一次示教尝试；若该次中断或回位失败，会显示原始失败原因并拒绝，不能自动选较早记录。**旧的 2026-09-24 CSV 不具备此版封存元数据，应继续走旧回放入口，不能伪装成新记录。**

2026-09-26 的一次 20 秒示教在 J4 接近 SDK 下限时中断，退出拖动后控制器留在模式 2、机械臂未回 S1；随后 `quick replay --recording latest` 曾只提示“控制器未处于 CAN 控制模式”。这个中断文件没有退出后的停稳样本，也没有完成回位，**不能回放**。现已改为先报告最近示教本身的 J4 限位错误。发生同类中断时，先查看 `python -m experiments.single_arm.lab.cli status`，从实时姿态恢复到 S1，再重新录制；不要修改失败 CSV、跳过限位或直接发送回放。此次限位边缘姿态使用的一次性恢复命令是 `python -m experiments.single_arm.lab.recover_teach_limit --run`，只识别绑定的那份记录和姿态，不是通用恢复入口。

历史上，完整限时示教若仅在**自动回 S1 的预检**失败，可另行完成回位并用下面的只读命令绑定成功报告。当前通用 `return run` 尚缺执行依据，这条补做路径暂停；旧原始 CSV/JSON 和报告不改写：

```bash
python -m experiments.single_arm.lab.complete_recording --recording <失败示教的CSV路径> --return-report <成功的回S1报告路径>
python -m experiments.single_arm.lab.cli records latest
```

完成凭证仅接受限时结束、有停稳样本、未请求急停、且失败点为回位预检的记录；报告须证明从该 CSV 末端经新的 `move_j` 计划到达 S1，并通过当前 S1 状态复核。任何文件或配置变化都会使凭证失效。轨迹执行仍需另行只读规划并现场核对完整活动范围。

示教结束时，控制器可能在退出拖动后的首条样本短暂报告 `STOP_RECORDING`，随后才报告 `DISABLED`。记录校验接受停稳段开头最多 5 条这种过渡状态，但仍要求至少 2 条后续 `DISABLED`，并拒绝状态倒退。2026-09-27 的 20 秒记录 `nero_session_20260927T052141097432Z` 即有 1 条过渡状态；原 CSV/JSON 未改动，修复校验后 `records latest` 与连续回放只读规划均通过。规划本身不会发送运动指令。

历史流程曾在示教结束后从实时角度重新规划回 S1。当前通用回位候选可用于分析，但缺少整条路线依据时不能执行；以下旧命令仅供追溯：

```bash
python -m experiments.single_arm.lab.cli return plan
# 现场核对输出的每段法兰范围后，用刚生成的路径显式执行：
python -m experiments.single_arm.lab.cli return run --plan experiments/single_arm/lab/data/planned_returns/刚生成的计划.json
```

旧通用 S1 自动回位使用过基座法兰 X≥0.15 m 的假设，但没有获批为项目工作空间。新候选规划不再隐式应用它，也不会因负 X 推断碰撞。统一 `quick init --target S1` 对精确匹配的已标定 H0 仍走独立路线；其他候选缺少执行依据时拒绝自动运动。S1 是带电实验起点，**不是断电支撑位**。

这里的“回位/位置复位”使用 `move_j` 并核对七轴；`move_p` 用于 VLA 的法兰点位运动实验。控制器电子 `reset` 仅用于解除电子急停，执行后关节可能失能，它不是位置复位命令。

当前左侧安装、轻触桌边的**新 H0 候选位**与 S1 之间，优先使用统一入口 `python -m experiments.single_arm.lab.cli quick init --target H0`（S1→H0）或 `--target S1`（H0→S1）。分开规划可用 `starts plan --target H0/S1`，再用 `starts run --plan <plan_path>`。底层也可直接运行 `optimized_home --direction home/start`：不加 `--run` 只读，加 `--run` 执行。默认路线已双向实机验收：7 个目标，J3 最大弯曲约 −0.7 rad，累计关节行程约 4.072 rad；保留高位 Y 负向绕行和桌边单 J1 低速动作。到位后**保持使能**，如需失能须先现场确认实体承托，再单独执行 `disable_at_h0_candidate`。本路线只接受已标定两端点，不能从任意姿态套用。旧 `supported_home` H0 路线仍被锁定。

回家中途若报告“仍停在上一小步附近”，不要直接重跑整条 `--direction home --run`：此时已离开 S1。现场确认停稳后，可用 `python -m experiments.single_arm.lab.optimized_home --direction home --resume-report <失败报告.json>` 只读复核剩余目标，再对同一命令加 `--run` 续行。若中途由人或其他程序移过机械臂，续行入口会拒绝旧报告；从新的实时姿态另行规划。

下一步先生成逐点回放计划，不会运动：

```bash
python -m experiments.single_arm.lab.cli replay plan --recording latest --mode point --speed-percent 5
```

高级 `replay plan` 必须写 `--mode point` 或 `--mode continuous`；Python `LabAPI.plan_replay()` / `quick_replay()` 也必须明确传 `mode`。只在交互 `quick replay` / `quick demo` 中提供可见的模式选项和回车选择。

读 `plan_path`、`forward_steps`、`flange_xyz_envelope_m` 与 `observed_start_joint_rad`。确认实时起点、全部连杆和线缆路线、桌面与支撑间隙后，显式执行：

```bash
python -m experiments.single_arm.lab.cli replay run --plan <上一步的plan_path>
python -m experiments.single_arm.lab.cli status
```

`point` 每个目标到位后才发布下一个 `move_j`，会有小步交界处停顿。`continuous` 使用同一份新示教的实测时序，五点平滑并按关节速度/加速度限制自动放慢，以 10 或 20 Hz 连续发布 `move_j`；首次实机使用前先只读运行：

```bash
python -m experiments.single_arm.lab.cli replay plan --recording latest --mode continuous --frequency-hz 20 --min-time-scale 6
```

核对输出的 `total_duration_s`、`waypoints_count` 和 `geometry_audit`。`--min-time-scale` 允许 6～30，表示至少把原示教时间拉长多少倍；关节速度和加速度限制可能让计划中的 `time_scale` 更大，不能只凭该参数估计总时长。2026-09-26 一份新 5 秒记录以最低 6 倍、实际 10.9 倍及 20 Hz 连续往返完成 2589/2589 个目标，最终关节误差约 0.000122 rad，现场反馈较逐点回放平稳、停顿明显减少；[原始报告](../experiments/single_arm/lab/data/replays/nero_continuous_20260926T072120577424Z.json)可查。两种模式都只执行一轮正向加原路返回，失败保留报告和最后发布目标，不自动重试或从中间猜测回 S1。历史**固定 20 秒源**仍用独立的 `continuous-archive plan/run`，不会静默套用到新记录。

### 停机回零位的拖动演示

S1 是带电实验起点，尚未定义实体支撑的停机回零位。要演示新的回零位置和安全中间路线，先扶稳机械臂并清空拖动范围，然后在工作区根目录运行：

```bash
python -m experiments.single_arm.lab.cli quick home-demo --duration-s 30
```

键入 `开始` 后，程序会进入 CAN 拖动并立即以 20 Hz 记录。请手拖经过你认为安全的中间位置，到实际有实体支撑的回零位，并在终点停稳至少 2 秒。录制结束会退出拖动、留在终点，输出原始 CSV、JSON 报告、起终点和最后 1 秒的关节变化。拖动退出后控制器可能保持模式 2；这可作为录制完成状态，**不能直接发送 `move_j`**。它不会自动回 S1、失能或按录制路线运动。下一步先核对终点支撑和实测路线，再独立设计从实时姿态到该位置的少目标 `move_j` 回零计划；终点不能直接当成已验收的停机位。`poses`/`pose_recording.record_poses` 只读，不能开启拖动，所以不能代替这条命令。

2026-09-26 首次 H0 演示已录到 599 条有效样本，用户确认拖动终点由桌面朝人一侧边缘支撑。原运行报告因错误地要求退出拖动后立刻为 CAN 模式而标失败；[离线复核](../experiments/single_arm/lab/data/home_demos/nero_home_demo_assessment_20260926T093518148277Z.json)确认记录有效。首次 H0→S1 自动回程到位且现场平稳；首次 S1→H0 自动回位虽报告五个关节目标到位，用户随后指出**最后下降接近桌边时擦碰**。终点由桌边支撑是预期的，中途擦碰使路线验收失败。原 H0 配置与路线仍锁定；`quick park` 现已转到新路线，不能执行旧计划。随后基座 X 负向外移约 42 mm 虽到位，用户观察却怀疑该方向朝向支架；此命令也已锁定。

后续以实测负 Y 为离桌方向设计的高位绕行试验，其 S1→外侧点 7 目标到位且现场平稳；外侧点→H0 的 2 目标虽到位，现场仍反馈有擦碰或终点承托不对。[试验入口](../experiments/single_arm/lab/high_y_home.py)现已锁住全部实机执行。不要用 `--run` 重试，也不要把当前 H0 关节反馈当作断电支撑验收；保持使能和可靠支撑，等待查明接触位置后重规划。

### C. 六面立方体几何标定

不必事先给程序边长或中心。若要重新录制一条边界轨迹，从已使能且停稳的状态运行一条交互命令；它会显示实时预检，等你键入 `开始` 才进入拖动，录制 30 秒后退出拖动、保存原始 CSV 并自动提取极值候选，**不会自动回 S1**：

```bash
python -m experiments.single_arm.lab.cli quick cube-sweep
```

非交互脚本须把三个参数写全并显式授权：

```bash
python -m experiments.single_arm.lab.cli quick cube-sweep --duration-s 30 --rate-hz 20 --margin-m 0.01 --yes
```

输出包含 `recording_path`、`map_path`、`candidate_path` 和 `report_path`。录制期间若拖动模式退出、故障或反馈中断，命令会尝试结束拖动，并在报告中记录失败；部分原始 CSV 保留，不会被当成有效标定。完成后机械臂停在实际记录末端；回 S1 须另核路线依据，当前通用 `return plan` 仅供候选分析，`return run` 不接通。若极值跨度太小，候选生成失败，原始记录仍保留。

也可**先用已有轨迹自动提取极值**，无需再次拖动：

```bash
python -m experiments.single_arm.lab.cli cube extrema --map experiments/single_arm/reachability/data/maps/nero_workspace_20260925T164948469376Z.csv --level new_full_continuous_path_sample
# 已完成正常 CAN 拖动的原始 CSV 可直接输入：
python -m experiments.single_arm.lab.cli cube extrema --recording experiments/single_arm/pose_recording/data/recordings/nero_poses_20260926T062951Z.csv
# 已录制的 2026-09-26 边界轨迹：普通方向内缩 10 mm，现场桌面侧 −X 内缩 30 mm
python -m experiments.single_arm.lab.cli cube extrema --map experiments/single_arm/workspace_cube/data/candidates/nero_drag_map_20260926T072641370333Z.csv --level new_hand_guided_sweep --margin-m 0.01 --bottom-margin-m 0.03
```

结果包含六个 XYZ 极值各自的实测姿态、内缩后六个**曾真实到达**的推荐姿态、基座轴对齐等边立方体的候选中心、边长、六面中心、八个角点及各面中心到轨迹最近的距离；产物保存在 `workspace_cube/data/candidates/`。左侧安装已由现场确认基座 X 正向离桌，所以 `--bottom-margin-m` 对桌面侧 `−X` 增加余量。原始拖动记录会先转换为带来源索引的地图 CSV/JSON，原件保留。此法按最窄的内缩轴取边长，另外两轴居中，故六个实测极值不一定落在所构造的六面中心；它只是 `EXTREMA_CANDIDATE_ONLY`，六面、内部、角点和实测点之间的控制器路径均未验证。旧多方向 `move_p` 曾碰桌，绝不可照候选面中心直接发布运动。

当前快速方案的[候选 JSON](../experiments/single_arm/workspace_cube/data/candidates/nero_cube_extrema_20260926T082126485145Z.json)直接来自同一条 30 秒、599 行正常拖动记录；六个原始单轴极值分别来自六个实际样本行，构造的边长约 321 mm。之前的六面分段采样只是另一种更严格的几何验证手段，本次按用户选择不继续采面。当前机械臂在分段采样停止后的姿态，并非 S1；如需回位，应从实时关节角用 `move_j` 重新规划。

若希望用一条命令依次采集六面，使用分段入口：

```bash
python -m experiments.single_arm.lab.cli quick cube-faces
# 非交互脚本：每面录 12 秒，面与面之间各有 5 秒准备时间
python -m experiments.single_arm.lab.cli quick cube-faces --seconds-per-face 12 --rate-hz 20 --yes
# 中途失败后，先只读查看缺面，再在同一命令内续录：
python -m experiments.single_arm.lab.cli cube status --session <session_path>
python -m experiments.single_arm.lab.cli quick cube-faces --session <session_path>
```

启动前扶稳机械臂并清空六面拖动的连杆、法兰、线缆与桌面范围。程序依次提示 `+X/-X/+Y/-Y/+Z/-Z`；这些标签是**你设想的立方体局部面**，不是基座坐标的单轴极值。每面准备好扶稳并按回车，程序才开启拖动并立即开始该面的计时录制；在同一平面上找三个分散位置，各停稳至少 0.3 秒。录完一面就退出拖动，再等待下一面的准备确认。六面录完保存各段原始 CSV，并从停留反馈自动取点拟合。若某面停留不足、点共线或六面不构成等边立方体，结果会明确 `geometry_valid: false`，仍保留原始数据供重录；**不会自动回 S1**。首次实机尝试仅得到 `+X` 部分数据，第二面前设备异常失能，六面标定尚未完成。

`--session` 会核对配置及已有原始录制的哈希，只跳过已存至少三个点的面；旧点仍参与最终拟合。每次开始拖动时，底层命令会在同次调用内核对实时控制器状态和七轴驱动；未通过立即退出，不需要另跑预检。之前[失败会话](../experiments/single_arm/workspace_cube/data/sessions/nero_cube_20260926T074556653307Z/manifest.json)的 `+X` 三点横跨较宽的 X 范围，是否真是同一设想平面尚未确认；若不认可，请省略 `--session` 开新会话，不能把旧点当作可靠面标定。

如果需要**六面几何标定**，由你在现场示教虚拟立方体的六个面：每面把法兰中心放到至少三个分散、不共线的可停留点，按 `+X/-X/+Y/-Y/+Z/-Z` 给对面分组。程序从六组原始点拟合中心、边长和朝向，发现缺面、长方体或面点不一致时明确给出原因。标签指立方体局部面，允许它相对基坐标系旋转；若轴与基座一致，`±X` 是 Y–Z 面，`±Y` 是 X–Z 面。采点前需现场确认工具偏移为零、安装/零点一致，且机械臂停稳；采样命令只读，不替你进入拖动模式或移动机械臂。

可先由现场人员进入已验证的 CAN 拖动模式，在机械臂扶稳后运行一个交互采样命令；每到一个面点，停稳并按回车。输入 `q` 会保留已有原始点，稍后可用同一 `--session` 续采。

```bash
python -m experiments.single_arm.can_setup.can_drag_teach --start --mount left
python -m experiments.single_arm.lab.cli cube guide
# guide 结束或中途取消后，都要显式退出拖动模式
python -m experiments.single_arm.can_setup.can_drag_teach --stop
# 若需要补点，重新进入拖动模式后续采
python -m experiments.single_arm.can_setup.can_drag_teach --start --mount left
python -m experiments.single_arm.lab.cli cube guide --session <会话路径>
python -m experiments.single_arm.can_setup.can_drag_teach --stop
```

退出拖动模式仍须按[CAN 拖动说明](../experiments/single_arm/can_setup/README.md)执行；采样进程不会替你切换控制模式。交互模式按六面各三点采完后自动拟合，输出 `fit_path`、`side_length_m`、`center_base_m` 和有效性。单点采样、离线查看和拟合也可以分开使用：

```bash
python -m experiments.single_arm.lab.cli cube new
python -m experiments.single_arm.lab.cli cube capture --session <session_path> --face +X
python -m experiments.single_arm.lab.cli cube status --session <session_path>
python -m experiments.single_arm.lab.cli cube fit --session <session_path>
```

每移动并停稳在一个点后，按该面重复 `cube capture`；六面共至少 18 个点。`cube status` 显示各面数量，`cube fit` 即使缺面也会保存带原因的拟合结果。只有等边、正交、六面覆盖和面偏差通过，才标 `geometry_valid: true`。它**仍不证明立方体内部可通行，更不证明连杆/线缆无碰撞**；程序不会写入全局运动边界。原始点、配置哈希、时间和拟合结果保存在会话目录。

### D. 小段探测与实测地图

```bash
python -m experiments.single_arm.lab.cli probe plan --axis x --delta-m 0.002
python -m experiments.single_arm.lab.cli map
```

法兰直线探测只有在实际起点与环境匹配且现场核对后，才使用 `probe run --plan <plan_path>`。`map` 只汇总历史实测点和已验收路径，边界数值不是自动碰撞许可。J4 关节小行程可用 `joint-probe plan`；它只适合 J4 已大于 +0.02 rad 的姿态，S1 的 J4 为负，正常情况下会拒绝。`move_p` 的老多方向计划发生过桌面接触，仍不可复用。

## 1. 进入环境

打开终端，逐条执行：

```bash
cd /home/leo/nero_dh116_ws
conda activate nero-py310
python --version
```

版本应为 Python 3.10。如果环境不存在，先运行 `conda env create -f environment.yml`。`pyAgxArm` 是 Python SDK；`can-utils` 是 Ubuntu 工具，用 `sudo apt install can-utils` 安装，不能用 `pip install cna-utils` 替代。

先查看接口是否存在：

```bash
ip -details link show can0
```

若显示 `Device "can0" does not exist`，先检查 USB-CAN 适配器与电脑的 USB 插头、线缆和供电，再运行 `lsusb` 与 `ip -details link show`。此时 `sudo ip link set can0 ...` 不能创建接口；须先让操作系统识别适配器。机械臂 CAN H/L 接线和机械臂供电也要核对，但它们不能解释 Linux 中完全没有 `can0`。

本机此前使用的 USB-CAN 在内核日志中显示为 `candleLight USB to CAN adapter`、USB ID `1d50:606f`，绑定 `gs_usb` 后创建 `can0`。2026-09-25 00:15 集线器断开，随后集线器重新识别时该适配器没有回来；若 `lsusb` 仍不含 `1d50:606f`，优先检查适配器本身的 USB 连接及集线器端口。重新插入后先确认 `lsusb` 出现 `1d50:606f`、`ip -details link show` 出现 `can0`，再考虑启用 CAN 接口。

若 `can0` **存在但为 DOWN**，在本机执行并输入自己的 sudo 密码：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

## 2. 只读确认通信与状态

```bash
python -m experiments.single_arm.can_setup.detect_firmware --channel can0
python -m experiments.single_arm.can_setup.nero_can_activate
```

应看到固件 `1.21`，控制模式 `CAN_CTRL`、整机 `NORMAL`、示教 `DISABLED`、错误码 `0x0000`、七个驱动 `使能=True`。若状态为 `EMERGENCY_STOP` 或 `JOINT_BRAKE_NOT_RELEASED`，按[急停恢复步骤](emergency_stop_recovery.md)处理；驱动位为 `True` 不足以证明急停下还能保持位置。

## 3. 只读确认当前位置是否为 S1

```bash
python -m experiments.single_arm.teaching.teach_session
```

输出“当前位置接近 S1”表示关节位置通过脚本阈值核对。这条命令不加 `--run`，不会切入拖动示教。若不接近 S1，不要直接运行 S1 示教或历史回放；先检查现场姿态和[起点转移说明](../experiments/single_arm/start_transfer/README.md)。

2026-09-25 已用两次完整的控制器 `move_j` 从当时的临时姿态回到 S1，随后独立只读检查通过。带时间戳的旧计划和审计文件只适用于那次起点，**不能重放**；今天若只读检查已经通过，就跳过回位步骤。

## 4. 试验控制器 IK 与单次法兰直线

先运行只读预检：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --dx-m -0.002
```

`-0.002` 表示沿基座 X 轴负方向平移 2 mm，法兰方向保持不变。输出须包含当前/目标六维位姿、预测关节变化及“未发送运动指令”。本机 IK 只用于检查可能的连续解；运动时控制器自行计算 IK 并执行直线。此脚本只发送一个 `move_l` 目标。当前工作空间边界为 `null`，因此**现场要检查连杆、线缆、台面和支撑**；法兰范围不足以排除碰撞。

只有在机械臂停稳、底座固定、安装/零点正确、整条运动范围清空并有人现场监护时，才执行：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --dx-m -0.002 --run
```

历史示例曾从实时七轴角重新规划 `move_j` 回 S1；当前通用回位执行入口未接通，以下命令仅供追溯：

```bash
python -m experiments.single_arm.lab.cli return plan
python -m experiments.single_arm.lab.cli return run --plan 上一步输出的计划.json
```

第二条命令现在会在缺少执行依据时拒绝，不能以候选计划代替路线和环境验证。法兰实验的历史执行记录写入 `experiments/single_arm/control_interfaces/data/runs/`；回位报告曾写入 `experiments/single_arm/lab/data/planned_return_runs/`。不得通过另换阈值强行回位。

若不在 S1，可先用只读位置记录建立**临时锚点**，按[控制器 IK 的临时起点步骤](../experiments/single_arm/control_interfaces/README.md)测试小范围运动。临时锚点不覆盖 S1 历史配置；回位执行应使用关节目标。14:08 的记录曾用于 2 mm 历史往返试验：当时去程、回程各发送一条 `move_l`，控制器均报告到位；现场确认回程平稳，随后独立状态检查为 `NORMAL`、七轴使能。现在法兰回位执行入口已停用。

2026-09-25 又从 S1 完成 X 负向 10 mm、再回 S1 的[单次直线往返](../experiments/single_arm/control_interfaces/report.md)：去程和回程各一条 `move_l`，均到位；回程最大关节误差约 `0.000419 rad`，独立状态与 S1 检查通过，现场确认平稳。这个结果只适用于当时核对过的路线；下一次仍须从实时姿态重新预检。

若设备曾无预警失能，可在现场支撑机械臂后，单独开一个终端运行只读诊断：

```bash
python -m experiments.single_arm.can_setup.monitor_state --duration 300 --rate 2
```

它会同时保存状态 CSV 和急停/使能 CAN 帧 CSV；结束时打印两个路径。它不自动使能、复位或运动。如何根据两份记录区分“收到了控制命令”和“未见控制命令”，见[失能诊断步骤](emergency_stop_recovery.md)。

若机械臂突然停机，先在现场支撑并记录**准确时间、底座灯色、适配器灯色以及灯色后来是否变化**；再用第 2 节的只读状态命令读取整机状态和七轴使能位。不要因为底座灯曾短暂变绿，就推断稍后的驱动仍已使能。若已有后台监测，保留状态、控制帧及主机侧 CSV；[分析命令和记录位置](emergency_stop_recovery.md)可帮助对齐失能、CAN 网卡变化和本工作区进程退出的时间。日志未覆盖事件瞬间时，原因应记录为未确认。

## 5. 控制器 IK 与 VLA 法兰点位 `move_p`

旧普通回位、位置复位和示教结束返回 S1 曾使用 `lab.cli return plan/run`；当前通用入口只支持候选分析，自动执行尚缺整条路线的环境与适用依据。VLA 运行时若发布法兰六维**终点**，才使用 `move_p`；控制器会求终点 IK，但中间法兰路径未知，同一法兰位姿也可能对应另一组七轴角。控制器电子 `reset` 是急停故障处理，不是位置回位。

2026-09-25 的 2 mm、4 mm `move_p` 去程和回程是接口验证的[历史实验](../experiments/single_arm/control_interfaces/report.md)，不是当前 S1 回位步骤；其中 4 mm 试验的法兰中途相对起点最大移动约 8.4 mm。VLA 接入前应按实时状态和现场范围重新审查其目标及停止条件，不能照搬历史点位计划。

历史[多方向点位演示](../experiments/single_arm/control_interfaces/README.md#多方向点位演示)从示教记录挑选五个候选目标，包含 Y 正负、X 负、Z 负和三轴组合。这份演示计划已停用，不应按旧说明继续执行。旧的 `cartesian_point_session.py --dx-m -0.04` 也不会自动变成大范围演示命令。

**2026-09-25：多方向点位计划现已停用。**宽范围计划完成 80 mm、114 mm、93 mm 三组往返后，X 负向第 7 段[碰到桌面并触发电子急停](../experiments/single_arm/control_interfaces/report.md#2026-09-25-x-负向点位发生桌面接触实验停止)。脚本会拒绝继续执行旧计划。先按[急停处理说明](emergency_stop_recovery.md)确认现场、支撑和稳定姿态；不要执行多方向计划的第 8 段或后续目标。桌面、支撑和线缆需要纳入下一版环境碰撞模型后才能重新规划。

若已经完成上述急停复位、稳定性检查和单次使能，并处于 2026-09-25 记录的低位，可按[单次抬升步骤](../experiments/single_arm/start_transfer/README.md#2026-09-25-桌面接触后的单次抬升)先运行 `python -m experiments.single_arm.start_transfer.lift_from_table` 只读预检。它仅适用这次左侧安装和低位记录；抬离桌面后再从实测姿态规划回 S1。

本次单次抬升已完成，现场确认法兰离开桌面。后续回 S1 的[两目标回位脚本](../experiments/single_arm/start_transfer/return_after_table_lift.py)仍须逐段先只读预检：`python -m experiments.single_arm.start_transfer.return_after_table_lift --stage 1`；只有实时起点及现场范围吻合，才考虑执行该段。旧多方向 `move_p` 计划继续停用。

## 6. 其他实验

[单臂实验总览](../experiments/single_arm/README.md)依次链接位置记录、拖动示教、轨迹回放和法兰 X 实验。先运行各 README 的只读命令，再按现场条件决定是否加 `--run`。20 秒源记录的旧逐点回放与新的定时连续回放均已有独立报告；两者的发布方式和时间不能混为一谈。任何中断后，先读状态、确认支撑和位置，再决定下一步。

## 7. 连续回放与可达空间

连续回放以完整 20.527 秒示教 CSV 为输入，离线生成 20 Hz 时标关节目标。2026-09-25 UTC 已按 1.8、2.2、5.0 秒前缀逐级验证，并完成 9445 目标的[完整实机往返](../experiments/single_arm/continuous_replay/report.md)；实际去程与回程各约 236 秒，独立分析和回 S1 状态通过。下一次仍须从实时 S1 重新只读预检，不能把历史运行报告当作当前机械臂状态。

若只是复核旧数据而不运动，可运行：

```bash
python -m experiments.single_arm.continuous_replay.analyze_run experiments/single_arm/continuous_replay/data/runs/nero_continuous_20260925T163204971088Z.json
python -m experiments.single_arm.reachability.analyze_probes
python -m experiments.single_arm.reachability.build_map
```

第一条检查完整回放的周期、CAN 四组关节指令、反馈运动和回位误差；第二条检查 16 段新 `move_l` 探测及段间连接；第三条重建有来源等级的空间地图。运行新实验的精确命令与顺序见[连续回放 README](../experiments/single_arm/continuous_replay/README.md)和[可达空间 README](../experiments/single_arm/reachability/README.md)。

本轮可达空间探测从 S1 做三轴小范围连接，再参考历史受控记录逐段沿 Y 正向到达 Y≈+0.0229 m，保持 3 秒后返回。16 段与 15 条段间连接通过独立检查，地图包含本轮 647 条新探测反馈及完整回放的 9445 条反馈。曾碰桌面的旧 `move_p` 第 7 段不能再次运行。地图的三轴包围盒不代表内部空间都安全；C2 初始化候选另已[实机精确到位并保持 3 秒](../experiments/single_arm/reachability/data/visits/nero_candidate_C2_20260925T170852026380Z.json)，但外观仍待用户判断，未更改正式 S1。C1、C3～C5 只有回放近邻证据。

## 三种控制命令的区别

| 输入 | SDK 命令 | 本工作区用途 |
| --- | --- | --- |
| 七个关节角 | `move_j` | 示教回放、每段一次完整目标的 S1 恢复 |
| 法兰位置和姿态六个数 | `move_p` | 控制器 IK 求终点，点位运动；中间法兰路线不能按直线假定 |
| 法兰位置和姿态六个数 | `move_l` | 新的单次直线运动，由控制器 IK 求解 |

`get_ik_joint_angles()` 是笛卡尔命令下发后的反馈接口，不能在运动前把它当作控制器的只读轨迹规划服务。首次 2 mm 往返的[运行记录](../experiments/single_arm/control_interfaces/report.md)含控制器 IK 反馈并显示两次到位，现场反馈平稳、无异常。

`move_p` 的[点位命令行实验](../experiments/single_arm/control_interfaces/README.md)已完成首次 2 mm 去程和回 S1，均由控制器报告到位，现场反馈平稳。报告中的少量法兰采样显示点位运动偏离理想直线；下一次仍须检查当前状态、终点和整个连杆活动范围，不能把 `move_l` 的直线判断套用到 `move_p`。
