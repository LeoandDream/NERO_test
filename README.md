# Nero 单臂实验：零基础命令行手册

本仓库使用 Python 3.10、SocketCAN 和 `pyAgxArm` 控制左侧安装的 Nero。下面的命令都在**工作区根目录**执行。第一次做实验，按“准备 → 看状态 → 到 S1 → 示教 → 回放 → 回家”的顺序；其他实验放在后面的命令索引。

**Step 3C 当前入口变化：**旧通用回 S1 的 `X≥0.15 m` 默认已移除。`return plan` 和非 H0 姿态的 `starts plan/quick init --target S1` 现在只生成 `candidate_only` 候选；`return run`、这类候选的 `starts run/quick init` 会在控制指令前拒绝。依赖自动回 S1 的新 `teach --run/quick teach/demo` 也在拖动前拒绝，直到有经核定的环境与路线依据。下方原有实机示教/通用回位步骤记录历史流程，**当前不可照其动作命令执行**。精确匹配的 H0↔S1 专用分流未因本项修改解锁或重认证。

> **先认清两个位置：**S1 是七轴使能时的实验起点，不是断电承托位；H0 候选位是法兰轻触朝人一侧桌边的支撑位。位置回位使用 `move_j`，电子 `reset` 只用于解除急停，不能让机械臂回到某个姿态。所有日期命名的旧计划和记录仅证明当时那次实验，不能从新的实时姿态直接重放。

阅读本页不会连接或移动机械臂；执行带动作的命令时，程序会连接控制器。人要在现场查看底座、桌面、法兰、全部连杆、腕部、线缆和支撑的活动范围。若预检失败或运动中止，就停在当时状态，读失败报告，不把同一条 `--run` 命令再执行一遍。

**最常用的两件事：**

| 想做的事 | 起点 | 命令与行为 |
| --- | --- | --- |
| 启动到 S1 | 已标定的 H0；其它姿态目前仅生成候选 | `python -m experiments.single_arm.lab.cli quick init --target S1`；精确 H0 仍选专用路线。其它姿态只打印候选并在动作确认前拒绝 |
| 回家并失能 | S1、七轴已使能、`NORMAL` | 先 `python -m experiments.single_arm.lab.cli quick init --target H0`；确认 H0 法兰轻触承托后，再 `python -m experiments.single_arm.lab.disable_at_h0_candidate --run` |

`quick init`、`quick park` 和 `starts plan/run` 共用 S1、C2、H0 命名目标入口。已标定 H0→S1 与 S1→H0 仍走各自专用路线；其它正常姿态→S1 不再隐式套用法兰 X 数值，数值候选不等于路线执行许可。2026-09-27 的非 H0 低位试验记录只对当时路线有效。

## 0. 命令怎么读

- 只复制代码块里的命令行，不要复制终端提示符、中文括号或说明文字。
- `<计划路径>`、`<会话路径>`是占位符，要换成**上一步终端实际打印**的路径，尖括号也要删掉。先不要手输旧日期文件名。
- 不带 `--run` 的专用运动脚本通常只做实时预检；加 `--run` 才执行。统一 `quick` 命令会在**同一次调用**中预检，并要求在终端键入“开始”；键入其他内容会取消。`--yes` 仅给明确授权的非交互脚本使用。
- 所有关节角单位是 rad，法兰位置单位是 m，命令里的 `0.002` m 是 2 mm。回放的 `point` 表示逐点 `move_j`，`continuous` 表示定时连续 `move_j`；法兰 `move_p` 是另一种控制方式。

## 1. 准备电脑和 CAN

在终端输入（将示例路径换成实际项目根目录）：

```bash
cd /path/to/nero_dh116_ws
conda activate nero-py310
python --version
ip -details link show can0
```

已有 `nero-py310` 环境就不要重新创建；没有时才运行 `conda env create -f environment.yml`。若最后一条显示 `can0` 存在但为 `DOWN`，在本机输入 sudo 密码后启用电脑侧 CAN，再查询：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

若提示 `Device 'can0' does not exist`，先检查 USB-CAN 模块、接线和 `lsusb`；启用命令不能凭空创建丢失的接口。固件和状态只读查询：

```bash
python -m experiments.single_arm.can_setup.detect_firmware --channel can0
python -m experiments.single_arm.lab.cli status
python -m experiments.single_arm.can_setup.nero_can_activate
```

状态需要显示 `NORMAL`、七轴使能且无错误，才可进入普通运动实验。若 `JOINT_BRAKE_NOT_RELEASED`、七轴 `False`，**先判断是否处于下面第 2 节已验收的桌边 H0**：若是，直接运行统一 S1 启动入口，不要提前手动使能。其他已确认可使能的姿态，先确认机械臂停稳、实体支撑可靠及夹点清空，再**单次**使能：

```bash
python -m experiments.single_arm.can_setup.nero_can_activate --enable
python -m experiments.single_arm.can_setup.nero_can_activate
```

第二条是独立只读核对。`--enable` 不负责回 S1。若是 `EMERGENCY_STOP`，按文末“急停恢复”操作，先不要使能或运动。

## 2. 到实验起点 S1

先只读查实时状态：

```bash
python -m experiments.single_arm.lab.cli status
```

若七轴已使能、状态 `NORMAL`，且关节角接近 S1 配置，可用下面的示教只读检查确认；输出“当前位置接近 ... S1”才表示已在起点，不用再发回位目标：

```bash
python -m experiments.single_arm.teaching.teach_session
```

**在 H0 运行这条示教检查会报“偏离安全起点”，这是预期拒绝，不表示机械臂已故障。**从 H0 或其他已建模姿态到 S1，统一运行：

```bash
python -m experiments.single_arm.lab.cli quick init --target S1
```

已标定 H0 无论当前七轴使能还是失能，都会自动选对应的 H0→S1 专用路线；失能状态只在该路线中单次使能。命令先做只读预检，打印目标和报告；现场确认桌边、连杆、腕部、线缆及支撑范围后，在终端键入“开始”才执行。若输出 `already_at_target: true`，无需运动。到达 S1 后再运行上面的 `teach_session` 只读检查。

**非 H0 低位历史试验：**2026-09-27 的窄姿态候选曾按 J3、J1、J2 分段规划并报告目标到位。当前 `quick init --target S1` 对匹配起点仍可给出候选，但不会执行低位前缀或后段。该试验前缀的局部 X/Y 数值只标记当时的候选条件；后段不再凭 `0.15 m` 假设取得许可。完整连杆、桌边、支撑和线缆的适用证据仍需核定。

历史首个 J3 目标与其后完整目标发布记录见实验报告。`--complete-low-route` 仍存在于旧 CLI，但本轮产生的 `candidate_only` 计划在其动作前被拒绝；旧计划也不能按新语义重放。这些历史结果不能替代当前环境与路线适用证据。

H0 专用路线重新连接时，若**只有**一次七轴停稳采样不通过，会在同一入口内有限次只读复测；报告保存每次关节变化。复测期间姿态改变超过 0.002 rad、持续不稳或出现其他故障时仍会在发送运动前退出。失败后先看报告中的 `completed_targets` 和实时状态，不直接反复输入“开始”。

若要把**命名目标规划**分开查看路线，可先保存计划，再用同一路径执行；H0→S1 会自动选专用路线：

```bash
python -m experiments.single_arm.lab.cli starts plan --target S1
python -m experiments.single_arm.lab.cli starts run --plan <上一步输出的plan_path>
```

这里的计划文件只能配合刚才的实时起点使用。从 S1 回桌边也可将 `--target S1` 改为 `--target H0`；执行前仍需现场确认整片范围。非 H0 的通用回 S1 计划目前仅是候选，分离命令与交互命令都不能执行它。其他桌面接触位、急停、未知安装条件下若被拒绝，保持现场姿态并根据实时情况处理。

从实时姿态回 S1 的通用入口目前只可查看候选，不读原轨迹做倒放：

```bash
python -m experiments.single_arm.lab.cli return plan
```

输出的 `execution_status=candidate_only` 和 `environment_assessment=not_evaluated` 表示未授予执行许可。可另给有来源的 `--min-flange-x-m <米数> --flange-x-source <来源>` 做**候选**筛选，负数本身合法；该输入也不能解锁 `return run`。

## 3. 只读记录当前位置

```bash
python -m experiments.single_arm.lab.cli poses --duration 5 --rate 5
```

程序会打印新的 CSV 路径。它只读反馈，**不会开启拖动模式**；关节使能时手推不动是正常的。需要精确诊断时也可用原入口 `python -m experiments.single_arm.pose_recording.record_poses --duration 5 --rate 5`。

## 4. S1 限时示教

先只读预检，确认输出允许在 S1 开始：

```bash
python -m experiments.single_arm.lab.cli teach --max-seconds 5
```

以下交互动作命令为历史步骤；当前会在进入拖动前因返回路线依据未核定而拒绝：

```bash
python -m experiments.single_arm.lab.cli quick teach --max-seconds 5
```

看完预检后键入 `开始`。进入拖动后先保持静止约 0.5 秒，再在已清空的空间内缓慢示教，5 秒后程序退出拖动并从**实际结束姿态**规划 `move_j` 回 S1。它保存 CSV 与同名 JSON，并打印记录 ID；回位用实时状态新规划，**不倒放录制轨迹**。另一种直接入口是 `python -m experiments.single_arm.lab.cli teach --max-seconds 5 --run`，它没有交互确认。

2026-09-26 的 5 秒实机试验验证了 S1 模式切换和基本静止时的零目标收尾；真正拖动后自动回位仍须按当次计划、反馈和现场环境核对。低位、H0、无时限手动拖动不允许用这个入口开始。详细限制见[示教实验](experiments/single_arm/teaching/README.md)。

查找刚完成且元数据完整的新记录：

```bash
python -m experiments.single_arm.lab.cli records
python -m experiments.single_arm.lab.cli records latest
```

若示教停在非 S1 且状态正常，回到第 2 节从实时姿态新规划；若预检失败或设备异常，先查状态和报告，不启动回放。`latest` 检查最近一次示教尝试：该次失败就直接报告原因，不会改选旧记录；要回放较早的完整记录须显式写记录 ID。旧 CSV 需按[旧轨迹回放说明](experiments/single_arm/replay/README.md)单独处理。

## 5. 回放新示教轨迹

确认机械臂位于该记录的起点 S1、七轴使能、路线范围清空。交互运行：

```bash
python -m experiments.single_arm.lab.cli quick replay --recording latest
```

程序会让你选 `1` 连续 `move_j`（20 Hz）或 `2` 逐点 `move_j`；直接回车选连续。它显示预检、总时长与法兰范围，再键入 `开始` 执行一轮正向及返程。旧逐点方式每个小步等待到位，会有明显停顿；连续方式的实际时间倍率由限速决定，不要把 20 Hz 误认为快速运动。

想先保存计划、再看报告路径后执行，可选下列**其中一种**：

```bash
# 定时连续方式：推荐用于刚封存的完整新记录
python -m experiments.single_arm.lab.cli replay plan --recording latest --mode continuous --frequency-hz 20 --min-time-scale 6
python -m experiments.single_arm.lab.cli replay run --plan <上一步输出的plan_path>
```

```bash
# 逐点方式：每一步到位后再发下一步
python -m experiments.single_arm.lab.cli replay plan --recording latest --mode point --speed-percent 5
python -m experiments.single_arm.lab.cli replay run --plan <上一步输出的plan_path>
```

两种执行方式都在运行前重新核对状态和起点。一次做“示教 → 选择回放”也可用 `python -m experiments.single_arm.lab.cli quick demo --max-seconds 5`；它会分别询问示教和回放确认。

## 6. 实验结束：保持 S1 或回桌边 H0

如果还要继续实验，停在 S1 并保持使能即可；核对：

```bash
python -m experiments.single_arm.lab.cli status
python -m experiments.single_arm.teaching.teach_session
```

若要**关机回家**，第一步把法兰放到已验收的轻触桌边 H0 候选位。从 S1、`NORMAL`、七轴使能出发，运行统一命名目标入口；预检后键入“开始”才运动：

```bash
python -m experiments.single_arm.lab.cli quick init --target H0
```

本入口保留高位 Y 负向绕开桌边、最后用低速单 J1 贴边；到位后**仍保持使能**。减小 J3 偏转的 7 目标路线已双向实机验收。到位数值不能替代现场检查：必须实际看到法兰轻触、承托可靠、无中途擦碰和线缆受拉。它只接受已标定的 S1/H0 和当前左侧安装，不能从任意位置直接回家。

**第二步才是失能关机。**先确认 H0 法兰轻触桌边、实体承托可靠、夹点和可能下沉范围清空，再单独执行一次失能入口：

```bash
python -m experiments.single_arm.lab.disable_at_h0_candidate
python -m experiments.single_arm.lab.disable_at_h0_candidate --run
python -m experiments.single_arm.can_setup.nero_can_activate
```

第三条只读核对七轴失能。此脚本仅接受它标定的轻触候选角度；未到位或支撑不可靠时应拒绝，不要使用电子 `reset` 代替正常停机。一次 H0 失能及之后的 H0→S1 启动已有现场验收，但仍是当前安装上的**有人监护试验**，不是无人值守开关机。

## 7. 其他实验命令索引

以下都是**独立实验**，不需要按表格顺序全部做。每次先用第 1 节查状态，再按各实验 README 的起点限制操作；只读预检的成功不代表实体桌面和线缆已经清空。

| 实验 | 只读/离线命令 | 执行命令及停止位置 |
| --- | --- | --- |
| 固件、驱动、电压 | `python -m experiments.single_arm.can_setup.detect_firmware --channel can0`；`python -m experiments.single_arm.can_setup.nero_can_activate` | 仅需使能时用 `nero_can_activate --enable`；见[CAN 说明](experiments/single_arm/can_setup/README.md) |
| 位置采样 | `python -m experiments.single_arm.lab.cli poses --duration 5 --rate 5` | 只读，不拖动；见[采样说明](experiments/single_arm/pose_recording/README.md) |
| S1→C2 命名姿态 | `python -m experiments.single_arm.lab.cli starts plan --target C2` | `python -m experiments.single_arm.lab.cli starts run --plan <新plan_path>`，停在 C2；见[统一接口](experiments/single_arm/lab/README.md) |
| 控制器 IK 直线 `move_l` | `python -m experiments.single_arm.control_interfaces.cartesian_linear_session --dx-m -0.002` | 同一命令加 `--run`，停在新点；见[法兰控制接口](experiments/single_arm/control_interfaces/README.md) |
| 控制器点位 `move_p` | `python -m experiments.single_arm.control_interfaces.cartesian_point_session --dx-m -0.002` | 同一命令加 `--run`，停在新点；控制器中间路径未知，见[法兰控制接口](experiments/single_arm/control_interfaces/README.md) |
| 本机 IK 法兰 X 往返 | `python -m experiments.single_arm.cartesian_x.cartesian_x_session --distance-m 0.02` | `python -m experiments.single_arm.cartesian_x.cartesian_x_session --plan <新plan_path> --run`；见[X 方向实验](experiments/single_arm/cartesian_x/README.md) |
| 小段直线探测 | `python -m experiments.single_arm.lab.cli probe plan --axis x --delta-m 0.002` | `python -m experiments.single_arm.lab.cli probe run --plan <新plan_path>`，停在新点；见[可达性实验](experiments/single_arm/reachability/README.md) |
| 局部 J4 往返 | `python -m experiments.single_arm.local_probe.local_joint_probe --speed-percent 5` | 同一命令加 `--run`；仅适用当前 J4 正角，S1 通常不满足；见[局部对照](experiments/single_arm/local_probe/README.md) |
| 整数度外观候选 | `python -m experiments.single_arm.lab.cli pose-candidate plan` | `python -m experiments.single_arm.lab.cli pose-candidate run --plan <新plan_path> --stage 1`；只作外观试验，不改公共 S1 |
| 固定旧 20 秒源连续回放 | `python -m experiments.single_arm.lab.cli continuous-archive plan --frequency-hz 20` | `python -m experiments.single_arm.lab.cli continuous-archive run --plan <新plan_path>`；仅固定旧源，见[连续回放](experiments/single_arm/continuous_replay/README.md) |
| 可达地图 | `python -m experiments.single_arm.lab.cli map`；`python -m experiments.single_arm.reachability.analyze_probes`；`python -m experiments.single_arm.reachability.build_map` | 都是数据读取或离线生成，见[可达性实验](experiments/single_arm/reachability/README.md) |

历史 20 秒示教 CSV 如需复核，只读规划用 `python -m experiments.single_arm.replay.replay_session --recording experiments/single_arm/teaching/data/recordings/nero_session_20260924T101130Z.csv --cycles 1 --speed-percent 5`。旧脚本加 `--run` 会实机回放；它不属于新手主流程，须先独立确认 S1、记录完整、法兰及所有连杆活动范围。旧连续回放报告只读分析可用 `python -m experiments.single_arm.continuous_replay.analyze_run <报告JSON路径>`。驱动状态的长时只读监测可用 `python -m experiments.single_arm.can_setup.monitor_state --duration 300 --rate 2`，会生成状态和本机控制帧 CSV。

旧日期的 `point_workspace_demo` 宽范围计划曾发生法兰触桌；不要把旧计划加 `--run` 重试。`move_p` 更适合 VLA 法兰点位运行，本工作区的启动、回 S1、回家仍用 `move_j` 验收七轴姿态。

### 轨迹极值与立方体候选

已有**正常结束的实测拖动 CSV**时，可离线提取六轴向极值的内缩立方体候选：

```bash
python -m experiments.single_arm.lab.cli cube extrema --recording <实测拖动CSV路径> --margin-m 0.01 --bottom-margin-m 0.03
```

也可从已有分级地图读取：

```bash
python -m experiments.single_arm.lab.cli cube extrema --map experiments/single_arm/workspace_cube/data/candidates/nero_drag_map_20260926T072641370333Z.csv --level new_hand_guided_sweep --margin-m 0.01 --bottom-margin-m 0.03
```

这只是**几何候选**，不等于六面实物标定或可运动空间。六面会话的 `cube new`、`cube capture --session <路径> --face +X`、`cube status --session <路径>`、`cube fit --session <路径>`只建立/读取数据；必须有真实、有效的六面点位才可能拟合。详细见[立方体实验](experiments/single_arm/workspace_cube/README.md)。

### 目前不要运行的旧入口

- `quick cube-sweep`、`quick cube-faces`、`quick home-demo` 和 `can_drag_teach --start`：低位拖动切换曾出现突然大幅上移，当前程序锁定这些启动入口。离线 `cube extrema` 仍可用。
- 旧 `supported_home` H0 接近路线：曾发生桌边擦碰，仍被锁定。`quick park --target H0` 现在只是统一命名目标入口的别名，使用已验收的新 H0 路线。
- 历史 `point_workspace_demo --run` 和旧日期计划：不能推定对当前姿态与桌面安全。每个历史目录的报告保留用于复盘。

## 8. 急停、中断与找报告

若出现急停、碰撞、突然下落、异常上抬或脚本报错：先停下并支撑机械臂，查看现场和只读状态，不继续运动。运动脚本可能已请求阻尼电子急停；**七轴驱动位显示 True 也不能覆盖整机 `EMERGENCY_STOP` 状态**。

```bash
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.can_setup.recover_emergency_stop
```

第二条只读观察 5 秒。仅当确实处于电子急停、机械臂有可靠承托、可能下落范围及夹点清空时，才由现场人员单次执行：

```bash
python -m experiments.single_arm.can_setup.recover_emergency_stop --run
python -m experiments.single_arm.can_setup.nero_can_activate
```

`reset` 后通常是 `JOINT_BRAKE_NOT_RELEASED`、七轴失能；确认姿态稳定后再按第 1 节单次使能，**重新读取实时姿态并生成新路线**。详细见[电子急停恢复](docs/emergency_stop_recovery.md)。

终端输出的 CSV、JSON、计划和报告路径都相对于工作区根目录。新示教文件在 `experiments/single_arm/teaching/data/recordings/`；实验报告分别在各目录的 `data/` 中。运行后找不到文件，先按终端打印的**完整路径**打开，不要猜文件名。历史整套 `unittest discover` 命令尚未被证实为零设备副作用的离线入口；执行测试前按[测试分级与逐文件审计](docs/test_tiers_audit.md)选择已核实的子集和解释器，记录实际运行结果。当前项目续接见[CURRENT_STATE.md](CURRENT_STATE.md)。

各实验的实现、证据和历史限制见[单臂实验总览](experiments/single_arm/README.md)、[统一 API/CLI](experiments/single_arm/lab/README.md)与[厂家用户指南](docs/user_guide.md)。
