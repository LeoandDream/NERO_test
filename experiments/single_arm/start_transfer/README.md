# 从当前位置自动移动到 S1

本目录保存已执行的关节恢复方法和记录。2026-09-25 的[两目标实机回位](report.md)已完成：每段向控制器发送一次完整 `move_j`，最终只读检查确认回到 S1。下文带时间戳的计划对应**当时的起点**，现在已执行完毕；不要复制旧计划路径再加 `--run`。S1 附近单次法兰直线 `move_l` 见[控制器 IK 实验](../control_interfaces/README.md)。从远处或急停下垂姿态直线回 S1 尚未验证。

## 当前临时起点至 S1 的两目标离线比较

2 mm `move_l` 往返结束后，运行报告中的临时起点与历史 S1 相距约 `0.915 rad`（关节空间）。若直接给控制器一个 S1 七轴目标，假定关节线性插值时 J2/J4 会在同一区域接近零。为了比较少量完整动作，新增**只读、没有 `--run` 入口**的[两目标几何分析](plan_two_target_s1.py)：

```bash
python -m experiments.single_arm.start_transfer.plan_two_target_s1
```

设备重连、复位或重新使能后，应另采集一份已停稳且状态为 `NORMAL` 的只读 CSV，再以 `--source-recording 该文件.csv` 生成新的离线比较；脚本会拒绝采样期间关节变化超过 `0.002 rad` 的记录。旧临时起点和旧规划不能充当实时反馈。

它读取已完成的 2 mm 回程报告，先比较一个直接关节目标，再比较两个关节目标。候选中间姿态让 J2 保持正弯、J4 先弯到 S1、J7 对齐 S1；下一目标才到 S1。每段只设想一条控制器 `move_j` 目标，不会发布密集小步。离线结果约为 `0.648 rad` 和 `0.779 rad` 两段；`min(max(|J2|, |J4|))` 由直接路线的 `0.022 rad` 增至两目标路线的 `0.079 rad`。两条路线都只按**假定的关节线性插值**计算，没有证据表明控制器实际会精确这样走。另做的离散裸臂网格检查没有发现非相邻连杆相交，但也不能代替控制器实际路径或现场障碍物检查。输出 JSON 标记 `executable=false`，**不能直接作为运动命令**；专用执行入口还要求同源网格审计、实时起点和驱动预检。

几何脚本使用固定版本的[厂商 URDF](geometry/README.md)计算各连杆原点范围，并对每个采样点核对 `link7` 与本机 SDK 的法兰 FK。它不检查连杆实体体积。当前设备是否仍处于该历史临时起点，须由现场重新只读确认；跨日记录不能代替实时反馈。

另对同一固定版本的厂商 STL 碰撞网格做了[离线采样初筛](geometry/mesh_audit_2026-09-25.md)：在直接路线的 101 个和两目标路线的 202 个假定插值姿态中，非相邻连杆网格均未相交，采样最小网格间距约 `12 mm`。这只检查机械臂裸臂模型的离散姿态；还缺少真实控制器路径和现场物体间隙，单独的候选文件不能用来执行。

2026-09-25 USB-CAN 重新接入并使能后，以[新静止记录](../pose_recording/data/recordings/nero_poses_20260925T065242Z.csv)重新生成[两目标比较](data/plans/nero_two_target_offline_20260925T065303Z.json)及[裸臂网格检查](data/diagnostics/nero_mesh_audit_20260925T065335Z.json)。起点约 `[0.122, 0.089, 0.056, 0.083, −0.017, 0.212, −1.041] rad`，已明显不同于前次锚点。两个假定完整关节目标的距离约 `0.693`、`0.787 rad`；采样姿态没有发现非相邻裸臂网格相交，最小间距仍约 `12 mm`。新结果同样标记 `executable=false`，不代表控制器实际轨迹、现场支撑和线缆安全。

为逐段验证控制器内部关节运动，新增[单目标回位入口](controller_joint_return.py)。每次运行最多发送一个 `move_j` 完整目标；`--stage 1` 停在中间姿态，下一次必须重新核对现场后才可使用 `--stage 2` 回 S1。先运行只读预检：

```bash
python -m experiments.single_arm.start_transfer.controller_joint_return \
  --plan experiments/single_arm/start_transfer/data/plans/nero_two_target_offline_20260925T071016Z.json \
  --audit experiments/single_arm/start_transfer/data/diagnostics/nero_mesh_audit_20260925T071353Z.json \
  --stage 1
```

这条带日期的命令仅展示当时的只读预检输入，不是可重复执行的回位步骤。若机械臂已在 S1，先运行 `python -m experiments.single_arm.teaching.teach_session` 核对当前位置；无需再次回位。

脚本只读预检检查实时七轴位置是否与该段起点相差不超过 `0.005 rad`、控制器及驱动是否正常，并复核目标、固定版网格审计和采样关节限位。控制器实际运动与假定关节插值可能不同；网格审计不包含现场障碍、支撑、线缆或工具，不能作为自动放行依据。出现停滞、故障、急停或位置偏差时，检查阶段报告和现场状态，不能直接重复运行。

**2026-09-25 完整目标试验：**第一次第 1 段控制器开始运动后各关节进度不同步，约 1 秒时触发旧的 `0.06 rad` 关节直线偏差门禁，并请求电子急停。复位、使能和新姿态采样后，监控改为检查每个关节是否留在本段起终点区间附近。新起点的[规划](data/plans/nero_two_target_offline_20260925T071016Z.json)和[第一段审计](data/diagnostics/nero_mesh_audit_20260925T071353Z.json)重新生成；审计抽查第 1 段关节不同步范围的 1,331 个裸臂姿态，没有发现非相邻网格相交。16 项相关离线测试通过。[第一段](data/runs/nero_controller_joint_return_20260925T072543Z.json)用单个目标到达中间姿态，最大关节误差 `0.000175 rad`。[第二段审计](data/diagnostics/nero_mesh_audit_20260925T072932Z.json)抽查 3,125 个姿态后，[第二段](data/runs/nero_controller_joint_return_20260925T074034Z.json)用单个目标回到 S1，最大关节误差 `0.000157 rad`。两段现场均报告平稳无异常；`teach_session` 独立只读检查确认当前位置接近 S1。网格审计只涵盖离散裸臂姿态，不包括现场物体、支撑、线缆或所有连续姿态。

## 2026-09-24 长回放再次急停后的恢复

10% 长示教回放在 422/644 小步后，第 423 步目标调用未产生预期运动，脚本发送阻尼急停。复位后重新使能并只读确认 `NORMAL`、七轴使能；实时关节角约 `[0.1300, 0.2566, -1.2248, -0.3176, -0.0542, 0.2056, -0.4688] rad`。该姿态与下文两种旧恢复计划的起点不同，旧计划不可复用。

新增 `after-replay-sag` 策略：从这类实测姿态先分两段展开 J3，保持 J4 弯曲后分两段使 J2 回 S1，最后对齐余下关节。只读生成命令：

```bash
python -m experiments.single_arm.start_transfer.recover_to_s1 --strategy after-replay-sag
```

本次[新计划](data/plans/nero_recover_s1_after_replay_20260924T133804Z.json)分 5 段、共 47 小步；每段必须以实时反馈重新核对起点和沿途法兰范围。程序只允许从本次急停后的 J2/J3/J4/J7 姿态区间生成该策略。单步未执行时，只有再次确认控制器正常且关节稳定停在上一目标附近，才暂停而不触发阻尼急停；计划随即标记失败，不能继续旧文件。任何状态异常或姿态漂移仍触发急停。法兰预测范围无法排除其他连杆、线缆、支撑或现场物体的碰撞。

每次只执行计划的下一段，现场核对本段全部连杆扫过范围后使用：

```bash
python -m experiments.single_arm.start_transfer.recover_to_s1 --plan experiments/single_arm/start_transfer/data/plans/nero_recover_s1_after_replay_20260924T133804Z.json --run
```

每段结束后检查误差、状态与现场，再决定是否执行下一段。

## 急停恢复后从当前位置分段回 S1

2026-09-24 长示教回放急停后的当前位置距离 S1 约 `1.37 rad`，超过下文单次转移脚本的 `0.8 rad` 上限。恢复、使能及 5%/10% 局部往返已完成，状态检查为 `NORMAL`、七轴使能。此时使用 [分段恢复脚本](recover_to_s1.py)。每次运行仍以**实时反馈**检查起点；旧采样只是离线设计依据。

SDK 的 `get_ik_joint_angles()` 是 `move_p`、`move_l`、`move_c` 下发后的逆解**反馈**，不是可在运动前调用的只读 IK 规划器。S1 的七个目标关节角已经实测并存入[配置](../config/nero_teach.json)，因此使用关节空间分段规划，并用 `fk()` 计算沿途法兰中心位置。

路线按顺序调整 J4、分两段调整 J2、分两段调整 J7，最后对齐其余关节。J2 过零位前先将 J4 弯至 S1 角。每段关节空间距离不超过 `0.8 rad`，小步不超过 `0.05 rad`。脚本只允许从本次恢复姿态的 J2/J4 区间出发；实时姿态不符即拒绝规划。法兰范围只覆盖**法兰中心**，不能判断连杆、手腕和现场物体是否碰撞。配置中的法兰工作空间边界目前为 `null`，未启用空间边界保护。

先在工作区根目录只读生成计划；无 `--run` 不会下发运动：

```bash
python -m experiments.single_arm.start_transfer.recover_to_s1
```

检查输出中实时起点、各段目标、小步数和法兰范围；保存打印出的 `nero_recover_s1_*.json` 路径。然后用实际生成的路径只读复核：

```bash
python -m experiments.single_arm.start_transfer.recover_to_s1 --plan experiments/single_arm/start_transfer/data/plans/nero_recover_s1_实际时间.json
```

现场确认本段**所有连杆和腕部**扫过范围清空、底座固定且有人监护后，才运行同一计划的下一段：

```bash
python -m experiments.single_arm.start_transfer.recover_to_s1 --plan experiments/single_arm/start_transfer/data/plans/nero_recover_s1_实际时间.json --run
```

该命令只执行**下一段**，默认速度 5%，有 5 秒倒计时；完成后把结果写回计划。逐段检查状态、现场和计划文件，再运行同一命令进入下一段。未完成上一段、当前姿态偏离下一段起点、故障或失能时都会拒绝继续。中途异常可能触发电子急停，机械臂可能失去位置保持；须按[急停恢复文档](../../../docs/emergency_stop_recovery.md)处理，不要重复运行旧计划。此前 10% 成功只适用于 `0.05 rad` 局部对照，不能据此认定本路线可用 10%。

本次计划的第 4 段 J7 首步超时，计划标记 `failed=true`。急停复位、重新使能后，可运行只读 [J7 诊断脚本](diagnose_j7.py)：

```bash
python -m experiments.single_arm.start_transfer.diagnose_j7
```

它连续读取 J7 驱动与电机反馈、整机状态、七轴角度，并保存到 `data/diagnostics/`。该命令不发送运动、复位或使能指令。诊断结果不能单独证明 J7 运动正常；故障原因查明前不要把原计划改为更高速度重试。

J7 诊断显示使能、无驱动故障或堵转标志，当前位置约 `+0.449 rad` 且静止。原计划中 J7 单独改变时首步未动；较早成功的去 S1 计划中，J7 曾与其他关节同时改变。这提示需要区分“单独 J7 控制未触发”与 J7 本体故障，目前只是待验证假设。[J4/J7 微小联动测试](probe_j7_coupled.py)默认只读规划：

```bash
python -m experiments.single_arm.start_transfer.probe_j7_coupled
```

它只在当前 J2/J4/J7 落在本次恢复姿态的限定区间时规划：J4 和 J7 各负向 `0.01 rad`，其他五轴目标不变。需要现场重新核对支撑、线缆及关节活动范围后，才可考虑显式 `--run`。测试成功会停在新姿态；若发出运动后首步仍超时，共用执行器会请求电子急停，机械臂可能再次改变姿态。因此不要把只读规划输出视为执行许可。

本次微小联动单步已完成，J4 与 J7 均实际改变，最大关节误差约 `0.000854 rad`。这支持继续**只读**生成新的联动回 S1 计划：

```bash
python -m experiments.single_arm.start_transfer.recover_to_s1 --strategy coupled-j7
```

新策略从当前实测姿态分 4 段回 S1，每个小步同时改变 J7 与至少一个其他关节；每步关节空间距离不超过 `0.025 rad`，到位容差收紧为 `0.003 rad`。先弯 J4，然后两段移动 J2，最后对齐其余关节。新计划保存在 `nero_recover_s1_coupled_*.json`，与已失败的旧计划区分。它仍只计算法兰中心范围，不能证明连杆和腕部无碰撞，也不能保证联动大行程一定成功。执行时使用新计划路径和同一脚本的 `--plan ... --run`，每次只执行下一段；每段都要先看现场及状态。

## 原有单次转移脚本

### 2026-09-25 桌面接触后的单次抬升

宽范围 `move_p` 第 7 段运动中法兰碰到桌面，旧演示计划已锁定，不能继续。现场确认此左侧安装下**基座 X 正向是离桌方向**。急停复位、稳定性检查和单次使能已完成；从[失能后稳定记录](../pose_recording/data/recordings/nero_poses_20260925T143752Z.csv)出发，新增[单次抬升脚本](lift_from_table.py)：只让 J2 从约 `+0.147` 减至 `+0.097 rad`，其他关节目标保持实时值。离线 FK 预测法兰基座 X 增加约 `28.7 mm`，共 51 个采样姿态，X 单调增加；厂商裸臂网格在这 51 个姿态未见非相邻连杆相交，最近间距约 `12.1 mm`。这些计算没有桌面模型，也不能证明控制器实际路线或线缆间隙。

在工作区根目录先运行只读预检：

```bash
python -m experiments.single_arm.start_transfer.lift_from_table
```

脚本核对 `NORMAL`、七轴使能、实时关节反馈和这份低位记录；如起点偏离超过 `0.005 rad` 会拒绝。现场再次确认法兰离桌方向、支撑和扫过范围后，才运行单次 `--run`：

```bash
python -m experiments.single_arm.start_transfer.lift_from_table --run
```

它只发一个 5% 的 `move_j` 目标，成功后停在抬升姿态，不自动回 S1，也不会解锁旧 `move_p` 演示。报告写入 `data/runs/nero_table_lift_*.json`。若已抬升或当前姿态改变，旧起点锁会拒绝重跑；应重新采样、重新规划。

本次[抬升报告](data/runs/nero_table_lift_20260925T144728Z.json)显示命令数 1、`completed=true`，J2 到约 `+0.0971 rad`，最大关节误差 `0.000192 rad`；现场确认法兰已离开桌面。从这份抬升报告到 S1 另有[两目标回位脚本](return_after_table_lift.py)，默认只读：

```bash
python -m experiments.single_arm.start_transfer.return_after_table_lift --stage 1
```

第 1 段仅把 J2 移至 `−0.2 rad`，增加离桌间隙；第 2 段从该中间姿态对齐 S1。离线假定关节插值的法兰 X 分别为 `0.007～0.175 m` 和 `0.175～0.382 m`，202 个裸臂网格采样未见非相邻连杆相交，最近约 `12.1 mm`。两段实机均已分别执行，报告最大关节误差为 `0.000102` 与 `0.000314 rad`；独立只读 `teach_session` 随后确认当前位置接近 S1。脚本每段核对实时起点、状态、驱动和预测法兰方向，每次 `--run` 最多发送一个 `move_j`。桌面模型尚未建立，不能把法兰 X 范围或裸臂网格结果当作环境安全证明。

使用 [去起点脚本](go_to_start.py)时，先从机械臂**实时关节反馈**生成一份规划，再复核并执行。同一次重启前的关节读数或规划文件不能代替实时检查。此功能用于从已确认的当前位置移动到 [默认配置中的 S1](../config/nero_teach.json)，不适用于从未知位置直接回零。

在工作区根目录执行只读规划：

```bash
conda activate nero-py310
ip -details link show can0
python -m experiments.single_arm.start_transfer.go_to_start
```

重启后若 USB-CAN 已连接、`can0` 存在但为 `DOWN`，需由本机用户在终端输入自己的管理员密码启用 1 Mbps CAN 接口：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

脚本检查七个驱动反馈、故障状态、关节角稳定性，以及 SDK 正运动学与实测法兰位置是否一致。常规情况下从当前七关节角到 S1 做关节空间直线插值，总关节空间距离上限为 `0.8 rad`。若当前位置已经落在既有的全零位禁用半径内，只有在 J4 已为负角且单独向 S1 的 J4 角度移动能够**单调远离全零位**时，才规划“先弯曲 J4、再去 S1”的两段路线；两段总长度上限为 `1.2 rad`。这只是对已处于近零姿态的有限脱离策略，不能证明周围空间安全。每小步不超过 `0.05 rad`，检查 SDK 关节限位，离开近零区域后恢复全零位排除规则；沿途每约 `0.01 rad` 计算法兰中心范围。规划保存在 `experiments/single_arm/start_transfer/data/plans/nero_to_start_*.json`，包含实时起点、目标、全部小步与法兰范围。只读规划不发送运动指令。

复核规划文件时：

```bash
python -m experiments.single_arm.start_transfer.go_to_start --plan experiments/single_arm/start_transfer/data/plans/nero_to_start_实际时间.json
```

脚本会拒绝已经执行、被修改或与当前安装方向、S1、工作空间配置不符的规划，也会拒绝当前位置偏离规划起点超过 `0.005 rad` 的情况。**规划使用关节空间插值，并未验证中间姿态下所有连杆与台面、夹具、人员或障碍物的间隙。** 配置中的 `flange_workspace_m` 当前为 `null`，法兰工作空间保护尚未启用；输出的法兰范围只是计算结果，并非现场安全边界。

确认底座固定、安装方向与零点正确、整条运动路径无障碍且有人现场监护后，才执行对应规划：

```bash
python -m experiments.single_arm.start_transfer.go_to_start --plan experiments/single_arm/start_transfer/data/plans/nero_to_start_实际时间.json --run
```

执行前有 5 秒倒计时。脚本切换到 CAN 关节控制，以 `5%` 速度逐小步下发 `move_j`，每步核对实测关节角、故障和驱动状态。按 `Ctrl+C` 可中止；已经发送运动指令后出现偏离、超时或其他异常时，脚本调用 SDK 电子急停。成功到达后，规划文件记为已执行，不能重复使用。

## 2026-09-24 实际执行

`can0` 恢复 1 Mbps 通信后，机械臂曾处于 `JOINT_BRAKE_NOT_RELEASED`、七个驱动未使能，关节姿态接近全零位。第一份[候选规划](data/plans/nero_to_start_20260924T082654Z.json)只读生成后，当前位置发生变化，复核时超过 `0.005 rad` 起点容差，因此**拒绝执行**。

重新读取姿态后生成[实际执行规划](data/plans/nero_to_start_20260924T083116Z.json)：24 个小步，先单独弯曲 J4，随后移动到 S1；法兰中心预测范围 `x=0.0004～0.3820 m`、`y=−0.0230～−0.0141 m`、`z=0.5724～0.7159 m`。使能后控制器反馈 `CAN_CTRL`、`NORMAL`、错误码 `0x0000`，七个驱动均使能；规划起点复核仍吻合。现场确认整片活动范围清空并准备监护后，脚本以 5% 速度完成 24/24 小步，规划文件标记 `executed=true`。独立只读核对：与 S1 的最大单关节误差约 `0.0000175 rad`，法兰中心 FK 约为 `(0.382055, −0.014080, 0.572436) m`，控制器保持 `NORMAL`、无故障码、七个驱动均使能。

此次验证的是这条具体关节路线。`flange_workspace_m` 仍为 `null`，法兰边界保护未启用；下一次从其他姿态出发必须重新规划并检查现场空间。
