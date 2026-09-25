# 关节目标与两种法兰目标发布接口

本目录将“要到哪里”与“向控制器发什么目标”分开。[发布器](publishers.py)提供三种 SDK 命令：

| 接口 | 输入 | SDK 下发 | 当前用途 |
| --- | --- | --- | --- |
| `JointTargetPublisher` | 7 个关节角，rad | `move_j` | 示教回放、分段回 S1、已验证的笛卡尔 X 本机逆解 |
| `CartesianPointPublisher` | 法兰 `[x,y,z,roll,pitch,yaw]`，m/rad | `move_p` | 控制器 IK 与点位运动；2 mm 去程和回 S1 已实机到位 |
| `CartesianLinearPublisher` | 法兰 `[x,y,z,roll,pitch,yaw]`，m/rad | `move_l` | 控制器 IK 与直线运动；临时锚点 2 mm 往返已实机到位 |

`move_p` 只指定法兰**终点**，SDK 不承诺中间法兰位置走直线；`move_l` 明确请求法兰直线。两者都由控制器选择逆解，因此同一法兰终点可能对应不同七轴姿态；回原关节姿态时还须核对关节反馈。`move_j` 输入的是已知七轴目标，不调用法兰 IK。官方[Nero 驱动源码](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/drivers/nero/default/driver.py)分别实现三种运动模式。

2026-09-24 成功的分段回 S1 **没有调用 SDK 的 IK**：脚本直接规划七个关节角，SDK `fk()` 只用于计算沿途法兰中心。现有[法兰 X 实验](../cartesian_x/README.md)则用本机数值逆解计算位置目标，最后仍通过 `move_j` 发布。SDK 的 `get_ik_joint_angles()` 只能读取 `move_p`、`move_l`、`move_c` 后控制器给出的逆解反馈，不能在运动前请求它生成一条可验证的关节路径。

[`cartesian_linear_session.py`](cartesian_linear_session.py)提供直线命令行实验。默认只读；`--run` 才会发送运动命令。它从实测关节与法兰姿态生成目标，以本机 FK/数值逆解对直线每 2 mm 采样、检查关节限位与变化，再**只发送一次 `move_l`**。本机预检不是控制器逆解的保证；运动中以实测法兰到目标线段的偏离、姿态变化、关节限位及关节相对起点的变化作门禁，本机 IK 关节折线仅记录作诊断，避免把控制器不同步的逆解进度误判为故障。发送前完整检查七轴驱动；运动中读取最新驱动故障位，暂时过期的驱动帧会记入运行报告。控制器已报告停稳时，偏离或未达目标会记录为失败，不再追加急停。当前 `flange_workspace_m=null`，因此只允许已核对起点附近不超过 30 mm 的直线，不能自动判断连杆、线缆或支撑碰撞。

2026-09-25 从 S1 做 X 负向 10 mm 的只读预检通过，预测最大单关节变化约 `0.124 rad`。随后[实机去程与回程](report.md)各发送一条 `move_l`，均到位并回到 S1，独立检查为 `NORMAL`、七轴使能，现场确认两程平稳。固定法兰姿态继续扩到 100 mm 时，本机近邻解的 J4 距 SDK 下限仅约 `0.041 rad`，因此不能直接把既有多步、允许姿态变化的 100 mm 实验改为单条 `move_l`。

10 mm 预检路线还用厂商裸臂网格离线抽查了 61 个姿态，未发现非相邻连杆相交；最近采样间距约 12 mm。[审计脚本](audit_linear_preview.py)仅使用 S1 配置和本机固定版网格，不连接机械臂。重复运行需已安装 `numpy`、`trimesh`、`python-fcl` 并保留[几何模型说明](../start_transfer/geometry/README.md)中的固定网格：

```bash
PYTHONPATH=/home/leo/.cache/nero_collision_audit/python:/home/leo/nero_dh116_ws \
  python -m experiments.single_arm.control_interfaces.audit_linear_preview \
  --dx-m -0.01 --mesh-dir /home/leo/.cache/nero_collision_audit/meshes
```

离线网格没有检查控制器实际 IK 路线、现场支撑、线缆和台面，因此这份结果只供现场核对，不是自动执行许可。

[`cartesian_point_session.py`](cartesian_point_session.py)使用同一套状态、锚点和目标检查，改为**只发送一次 `move_p`**。首次试验仅允许法兰平移 1～5 mm；离线采样只是终点可达性的筛查，不是 P 模式真实路径。运动中记录法兰和关节反馈，只允许法兰相对起点不超过 30 mm、关节相对起点不超过 0.25 rad，并检查故障与到位状态。这些数值不能证明连杆和现场环境安全。2026-09-25 在 S1 附近完成了 X 负向 2 mm 的[去程](data/runs/nero_move_p_20260925T080436Z.json)与[回程](data/runs/nero_move_p_20260925T080758Z.json)，各发送一次 `move_p` 并到位；现场报告均平稳无异常，回程后独立只读检查为 `NORMAL`、七轴使能，且接近 S1。只读命令：

用户随后自行完成 X 负向 4 mm 的[去程和回程](report.md)，每程一条 `move_p`，法兰及七轴反馈到位。去程法兰相对起点最大平移约 7.7 mm，回程约 8.4 mm；两程采样均明显偏离连接起终点的直线。现场核对必须覆盖整个可能活动范围，不能只按 4 mm 终点位移留间隙。输入 `-0.04` 代表 40 mm，会在预检阶段被 5 mm 上限拒绝；距离单位为米。

## 多方向点位演示

[`point_workspace_demo.py`](point_workspace_demo.py)把历史示教记录用于选取五个法兰目标，分别测试侧向、X 负向、Z 负向及三轴组合。第一点直接取 2026-09-23 五秒示教的第 195 条六维法兰反馈；其余目标的三维位置均落在两份示教记录的**按轴观测范围**内，姿态保持 S1。按轴观测范围并不证明这些组合姿态或控制器中间路径安全；脚本另用本机近邻 IK 筛查每个目标。2026-09-24 的 20 秒记录曾在逐点回放中触发急停，这里仅用其位置范围选点，不重放该轨迹。

已生成的[离线计划](data/plans/nero_point_demo_20260925T134206Z.json)共有五个目标。每个目标用一次 `move_p` 去程，再用一次 `move_p` 回 S1，共 10 段；不会自动连续执行。按顺序是：实测 Y 正向约 37 mm、Y 负向 40 mm、X 负向 40 mm、Z 负向 30 mm、三轴组合约 37 mm。离线近邻 IK 最小关节限位余量分别约为 0.706、0.677、0.344、0.379、0.513 rad。固定版厂商裸臂网格在这些假设近邻 IK 路线上共抽查 480 个离散姿态，未发现非相邻连杆相交，最小采样间距约 12 mm；此审计**没有**覆盖控制器实际 `move_p` 轨迹、工具、线缆、台面或支撑。

第 1 段 37 mm 去程已于 2026-09-25 实机到位。用户希望进一步扩大展示范围，因此另生成[宽范围计划](data/plans/nero_point_demo_20260925T134828Z.json)。它包含 Y 负向 80 mm、20 秒示教中两个实测六维法兰姿态（距 S1 约 114 mm 和 93 mm）、X 负向 60 mm、Z 负向 45 mm。近邻 IK 最小限位余量分别约 `0.608/0.242/0.388/0.226/0.268 rad`；固定版裸臂网格对假设近邻路线共抽查 599 个离散姿态，没有发现非相邻连杆相交，最近采样间距约 12 mm。20 秒示教记录本身曾发生回放急停，宽范围计划只使用其中两个实测**目标位姿**，不采用当时失败的逐点回放路线，也不证明控制器 P 模式会按本机预测路线运动。

**2026-09-25 现场更新：两份多方向计划均已停用。**宽范围计划的 80 mm、114 mm、93 mm 目标完成往返后，第 7 段 X 负向运动时法兰先碰到桌面，随后控制器报运动失败，脚本发送电子急停。用户补充说明：左侧安装的默认低位法兰本就会接触桌面，需抬升才能离开；现场无人受伤，机械臂已支撑并停稳。[事件记录](report.md#2026-09-25-x-负向点位发生桌面接触实验停止)包含反馈和处理状态。脚本现于连接 CAN 前拒绝继续执行；以下命令只保留作历史流程说明，**不得在当前现场照抄运行**。后续须从急停后的实际低位重建抬升路径，并纳入桌面几何。

先检查状态及 S1，然后只读预检第 1 段：

```bash
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.teaching.teach_session
python -m experiments.single_arm.control_interfaces.point_workspace_demo \
  --plan experiments/single_arm/control_interfaces/data/plans/nero_point_demo_20260925T134206Z.json --leg 1
```

预检会重新读取实时起点、状态和七轴使能，并计算本段候选 IK 与反馈停止门槛。终端出现“只读预检完成”后，现场检查**整个机械臂、腕部、线缆和支撑可能扫过的范围**，再由现场人员在同一命令末尾加 `--run`。完成第 1 段并检查现场与运行报告后，用 `--leg 2` 只读预检返回 S1，再另行执行。此后依次使用 `--leg 3` 至 `--leg 10`；每段只发一个完整点位目标。任何一步未到位、状态异常或现场有接触，都停止后续段，不复用已失效的起点假设。

计划可在**不连接 CAN** 的环境重新生成：

```bash
python -m experiments.single_arm.control_interfaces.point_workspace_demo --build-plan
python -m experiments.single_arm.control_interfaces.point_workspace_demo --build-plan --profile wide
```

旧 `cartesian_point_session.py` 仍保留 1～5 mm 首次试验门槛。多方向计划使用逐段、独立的反馈门槛：按该段距离和本机 IK 关节变化设置较大的监测包络，防止把 4 mm 试验的门槛误用于 40 mm 运动。超出包络且控制器仍在运动时会发电子急停；因此现场必须有支撑和监护。若控制器已停稳但未到位，则记录失败，不追加急停。

```bash
python -m experiments.single_arm.control_interfaces.cartesian_point_session --dx-m -0.002
```

现场核对后才在同一命令末尾加 `--run`；成功后先只读运行 `python -m experiments.single_arm.control_interfaces.cartesian_point_session --to-s1`，核对目标和关节分支后才考虑回程 `--run`。每次命令仅发送一个点位目标；每次新实验仍须从实时姿态重新预检。

在工作区根目录运行，第一次只做 S1 附近 X 负向 2 mm 预检：

```bash
conda activate nero-py310
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --dx-m -0.002
```

确认控制器 `NORMAL`、七关节使能，核对目标位姿、连杆扫过范围和现场支撑，然后再由现场人员显式执行：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --dx-m -0.002 --run
```

若成功停在新姿态，需要回 S1 时，先只读预检 `--to-s1`，核对后另行加 `--run`。回位仍然**只发送一个法兰目标**；脚本不会把它拆成多段 `move_j`。两条命令各自单独生成记录，保存在 `data/runs/`，包含执行期间的关节、法兰、控制器 IK 和运动状态采样，可用于检查实际连续性。临时锚点 2 mm 往返已在实机到位，详见[实验记录](report.md)；现场确认回程平稳，无异常。

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --to-s1
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --to-s1 --run
```

从急停后的大幅下垂姿态直接到 S1 的六维直线，离线预检未收敛；本脚本会在超过 30 mm 或不能求得连续近邻解时拒绝命令。用户要求的“不拆分回位”在**符合范围的起点**由单次 `move_l` 实现；任意姿态一键恢复仍需控制器轨迹能力和实机验证，不能绕过预检强行下发。旧[五段关节恢复记录](../start_transfer/report.md)保留为历史实测。

如果机械臂不在 S1，但现场确认当前姿态安全，可以选一份**末尾连续停稳**的采样 CSV 作为临时锚点，不修改 S1。2026-09-24 突然停机后得到的[25 条只读记录](../pose_recording/data/recordings/nero_poses_20260924T140821Z.csv)末 5 条稳定；需要先重新使能并独立确认实时姿态未改变。以该文件做 2 mm 外行程时，命令为：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --anchor-recording experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T140821Z.csv --dx-m -0.002
```

只读预检通过、现场范围清空后，同一条命令末尾加 `--run` 才执行。成功后，用**同一份 CSV** 先执行 `--to-anchor --anchor-recording ...` 只读预检，再加 `--run` 回到记录的法兰位姿。去程和回程各只发布一个 `move_l` 目标。回程还会核对七关节是否接近原姿态；法兰到位但控制器选了另一组关节解，会报告未完成。临时锚点要用实时关节反馈核对；若当前位置偏离记录，外行程会拒绝。机器人突然停机的原因尚未查明，因此实验时仍需现场支撑和监护。

三种接口共用的现场要求是：从实时反馈确认起点、安装方向和零点；检查全部连杆、工具、线缆和支撑；执行时检查反馈新鲜度、运动状态、目标误差和故障码；只允许一个进程向 CAN 发布运动目标。当前 `JointTargetPublisher` 已接入[共享关节执行器](../start_transfer/go_to_start.py)。

离线接口测试：

```bash
python -m unittest experiments.single_arm.control_interfaces.tests.test_publishers
```
