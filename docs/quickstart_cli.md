# Nero 单臂实验：从零开始的命令行步骤

所有命令由现场人员在终端执行。先读每一步的输出，再决定是否进入下一步；运动实验须显式加 `--run`，使能命令须显式加 `--enable`。当前设备为 Nero 固件 1.21、左侧装、CAN `can0`，安全起点 S1 已记录在[配置](../experiments/single_arm/config/nero_teach.json)。更换安装方向、机械臂或零点后，不能直接复用 S1。

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

`-0.002` 表示沿基座 X 轴负方向平移 2 mm，法兰方向保持不变。输出须包含当前/目标六维位姿、预测关节变化及“未发送运动指令”。本机 IK 只用于检查可能的连续解；运动时控制器自行计算 IK 并执行直线。此脚本实际只发送一个 `move_l` 目标，不会把回位拆成多个关节小步。当前工作空间边界为 `null`，因此**现场要检查连杆、线缆、台面和支撑**；法兰范围不足以排除碰撞。

只有在机械臂停稳、底座固定、安装/零点正确、整条运动范围清空并有人现场监护时，才执行：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --dx-m -0.002 --run
```

先确认完成信息和现场无异常，再只读预检回 S1：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_linear_session --to-s1
```

核对目标和活动范围后，才用同一命令加 `--run`。每次命令只发一个法兰目标；执行记录写入 `experiments/single_arm/control_interfaces/data/runs/`。若起点离 S1 太远、逆解不收敛或预测关节变化过大，脚本会拒绝；不能通过调大阈值强行回位。

若不在 S1，可先用只读位置记录建立**临时锚点**，按[控制器 IK 的临时起点步骤](../experiments/single_arm/control_interfaces/README.md)测试小范围运动。临时锚点不覆盖 S1 历史配置，回锚点仍是单个 `move_l` 命令。14:08 的记录已用于 2 mm 往返试验：去程、回程各发送一条 `move_l`，控制器均报告到位；现场确认回程平稳，随后独立状态检查为 `NORMAL`、七轴使能。

2026-09-25 又从 S1 完成 X 负向 10 mm、再回 S1 的[单次直线往返](../experiments/single_arm/control_interfaces/report.md)：去程和回程各一条 `move_l`，均到位；回程最大关节误差约 `0.000419 rad`，独立状态与 S1 检查通过，现场确认平稳。这个结果只适用于当时核对过的路线；下一次仍须从实时姿态重新预检。

若设备曾无预警失能，可在现场支撑机械臂后，单独开一个终端运行只读诊断：

```bash
python -m experiments.single_arm.can_setup.monitor_state --duration 300 --rate 2
```

它会同时保存状态 CSV 和急停/使能 CAN 帧 CSV；结束时打印两个路径。它不自动使能、复位或运动。如何根据两份记录区分“收到了控制命令”和“未见控制命令”，见[失能诊断步骤](emergency_stop_recovery.md)。

若机械臂突然停机，先在现场支撑并记录**准确时间、底座灯色、适配器灯色以及灯色后来是否变化**；再用第 2 节的只读状态命令读取整机状态和七轴使能位。不要因为底座灯曾短暂变绿，就推断稍后的驱动仍已使能。若已有后台监测，保留状态、控制帧及主机侧 CSV；[分析命令和记录位置](emergency_stop_recovery.md)可帮助对齐失能、CAN 网卡变化和本工作区进程退出的时间。日志未覆盖事件瞬间时，原因应记录为未确认。

## 5. 试验控制器 IK 与法兰点位 `move_p`

先完成第 2、3 节的只读状态与 S1 核对，然后只读预检一个 X 负向 2 mm 的点位目标：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_point_session --dx-m -0.002
```

输出“只读预检完成；未发送运动指令”后，检查目标位姿与现场全部连杆、线缆、支撑的活动范围。`move_p` 把六维**终点**交给控制器计算 IK 和点位运动；离线采样不能预测控制器实际中间路线。现场确认后，执行一次去程：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_point_session --dx-m -0.002 --run
```

只有去程完成、现场无异常，才先只读核对回 S1，随后在现场确认回程范围后执行：

```bash
python -m experiments.single_arm.control_interfaces.cartesian_point_session --to-s1
python -m experiments.single_arm.control_interfaces.cartesian_point_session --to-s1 --run
```

每条带 `--run` 的命令只发送一个 `move_p` 目标。完成后再次运行第 2、3 节的只读检查。2026-09-25 已有 2 mm 和 4 mm 的[实测报告](../experiments/single_arm/control_interfaces/report.md)；4 mm 试验中，法兰中途相对起点最大移动约 8.4 mm，所以不能只按终点的 4 mm 距离检查障碍。距离单位是米：`-0.004` 为 4 mm，`-0.04` 为 40 mm；脚本会拒绝超过 5 mm 的首次点位试验。历史结果不替代本次的现场核对。

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
