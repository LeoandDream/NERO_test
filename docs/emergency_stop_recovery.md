# Nero 电子急停与实验恢复

本文针对当前工作区的 Nero 固件 1.21、左侧装、CAN 控制。恢复操作必须依据**实时状态和现场姿态**；过去的示教 CSV 不能代替当前关节反馈。

**2026-09-25 桌面接触事件：**多方向 `move_p` 的 X 负向段运动中法兰先碰到桌面，随后控制器报运动失败，脚本发送电子急停。用户说明这套左侧安装在默认低位时法兰本就会接触桌面，需抬升才能离开；接触本身并不表示设备受损。恢复前先确认机械臂由可靠支撑承重、当前关节角及法兰和桌面的受力状态。**不要直接照搬下面的一般复位和回 S1 命令**：官方 API 说明 `reset()` 可能让抬起的关节立即失电下落；从当前低位抬升也必须根据实时姿态重新规划。见[本次事件报告](../experiments/single_arm/control_interfaces/report.md#2026-09-25-x-负向点位发生桌面接触实验停止)。

## 先判断当前状态

在工作区根目录执行 `python -m experiments.single_arm.can_setup.nero_can_activate`，默认只读。机械臂停稳且现场有人监护后，按**整机状态、错误码和七轴驱动位一起**判断：

| 只读反馈 | 下一步 |
| --- | --- |
| `NORMAL(0x0)`、错误码零、七轴 `使能=True` | 重新读取当前关节角并规划；不能假定仍在 S1。 |
| `EMERGENCY_STOP(0x1)` | 先可靠支撑，再运行 `recover_emergency_stop` 只读检查；确认静止和现场范围后，才考虑单次 `--run` 复位。急停时的 `使能=True` 不表示仍在保持位置。 |
| `JOINT_BRAKE_NOT_RELEASED(0x6)`、七轴 `使能=False`、错误码零 | 先确认机械臂停稳、无接触且有支撑；可用 `python -m experiments.single_arm.pose_recording.record_poses --duration 5 --rate 5` 只读复核。之后现场执行一次 `python -m experiments.single_arm.can_setup.nero_can_activate --enable`，再独立运行只读状态命令确认 `NORMAL`。此状态无需重复 `reset()`。 |
| 其他状态、非零错误码、驱动欠压/故障或反馈缺失 | 暂停运动与使能，保存原始输出，核对供电、CAN 和现场机械状态。 |

`--enable` 只处理使能，不负责把机械臂移回 S1。任何恢复动作都需基于使能后的**最新**姿态重新预检。

## 发生了什么

2026-09-24 的 20 秒示教回放规划为 322 步去程、322 步返程。实机在返程总进度 558/644 后，脚本报告小步目标未按时到位并调用 `electronic_emergency_stop()`，本轮没有返回 S1。[回放失败报告](../experiments/single_arm/replay/data/replays/nero_replay_20260924T104954Z.json)记录 `completed_cycles=0`，该轮约运行 120.085 秒。旧报告未保存急停前的目标及反馈，不能确定是整轮时限到期还是单步停滞。

随后只读检查显示 `CAN_CTRL`、`EMERGENCY_STOP`、错误码 `0x0000`。[停机后采样](../experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T105527Z.csv)的 9 条记录姿态稳定，约为 `[0.16567, 0.26487, -1.16218, -0.26527, -0.05424, 0.20567, -0.46887] rad`。该姿态距 S1 约 `1.468 rad`，已不在原回放轨迹上。现场确认无碰撞、异响或人工拖动，机械臂现已停稳。现有日志没有急停前一刻的关节角，不能判断姿态变化的确切时刻。

之后机械臂姿态再次变化；[10 秒复核采样](../experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T111103Z.csv)的 49 条记录均为 `EMERGENCY_STOP`，稳定在约 `[0.23803, 0.33111, 0.06733, 0.02995, 0.01119, 0.20567, -0.46887] rad`。第一次 0.4 秒检查无法排除随后继续阻尼下落，因此恢复脚本现连续观察约 5 秒，并在发送复位前再复核一次。上述旧姿态和到 S1 的距离不能作为当前规划依据。

## 急停、失能与复位

[官方 Nero API](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/nero/nero_api.md)说明：电子急停是**阻尼停止**，抬起的关节可能缓慢落下；`reset()` 会令机械臂**立刻失电**，抬起的关节可能立即掉落。急停后的驱动反馈即使显示 `使能=True`，也不能据此推断关节仍在位置保持。复位只是清除急停状态的步骤，不会自动回 S1。不要用 `disable()` 代替复位，也不要从现在的位置直接运行旧的示教回放或去 S1 脚本。

本次状态变化的顺序是：第 423 个关节目标未到位 → 回放脚本因超时调用 `electronic_emergency_stop()` → 整机状态 `EMERGENCY_STOP`，驱动位仍显示 `True`，但不代表位置保持 → 现场支撑后调用一次 `reset()` → 稍后反馈 `JOINT_BRAKE_NOT_RELEASED(0x6)` 且七个驱动位均为 `False` → 单次 `--enable` 后延时检查恢复 `NORMAL`、七个驱动位均为 `True`。代码在中断时没有调用 `disable()`；**明确失能发生在复位之后**。造成第 423 个目标未执行的控制器内部原因尚未查明，不能归结为速度低。详见[10% 回放报告](../experiments/single_arm/replay/data/replays/nero_replay_20260924T131725Z.json)。

随后又出现一次**不同的失能事件**：机械臂已回到 S1 后，现场报告在使用额度耗尽时突然停机，未主动发送运动、失能或复位命令。当时仅用机械臂随箱原装的 24V 适配器供电，没有外接电池；尚未抄录适配器铭牌的输出电流。现场观察为停机后机械臂底座面板和适配器两处指示灯都变绿。新实验的只读预检后来发现 `JOINT_BRAKE_NOT_RELEASED(0x6)`，七轴驱动位全为 `False`；[14:08 的 25 条采样](../experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T140821Z.csv)确认末 24 条在约 4.8 秒内稳定，但姿态已变为约 `[0.0496, 0.0780, 0.0281, 0.0786, 0.0115, 0.2152, -0.9849] rad`，法兰 X 约 `−0.0649 m`。现场确认有支撑且无接触，之后一次使能报告 `NORMAL`。[用户手册](user_guide.md)把底座绿灯常亮定义为正常使能，现场未留意绿灯后来是否转红，因此不能把停机瞬间的灯色与稍后的驱动位反馈当作同一时刻的状态。适配器灯色也没有同步电压、电流记录作为解释。现有记录没有突然停机瞬间的 CAN 事件或供电变化，因此不能把原因确定为额度、SDK 断连、通信看门狗或供电问题；下一步实验必须从**最新实时姿态**核对，不能沿用 S1。

[用户手册的供电要求](user_guide.md)为 DC 24V；外部供电需稳压、电流能力不小于 10A，电压不得超过 25V。现场确认使用原装适配器，但尚未读取其铭牌电流，也没有停机瞬间的适配器端或机械臂输入端电压测量。本次持续监测采到的 23.4～23.9V 是驱动反馈值，不能用它排除短时供电跌落，更不能反推上次停机时的输入电压。

对照本机会话日志，2026-09-24 **13:50 UTC** 用户运行不带 `--run` 的 `teach_session`，反馈仍接近 S1；**14:07 UTC** 用户运行不带 `--run` 的 `cartesian_linear_session`，首次报告 `JOINT_BRAKE_NOT_RELEASED(0x6)`；随后独立只读状态检查确认七轴失能。这约 17 分钟内会话工具记录主要是源码阅读、文件修改和离线测试，没有新的实机运动命令。该时间线缩小了突停发生区间，并说明当时不是已知回放脚本的“目标未到位后主动急停”分支；它不能证明是否有其他本机进程、控制器内部策略或供电事件，因为当时没有连续的状态、CAN 与适配器输出记录。

复查前一次系统启动的持久化日志：上述本地时间 **21:45～22:20** 的用户态日志仍存在，但该时间段没有可用的内核 CAN/USB 事件记录。因此不能从“没看到 USB 断连或总线错误”推断接口当时没有发生瞬态故障；这份日志也无法提供控制器电源的测量值。

已核对本机安装的 SDK 源码及[官方 `disconnect()` 实现](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/drivers/core/arm_driver_abstract.py)：`disconnect()` 只调用[通信上下文关闭流程](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/drivers/core/driver_context.py)，停止读/监控线程并关闭 CAN 资源；通信对象析构时也只是关闭收发 Socket。上述清理路径没有调用 `disable()`、`reset()` 或 `electronic_emergency_stop()`。所以“脚本正常退出时 SDK 主动下发失能命令”缺少代码依据。这个源码检查仍不能排除控制器侧通信超时策略、外部电源事件或其他进程的 CAN 命令；确定突然失能的根因需要停机瞬间的时间戳和 CAN/供电记录。

再次审计本工作区脚本：没有以 `robot.disable()` 作为进程退出清理动作；电子急停调用只存在于运动执行器检测到异常后的处理分支，`reset()` 只在显式运行急停恢复脚本时发送。只读预检和监测程序退出时只断开 SDK 连接。这能排除这些**已检查代码路径**在正常退出时主动发失能，但无法证明 2026-09-24 突停时究竟运行了哪一个进程，也不能排除未被工作区覆盖的发送端。

单次法兰直线实验另发现一处潜在的软件误判：原运动循环在检查驱动状态时可能为等待较慢的驱动帧阻塞进展判断。现已改为发送前完整核对七轴使能、运动中不等待过期的驱动帧，并将过期关节编号写入[运行记录](../experiments/single_arm/control_interfaces/README.md)；新鲜反馈若报告失能、欠压或驱动故障仍立即中止。9 项对应离线测试通过。这个修正不能证明它就是此前两次停机的原因，后续仍需记录停机瞬间的状态和供电反馈。

若需要排查是否再次自行失能，可在现场支撑就位后运行一条只读日志命令：

```bash
python -m experiments.single_arm.can_setup.monitor_state --duration 300 --rate 2
```

程序会同时保存 `nero_state_*.csv` 和 `nero_control_frames_*.csv` 到 `experiments/single_arm/can_setup/data/monitoring/`。前者每行记录状态、七关节使能位、电压和关节角；后者记录 CAN `0x150`、`0x151`、`0x471` 原始数据与时间戳，分别涵盖电子急停/复位、运动模式、失能/使能命令。按 `Ctrl+C` 可结束。它只观察，不自动复位、使能或维持机械臂位置；突然断电或进程退出时，以实际已写入的行为准。CAN 监听若报错，程序会明确报“诊断记录不完整”，不要把缺帧解释成没有控制命令。

2026-09-25 已在本机运行 10 秒只读试验：[状态记录](../experiments/single_arm/can_setup/data/monitoring/nero_state_20260925T074524Z.csv)生成 21 行，启动瞬间首行尚无新鲜反馈，其余行均为 `NORMAL`、七轴使能。随后一小时[连续状态记录](../experiments/single_arm/can_setup/data/monitoring/nero_state_20260925T081239Z.csv)生成 7,201 行，状态持续为 `NORMAL`，没有复现失能；监测到时退出后独立只读检查也仍为 `NORMAL`、七轴使能。因此这次正常的 SDK 断开没有导致失能，但不能证明此前突然失能的原因。

曾发现监听盲区：本机安装的 SDK 默认 `local_loopback=False`，旁路 SocketCAN 监听器看不到这些本机发送帧。用同一 5% 速度设置做对照，默认配置下[控制帧记录](../experiments/single_arm/can_setup/data/monitoring/nero_control_frames_20260925T081239Z.csv)没有增加；将发送端配置为 `local_loopback=True` 后，同一文件记录到 `0x151 / 01FF050000000000`。该帧只是速度设置，**没有运动目标**。工作区会发指令的入口已开启本机回环，便于此后的旁路监测；其他程序或设备若未开启回环，仍可能不在这份记录里。尤其不能把旧记录中“未见急停/失能帧”解释为未曾发送。

现在又启动了最长六小时的后台只读监测。实际状态和控制帧路径由 `experiments/single_arm/can_setup/data/monitoring/active_monitor.log` 给出，进程号保存在同目录的 `active_monitor.pid`。查看命令：

```bash
cat experiments/single_arm/can_setup/data/monitoring/active_monitor.log
cat experiments/single_arm/can_setup/data/monitoring/active_monitor.pid
```

需提前结束时，先把上一步读到的进程号填入下面两条命令；**确认 `ps` 显示的是 `monitor_state` 后**才执行 `kill`：

```bash
ps -fp 这里填进程号
kill 这里填进程号
```

后台监测会自动到时结束，不会改变控制器状态。状态 CSV 只含驱动反馈电压，不能代替电池供电或 USB 总线的独立电气测量。

另有一个[主机侧只读记录器](../experiments/single_arm/can_setup/monitor_host.py)与上述监测同时运行。它每 2 秒写下 `can0` 是否存在、网卡收发/丢包计数，以及当时运行的**本工作区模块名与 PID**，不保存完整命令行或环境变量。文件路径在 `active_host_monitor.log`，进程号在 `active_host_monitor.pid`；它同样不发送 CAN 指令。如果再次失能，将两个 CSV 按 UTC 时间对齐，可检查是否同时出现 USB-CAN 消失、收发错误或某个工作区进程退出。没有这些现象，也仍不能排除控制器或适配器供电事件。

还准备了[只读客户端异常退出对照](../experiments/single_arm/can_setup/probe_client_exit.py)：默认命令只读取当前状态；显式 `--run` 才启动另一个只读 SDK 客户端并用 `SIGKILL` 强制结束它，随后只读检查七轴状态。该动作本身**可能导致失能与下落**，目前尚未实机执行；需先确认现场支撑和人员避开夹点。正在运行的状态监测本身仍占用一个 SDK 连接，所以即使此对照未触发失能，也不能据此排除“最后一个客户端退出”才会触发的控制器策略。该对照不模拟适配器电压波动。

拿到一对监测文件后，可以用[只读分析命令](../experiments/single_arm/can_setup/analyze_monitor.py)列出整机状态或七轴使能位的变化、采样到的驱动电压范围和最长采样空档，并显示变化前后 5 秒内记录到的控制帧；把命令中的文件名换成 `active_monitor.log` 打印的实际路径：

```bash
python -m experiments.single_arm.can_setup.analyze_monitor \
  experiments/single_arm/can_setup/data/monitoring/nero_state_实际时间.csv \
  experiments/single_arm/can_setup/data/monitoring/nero_control_frames_实际时间.csv \
  --host-csv experiments/single_arm/can_setup/data/monitoring/nero_host_实际时间.csv
```

没有主机侧 CSV 时，省略整段 `--host-csv ...` 即可；若有状态跳变，分析结果会列出跳变前后的相邻主机采样。主机样本可能比事件晚约 2 秒，不能据此精确排序同一采样间隔内的供电和控制动作。

对已完成的一小时文件运行该命令，输出为 7,201 条状态、1 条速度设置帧，有效采样中**未见状态或使能位跳变**。这只说明该窗口内没有复现问题，不是此前失能的原因证明。

若希望用 `can-utils` 独立核对控制帧，可另开终端运行：

```bash
cd /home/leo/nero_dh116_ws
mkdir -p experiments/single_arm/can_setup/data/monitoring
stdbuf -oL candump -L can0,150:7FF,151:7FF,471:7FF > experiments/single_arm/can_setup/data/monitoring/can_control_trace.log
```

该终端会持续等待；需要结束时按 `Ctrl+C`。根据 SDK 的[运动控制帧定义](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/msgs/piper/default/transmit/arm_motion_ctrl.py)、[Nero 模式帧定义](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/msgs/nero/default/transmit/arm_mode_ctrl.py)和[驱动使能帧定义](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/msgs/nero/default/transmit/arm_motor_enable_disable.py)：`0x150` 的第 1 字节 `01` 是电子急停、`02` 是复位；`0x151` 第 2 字节区分 P/J/L/C 运动模式；`0x471` 的第 2 字节 `01` 是失能、`02` 是使能。若状态切换前抓到失能或复位命令，应继续定位发帧进程；若没抓到，只能说**本次抓取未见这些命令**，还需排查控制器和供电。抓取程序只监听这些帧，不会发送 CAN 帧；`candump` 或监听程序若中途停止或漏帧，缺少命令帧不能作为排除证据。

## 回放脚本的急停修订

2026-09-24 两次长回放中断后的电子急停均由本工作区脚本主动请求。第一次旧版约 120 秒时限处理不当；第二次第 423 小步在 5 秒内没有执行到目标，旧版通用异常处理请求急停。2026-09-25 的修订使整轮时限只在小步确认到位后暂停；单步未动时连续核对整机和关节状态，必要时只重发相同目标一次，仍未动则暂停并保留当前位置。关节漂移、控制器异常、路径越界或反馈缺失仍请求急停。新回放报告分别记录 `pause_reason` 与 `emergency_stop_requested`。这些改动通过离线测试，未经过长回放实机验证；旧长轨迹不可直接作为已验证方案重复执行。

核对当前安装的 pyAgxArm 源码：`electronic_emergency_stop()` 明确发送运动控制消息 `ArmMsgMotionCtrl(1)`；`disconnect()` 只停止 SDK 线程并关闭 CAN 通信资源，没有发送急停或失能消息。因此上述两次报告中的主动急停来自回放脚本的异常处理，不是正常断开连接本身。若之后再次发生失能，仍须用当时的 CAN 帧和状态记录判断，不应直接套用这两次的结论。

## 命令行恢复顺序

在工作区根目录激活 `nero-py310` 后，先运行**只读检查**：

```bash
python -m experiments.single_arm.can_setup.recover_emergency_stop
```

它检查状态、关节反馈、法兰反馈和约 5 秒内的持续静止情况，不发送复位或运动指令。若状态已变化、机械臂仍在移动、错误码不为零或反馈缺失，应停止该流程并重新判断现场情况。2026-09-24 的第一次 0.4 秒静止检查后，机械臂姿态相对前一份停机采样又明显变化，因此脚本已改为约 5 秒连续检查；复位前还会再次检查。

复位前，现场人员必须确认底座固定、机械臂有**可靠支撑**、人员避开所有关节夹点和可能下落的范围。准备就绪后，下面的命令有 5 秒倒计时，只发送一次 `reset()`，随后只读记录状态与关节角；**不会自动使能或运动**：

```bash
python -m experiments.single_arm.can_setup.recover_emergency_stop --run
```

恢复记录保存在 `experiments/single_arm/can_setup/data/recoveries/`。即使复位后反馈未确认，也不要盲目重复发送 `reset()`。先看现场姿态，再运行下面的只读状态命令：

```bash
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.pose_recording.record_poses --duration 2 --rate 5
```

复位后的当前位置必须重新作为规划输入。不能沿旧计划自动回 S1。历史上曾按现场核对的分段关节路径恢复 S1；这些路径只适用于当时起点。新[控制器 IK 实验](../experiments/single_arm/control_interfaces/README.md)只允许 S1 附近单次 `move_l`，从大幅下垂姿态直接回 S1 的六维直线路线尚未通过预检。不要为满足“一次回位”而从未知姿态强制发送单个目标。

## 关于提高回放速度

此前停机尚不能单独归因于 5% 速度。后续已在安全新起点完成 5% 和 10% 的 J4 短行程对照，均无现场异常；这不能证明完整长轨迹在 10% 下安全。回放代码已将整轮时限到期改为小步到位后暂停，并加入单步进展等待与急停前反馈记录；长轨迹尚未实机复测，流程见[回放使用说明](../experiments/single_arm/replay/README.md)。

急停解除、七关节重新使能且延时只读检查确认为 `NORMAL` 后，现场选择以当前姿态为新起点。短行程对照使用[局部关节实验](../experiments/single_arm/local_probe/README.md)：J4 正向 `0.05 rad` 再返回，避免从当前 J4 正角穿越零位；先 5%，成功后才考虑 10%；每次都先只读规划并由现场检查活动范围。这一对照不自动返回旧 S1，也不能直接证明长轨迹可提速。

## 本次复位执行记录

现场确认支撑就位且范围清空后，执行了一次 `recover_emergency_stop --run`；详见[恢复记录](../experiments/single_arm/can_setup/data/recoveries/nero_emergency_recovery_20260924T111407Z.json)。复位前约 5 秒内关节反馈未变化；脚本在倒计时后再次核对并发送一次 `reset()`。随后的第一次状态反馈为 `NORMAL`、错误码 `0x0000`，关节角与发送前一致。此记录**不包含自动使能或运动**；是否存在稍后的姿态变化仍需继续只读观察。

稍后的只读驱动检查显示 `CAN_CTRL`、`JOINT_BRAKE_NOT_RELEASED(0x6)`、错误码零，七个关节驱动使能位均为 `False`，电压约 23.5–24.0 V，无欠压和驱动故障。这是复位后尚未使能的状态，不表示可以执行运动。复位后一段时间的实际关节姿态仍需重新采样；不能沿用恢复记录中的第一次姿态读数。

[复位后 5 秒采样](../experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T112133Z.csv)共 24 条，均为 `JOINT_BRAKE_NOT_RELEASED`，七关节角没有可见变化，约 `[0.25710, 0.35004, 0.06733, 0.03538, 0.01098, 0.11252, 0.45009] rad`。现场确认无接触、已停稳。该姿态距 S1 的关节空间距离约 `1.377 rad`，超出现有 `0.8 rad` 起点转移上限；后续不能直接沿旧规划去 S1。

现场随后发送了一次关节使能命令：七个驱动均回报 `True`，现场无异常移动或接触；命令刚结束时读取的整机状态仍为 `JOINT_BRAKE_NOT_RELEASED(0x6)`。两类反馈异步到达，这个即时读数不能证明整机已恢复 `NORMAL`。使能脚本现要求等待新的 `NORMAL` 反馈后才报告成功；随后的独立只读检查确认 `NORMAL`、七个驱动 `True`。此后又一次急停的复位和使能也按相同顺序恢复；后续须以最新实时状态为准。
