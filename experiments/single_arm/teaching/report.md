# Nero 示教过程记录（2026-09-23）

本文记录本工作站已实际执行的 CAN 拖动示教和回位。以下时间均为北京时间（UTC+8）；CSV 和 JSON 内保存的是 UTC 时间。机械臂为 Nero，固件 1.21，使用 `NeroFW.V121`、`can0` 和 Python 3.10。机械臂实际为**左侧装**，示教时按安装码 `0x02` 设置重力补偿。

## 过程与结果

| 时间 | 操作与结果 |
| --- | --- |
| 22:00–22:09 | 连续记录关节角和法兰位姿，共 5269 条；其中 22:07:52–22:08:50 为拖动示教状态，共 585 条。提取出 P1–P4 四个停留候选点。 |
| 22:53 | 以实测 P4 为起点进行第一次 5 秒限时示教，记录 245 条。到时退出拖动后，机械臂还移动了约 0.078 rad（七关节空间距离），最后记录点与停稳姿态超过当时的 0.05 rad 端点阈值。脚本拒绝自动回位；此次 `return_completed=false`，**未发送回位运动指令**。 |
| 22:59 | 手动精确回到 P4 困难。退出拖动后，现场确认当前稳定姿态安全，将 CAN 实测的七关节位置保存为新起点 **S1**。原 P4 配置单独保留。 |
| 22:59–23:00 | 从 S1 再做一次 5 秒示教，录得 245 条拖动样本；结束指令后继续补录 35 条，确认停稳，总计 280 条。倒序路径规划出 22 个小步。切回 CAN 控制后反馈 `teach_status=TERMINATE_EXECUTION(0x6)`，旧判断拒绝下发回位关节指令。 |
| 23:06 | 使用第二次示教的完整记录，从记录末端独立恢复回位。现场清空路径后，按 10% 速度走完 22 个小步，并逐点核对实际关节反馈。回到 S1；独立只读检查显示当前位置与 S1 的最大单关节误差小于 0.001 rad，机械臂状态 `NORMAL`、错误码 `0x0000`，七个驱动均使能。第二次记录最终标记 `return_completed=true`。 |

第二次会话的 JSON 保留了第一次自动回位被拦截的原因 `initial_return_error`，也记录了后来完成的 `resume_attempts`。这两个字段描述的是同一次示教的不同回位尝试。回位后控制器仍反馈 `teach_status=0x6`，但同时反馈 CAN 控制、`NORMAL` 和零错误码；当前脚本只在这些条件同时满足时接受这一状态。

## 起点与数据

默认起点现在是 [S1 配置](../config/nero_teach.json)，七个关节角单位为 rad：

```text
[0.017383, -0.578751, 0.028047, -0.299516, 0.011240, 0.205670, -0.473438]
```

S1 由现场人员确认姿态安全，并在停稳后连续读取 CAN 反馈保存。该次原始配置另存为 [带时间戳的 S1 配置](../config/nero_teach_start_20260923T145941Z.json)。原 P4 来自 [停留候选点](../pose_recording/data/poses/nero_poses_20260923T140013Z_held_candidates.csv)，其参数保存在 [P4 配置](../config/nero_teach_p4.json)。不要把 P4 和 S1 的记录混用，也不要从未知位置或全零位直接发往任一起点。

| 文件 | 用途与结果 |
| --- | --- |
| [最初的完整记录](../pose_recording/data/recordings/nero_poses_20260923T140013Z.csv)、[示教状态筛选记录](../pose_recording/data/recordings/nero_poses_20260923T140013Z_teaching_only.csv) | 提取 P1–P4 的来源；不属于后来的两次 5 秒会话。 |
| [第一次 5 秒记录](data/recordings/nero_session_20260923T145322Z.csv)、[结果 JSON](data/recordings/nero_session_20260923T145322Z.json) | 起点 P4；因退出拖动后的轨迹缺口，未回位。 |
| [第二次 5 秒记录](data/recordings/nero_session_20260923T145952Z.csv)、[结果 JSON](data/recordings/nero_session_20260923T145952Z.json) | 起点 S1；含停稳补录，后来沿原记录倒序成功回位。 |

## 以后怎样操作

在工作区根目录、`nero-py310` 环境中，先做只读起点检查，再由现场人员监护运行限时示教：

```bash
conda activate nero-py310
python -m experiments.single_arm.teaching.teach_session
python -m experiments.single_arm.teaching.teach_session --run --max-seconds 5
```

默认最长示教时间为 120 秒，命令行允许 1–300 秒。启动前脚本检查七个驱动、故障状态、关节限位、当前位置是否接近 S1（每关节最多偏差 0.05 rad），并排除全零位附近。到时或示教期间按一次 `Ctrl+C` 后，脚本退出拖动、记录停稳过程、检查轨迹连续性，等待 5 秒，再尝试沿本次轨迹倒序返回**本次示教前的实际位置**。倒计时期间按 `Ctrl+C` 可取消回位；已发送回位运动指令后，若未达到小步目标或状态异常，脚本停止并发送电子急停。

若未来某次示教记录完整，但回位未完成且机械臂仍位于该记录末端，可先只读检查，再在现场确认整条返回路径后显式运行恢复命令。用那次示教的 CSV 和起点配置替换示例路径：

```bash
python -m experiments.single_arm.teaching.return_session experiments/single_arm/teaching/data/recordings/nero_session_YYYYMMDDTHHMMSSZ.csv --config experiments/single_arm/config/nero_teach.json
python -m experiments.single_arm.teaching.return_session experiments/single_arm/teaching/data/recordings/nero_session_YYYYMMDDTHHMMSSZ.csv --config experiments/single_arm/config/nero_teach.json --run
```

恢复工具会拒绝已标记为回位完成的记录。此次 [第二次 5 秒记录](data/recordings/nero_session_20260923T145952Z.csv)已经完成回位，不能再次用该工具执行。在 2026-09-23 这一步验证的是**拖动示教、记录和倒序回位**；后续正向回放结果见[回放报告](../replay/report.md)。即使记录轨迹有效，工作范围中后来出现的障碍物仍需现场检查。

## 后续 20 秒试验

23:12 又从 S1 启动一次最长 20 秒的示教。约 4 秒时 J4 到达脚本关节边界，脚本结束拖动并拒绝回位；[记录](data/recordings/nero_session_20260923T151241Z.csv)共 196 条，[元数据](data/recordings/nero_session_20260923T151241Z.json)为 `return_completed=false`。停稳后的 J4 约为 `−1.0233 rad`。这次的限位来源、法兰中心观测范围和后续测量方法见 [法兰工作空间与关节限位复核](flange_workspace_assessment.md)。

约 23:34，现场确认后曾尝试以左侧装 CAN 拖动模式把 J4 手动调回最后有效记录点 `−0.9829 rad`。控制器确认进入拖动，但关节反馈随后快速偏离目标；观察到 J4 接近 0 rad、其他关节也有明显变化，于是立即按 `Ctrl+C` 结束拖动。停止指令得到确认。现场人员确认机械臂停稳，未发生碰撞或异响。随后两次只读关节反馈稳定在约 `[0.0331, −0.7721, 0.0637, −0.0377, 0.0049, −0.1282, −0.1004] rad`；控制器为 `TEACHING_MODE`、示教 `DISABLED`、状态 `NORMAL`、错误码 `0x0000`，七个驱动均使能。**没有发送任何自动回位关节运动指令。** 这次位移的原因尚未确认，不能据此推断左侧装重力补偿已正确生效。

20 秒记录没有退出拖动后的停稳轨迹，而且其 `stop_reason=error`。恢复脚本现明确拒绝用这类不完整记录倒序回位。当前姿态也与该记录末端相差很大；不要再次运行该记录的手动末端恢复或自动回位。后续需先由现场人员重新确认当前姿态、安装方向与重力补偿行为，再决定新的安全起点和可用工作区域。

现场人员随后反馈：**网口朝向或关节零点是否正确仍有疑问**。[用户指南](../../../docs/user_guide.md)说明侧装以默认零点姿态下网口朝下定义，并警告关节零点设置不正确会使示教模式的重力补偿异常。因此，在确认物理安装、控制器安装方向以及七关节零点之前，不再执行 CAN 拖动或自动运动。不要为排查而直接触发自动零点校准：指南说明电机会先走到机械限位，再返回零点，需由现场按厂家流程准备。

2026-09-24，现场人员回复安装方向与关节零点已核对正确。重启后重新连接 USB-CAN、启用 `can0`，从当时的近零姿态生成[去 S1 的独立规划](../start_transfer/README.md)。第一份规划因当前位置变化被拒绝；第二份规划在使能后复核吻合，现场清空范围并监护，最终以 5% 速度完成 24 小步，独立核对已到达 S1。此前的 20 秒错误记录仍不可用于倒序回位。

随后按[五秒示教回放流程](../replay/README.md)测试已成功回位的 S1 记录：先正向回放、再沿相同关节路线返回，现场监护下共完成三轮。每轮正向 22 小步、反向 22 小步，返回各轮实际起点的最大单关节误差分别为 `0.000140`、`0.000140`、`0.000052 rad`。首轮后和三轮结束后均独立核对到 S1 附近、机械臂状态 `NORMAL`、无故障码、七个驱动使能。报告见[首轮](../replay/data/replays/nero_replay_20260924T085003Z.json)及[后两轮](../replay/data/replays/nero_replay_20260924T085053Z.json)。

同日继续做[法兰 X 负向 0.10 m 的自主计算轨迹试验](../cartesian_x/README.md)：从实时 S1 姿态生成位置逆解，现场确认整条连杆活动范围后，以 5% 速度完成 26 步外行程并原路返回 26 步。独立读数显示最大单关节返回误差约 `0.000070 rad`，法兰返回误差约 `0.000025 m`，控制器 `NORMAL`、无故障码，七个驱动检查通过。本次[规划及执行记录](../cartesian_x/data/plans/nero_cartesian_x_20260924T095015Z.json)已标记执行，不能复用。

之后的命令行分步测试中，现场实际运行了最长 **20 秒**的示教会话，超过原先拟定的 5 秒。该次[CSV](data/recordings/nero_session_20260924T101130Z.csv)记录 1022 行，其中退出拖动后的停稳补录为 24 行；[元数据](data/recordings/nero_session_20260924T101130Z.json)记为 `stop_reason=time_limit`、`return_completed=true`。离线复核相邻样本最大关节跨度约 `0.032 rad`，J4 最低约 `−0.953 rad`，距脚本限位边界最近约 `0.049 rad`。执行后操作者再次运行只读 S1 检查，关节角与本次示教前起点的最大单关节差约 `0.000105 rad`；随后确认该次自动回位无碰撞、异响或明显抖动。由于此记录较长且 J4 更接近边界，本轮回放仍使用已验证的旧 5 秒记录。
