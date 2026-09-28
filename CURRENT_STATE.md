# NERO 当前项目状态

更新：2026-09-27 21:54 CST。本文是可替换的**项目状态索引**，不是实时设备状态、运行许可或历史日志。正式操作契约见 [PROJECT_DESIGN.md](PROJECT_DESIGN.md)，证据和 Run 规则见 [实验规范](docs/experiment_run_standard.md)。事实来源注明于各项；未列出的条件一律未知。

## 阶段与已知范围

- **阶段**：Step 1B 正式 Runtime 设计契约、Step 2B 实验规范与 Step 2C 最小离线测试边界已落盘，并在 Step 2D 纳入独立 Git 治理基线。Step 3A 有未接线的内部纯内存状态解析内核；Step 3B 增加未接线的有限反馈采集适配。完整正式 Runtime 尚未实现或晋升。[设计](PROJECT_DESIGN.md)；仓库外整改记录：《05 Step 2B 实验规范与连续性落盘 20260927 161254.md》《06 离线测试边界验证 20260927 163256.md》《08 状态快照内核实现与离线验证 20260927 173740.md》《09 反馈采集适配与安全触发溯源 20260927 215426.md》。治理基线 SHA 以 Git 为准；Step 3A/3B 代码已纳入本仓库。
- **已验证离线子集（OFFLINE_TESTED）**：`control_interfaces/tests/test_publishers.py` 的 `PublisherTests` 两个方法，2026-09-27 08:32:15 UTC 在 `main@060d3491…`、Python 3.10.21 下显式运行 2/2 通过；测试使用本地 Fake，没有检测到设备副作用。精确命令、哈希和导入链见仓库外整改记录《06 离线测试边界验证 20260927 163256.md》与[逐项审计](docs/test_tiers_audit.md)。**整套 discover 仍为 `UNKNOWN_SIDE_EFFECT`**；这两个测试不代表 Runtime 全面回归。
- **Step 3A 新离线证据（OFFLINE_TESTED）**：`src/nero_runtime/snapshot.py` 对调用方提供的合成内存反馈进行纯解析；`tests/test_runtime_snapshot.py` 在治理基线 `caf3c2b…` 加未提交工作树、Python 3.10.21 下显式运行 15/15 通过，导入隔离检查未加载 SDK/CAN/旧实验模块。精确命令、被测 SHA-256 与边界见[测试审计](docs/test_tiers_audit.md)及仓库外整改记录《08 状态快照内核实现与离线验证 20260927 173740.md》。未验证真实 SDK 反馈、同步采集、停稳、FK、许可或硬件；**整套 discover 仍为 `UNKNOWN_SIDE_EFFECT`**。
- **Step 3B 新离线证据（OFFLINE_TESTED）**：`src/nero_runtime/acquisition.py` 只读取调用方提供的合成反馈源并调用 Step 3A 解析器；在 `main@caf3c2b…` 加当前未提交工作树、Python 3.10.21 下，显式运行 Step 3A 回归 15/15 与新采集测试 12/12，通过范围仅限合成源和本轮文件字节。精确命令、哈希与风险见[测试审计](docs/test_tiers_audit.md)及仓库外整改记录《09 反馈采集适配与安全触发溯源 20260927 215426.md》。未验证真实 SDK 连接/读取、同步快照、许可、执行或硬件；**整套 discover 仍为 `UNKNOWN_SIDE_EFFECT`**。
- **历史验证范围（DOCUMENTED）**：单臂左侧安装、当时裸法兰和受监护现场，S1↔新 H0 `reduced-j3` 路线曾双向 7/7 到位并获现场无异常反馈；H0 是轻触桌边的候选支撑位，不是跨环境通用 PARK。[实验报告](experiments/single_arm/lab/report.md)、[去程原报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260926T174642572191Z.json)、[回程原报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260926T174905716531Z.json)。这只证明当时条件，不授权今天的路线。
- **锁定和未验证**：旧 `supported_home` 擦碰路线与高位旧入口保持 locked；独立 Drag 未晋升；六面几何和任意内部运动区域未验证。[职责地图](PROJECT_MAP.md)、[立方体报告](experiments/single_arm/workspace_cube/report.md)。正式 Environment Profile 的整体 validated 版本尚未建立，[设计](PROJECT_DESIGN.md)中的 Profile 仍是未来契约。

## 最近事件与开放问题

- 2026-09-27 07:26 UTC 一次 S1→H0 运行报告记载 5/7 目标到位，随后 `InterruptedError` 且请求软件电子急停；报告 `completed=false`。[原报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T072614491520Z.json)。其出现不等于本轮亲见机器人动作。
- 07:48 UTC 的[恢复记录](experiments/single_arm/can_setup/data/recoveries/nero_emergency_recovery_20260927T074807Z.json)记载一次 reset 尝试与前后反馈。用户贴出的 07:50:58 UTC `status` 曾显示七轴使能、`ready_for_motion=true`、法兰 X≈−0.182 m；其来源与缺项已记入仓库外整改记录《05 Step 2B 实验规范与连续性落盘 20260927 161254.md》。07:49 和 07:51 的[H0 预检拒绝](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T074932799103Z.json)与[S1 预检拒绝](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T075128293432Z.json)说明当时关节角既不匹配 S1，也不匹配新 H0。这些只是带时间的历史采样；**此刻设备状态、实体承托、安装/线缆条件均未知**。
- **开放问题**：07:26 中止后至恢复记录间的实际停点与位移原因未由现有报告证明；低位非命名姿态没有已验证的普通恢复路线。用户批准的 12:09 UTC 实测姿态邻域受监护候选，后续运行报告仅证明该次目标发布与反馈到位，不能外推到其它低位或称为通用避障；现场桌边/线缆观察仍待确认。[本次实验报告](experiments/single_arm/lab/report.md)、[设计提案](docs/low_pose_s1_design_change_proposal.md)。07:26 运行报告未完整给出中止瞬间最后命令尝试与反馈，不能从 `completed_targets` 推定实际停点。[路线报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T072614491520Z.json)、[未来 Cancel/Fault 契约](PROJECT_DESIGN.md)。
- **待实施整改：未经批准的空间假设**：旧 `planned_return` 的法兰基座 X 默认门槛 `0.15 m` 与“X 非负”检查属于特定回 S1 规划路径；用户明确没有批准它们为项目工作空间或全局安全边界，现有材料未给出该数值的标定几何依据。它们不能进入新 Runtime 的状态读取/采集或通用许可。旧代码本轮未修改；后续须删除通用性表述和继承，单独决定是否有证据将其限定为实验路线条件，或由真实环境标定表达。厂家保护与事故路线锁定分别处理。[溯源报告](</home/leo/NERO 架构整改记录/09 反馈采集适配与安全触发溯源 20260927 215426.md>)。

## 续接

- **最近完成**：Step 3B 实现并离线验证内部反馈采集适配与 Step 3A 解析组合，同时静态溯源旧回 S1 空间门槛和运动后异常处理。未接线旧运行链，未改变旧安全检查；历史 Step 2C 的 PublisherTests 2/2 证据保持原范围。
- **Architecture Track**：下一步另行确定未获批准 X 门槛的整改范围和有依据的环境表达，再设计真实 SDK 接线、逐操作许可与执行故障处理；须验证反馈时基、缓存完整性和阻塞行为，不得以纯采集/解析结果替代旧连接、第二次采样、FK 或许可检查。
- **Hardware Track**：用户提供的 12:09:42 UTC 状态显示非 H0 低位、NORMAL、七轴使能且未接触桌边，当时左侧裸法兰布局一致。12:50 UTC 用户执行首目标 J3→−0.300 rad，[首目标报告](experiments/single_arm/lab/data/planned_return_runs/nero_planned_return_run_20260927T125008189660Z.json)记录 1/6 到位并暂停，用户确认平稳无异常。用户随后要求一次命令跑完剩余多个目标。13:01–13:02 UTC [完整路线报告](experiments/single_arm/lab/data/planned_return_runs/nero_planned_return_run_20260927T130136153836Z.json)记录 5/5 到位、最终最大关节误差 0.000070 rad；13:02:19 UTC 独立只读状态为 S1 附近、NORMAL、七轴使能、零错误且停稳。**完整路线的现场桌边/支撑/线缆观察仍待用户确认**，不能据此宣称无接触。所有状态均为带时点采样，不是持续有效许可；其它未知低位不能套用。
- **不可重试**：不重放旧 S1→H0 计划，不把非命名低位当 H0，不因 `ready_for_motion=true` 断言路线许可或实体承托，不临时绕过有依据的保护以通过预检；未经批准的 X 空间假设须另行整改。失能、reset、Drag、运动和急停均需新的具体现场判断与授权。

维护：造成**项目能力、锁定状态、验证结论或关键现场证据**实质变化的执行者，在对应报告/Run 先留过程和来源，再替换本文过期摘要；只读的实时姿态不得写作持续有效的当前状态。无实质变化不追加检查点。
