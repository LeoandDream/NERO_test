# NERO 当前项目状态

更新：2026-09-27 16:49 CST。本文是可替换的**项目状态索引**，不是实时设备状态、运行许可或历史日志。正式操作契约见 [PROJECT_DESIGN.md](PROJECT_DESIGN.md)，证据和 Run 规则见 [实验规范](docs/experiment_run_standard.md)。事实来源注明于各项；未列出的条件一律未知。

## 阶段与已知范围

- **阶段**：Step 1B 正式 Runtime 设计契约、Step 2B 实验规范与 Step 2C 最小离线测试边界已落盘，并在 Step 2D 纳入独立 Git 治理基线；正式 Runtime 尚未实现或晋升。[设计](PROJECT_DESIGN.md)；仓库外整改记录：《05 Step 2B 实验规范与连续性落盘 20260927 161254.md》《06 离线测试边界验证 20260927 163256.md》。具体基线 SHA 以 Git 为准。
- **已验证离线子集（OFFLINE_TESTED）**：`control_interfaces/tests/test_publishers.py` 的 `PublisherTests` 两个方法，2026-09-27 08:32:15 UTC 在 `main@060d3491…`、Python 3.10.21 下显式运行 2/2 通过；测试使用本地 Fake，没有检测到设备副作用。精确命令、哈希和导入链见仓库外整改记录《06 离线测试边界验证 20260927 163256.md》与[逐项审计](docs/test_tiers_audit.md)。**整套 discover 仍为 `UNKNOWN_SIDE_EFFECT`**；这两个测试不代表 Runtime 全面回归。
- **历史验证范围（DOCUMENTED）**：单臂左侧安装、当时裸法兰和受监护现场，S1↔新 H0 `reduced-j3` 路线曾双向 7/7 到位并获现场无异常反馈；H0 是轻触桌边的候选支撑位，不是跨环境通用 PARK。[实验报告](experiments/single_arm/lab/report.md)、[去程原报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260926T174642572191Z.json)、[回程原报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260926T174905716531Z.json)。这只证明当时条件，不授权今天的路线。
- **锁定和未验证**：旧 `supported_home` 擦碰路线与高位旧入口保持 locked；独立 Drag 未晋升；六面几何和任意内部运动区域未验证。[职责地图](PROJECT_MAP.md)、[立方体报告](experiments/single_arm/workspace_cube/report.md)。正式 Environment Profile 的整体 validated 版本尚未建立，[设计](PROJECT_DESIGN.md)中的 Profile 仍是未来契约。

## 最近事件与开放问题

- 2026-09-27 07:26 UTC 一次 S1→H0 运行报告记载 5/7 目标到位，随后 `InterruptedError` 且请求软件电子急停；报告 `completed=false`。[原报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T072614491520Z.json)。该文件目前未跟踪；其出现不等于本轮亲见机器人动作。
- 07:48 UTC 的[恢复记录](experiments/single_arm/can_setup/data/recoveries/nero_emergency_recovery_20260927T074807Z.json)记载一次 reset 尝试与前后反馈。用户贴出的 07:50:58 UTC `status` 曾显示七轴使能、`ready_for_motion=true`、法兰 X≈−0.182 m；其来源与缺项已记入仓库外整改记录《05 Step 2B 实验规范与连续性落盘 20260927 161254.md》。07:49 和 07:51 的[H0 预检拒绝](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T074932799103Z.json)与[S1 预检拒绝](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T075128293432Z.json)说明当时关节角既不匹配 S1，也不匹配新 H0。这些只是带时间的历史采样；**此刻设备状态、实体承托、安装/线缆条件均未知**。
- **开放问题**：中止后至恢复记录间的实际停点与位移原因未由现有报告证明；低位非命名姿态没有已批准的普通恢复路线。运行报告未完整给出中止瞬间最后命令尝试与反馈，不能从 `completed_targets` 推定实际停点。[路线报告](experiments/single_arm/lab/data/optimized_home/nero_optimized_home_20260927T072614491520Z.json)、[未来 Cancel/Fault 契约](PROJECT_DESIGN.md)。

## 续接

- **最近完成**：Step 2D 将已批准的设计与实验治理文档固定为独立 Git 基线；未运行项目代码或执行任何硬件操作。Step 2C 仅显式运行上述 T1 测试类 2/2 通过，详情见仓库外整改记录《06 离线测试边界验证 20260927 163256.md》。
- **Architecture Track**：无已获授权的后续实施任务。下一步可逐个核清更多测试的导入/fixture 链并扩展离线基线，或在另行授权后开始首阶段 Runtime 提取；当前两个测试覆盖范围很窄。
- **Hardware Track**：实时设备状态、实体承托和恢复路线仍未知。07:26 中断链的现场复核与任何恢复动作均需独立授权和新鲜证据；本轮架构验证不更新现场许可。
- **不可重试**：不重放旧 S1→H0 计划，不把非命名低位当 H0，不因 `ready_for_motion=true` 断言路线许可或实体承托，不放宽门槛以通过预检；失能、reset、Drag、运动和急停均需新的具体现场判断与授权。

维护：造成**项目能力、锁定状态、验证结论或关键现场证据**实质变化的执行者，在对应报告/Run 先留过程和来源，再替换本文过期摘要；只读的实时姿态不得写作持续有效的当前状态。无实质变化不追加检查点。
