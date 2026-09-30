# NERO 当前状态（F8 软件架构收口）

本文记录项目证据与续接边界，**不是机器人实时状态**。正式契约见 [PROJECT_DESIGN.md](PROJECT_DESIGN.md)，职责见 [PROJECT_MAP.md](PROJECT_MAP.md)，旧入口见[迁移表](docs/legacy_runtime_migration.md)；F1～F7 的逐轮详情见仓库外第 11～18 号报告和[测试审计](docs/test_tiers_audit.md)。

## Current Runtime

`src/nero_runtime/` 已按 F1～F7 形成单臂 Runtime：状态解析与采集、只读点位、顺序关节路点 Motion、轨迹记录、Drag Core、Guided Recording、Replay、Environment Profile 与 Start/Park/Return READY 工作流。F8 固定了惰性 Python 公开导出、内部注入执行边界、CLI 帮助、旧实验迁移分类和显式离线回归入口。正式 Runtime 不依赖 `experiments`；旧实验仍按各自身份保留，不是正式 API。

普通 CLI 的 `status`、`waypoint capture`、`trajectory record` 会显式连接只读设备来源，**尚未完成真实设备只读核验**；show/list 等只读本地文件。Motion、Drag、Teach、Replay run、Start/Park/Return、disable、reset 没有普通真实动作 CLI。Python 子模块中的执行内核只接受调用方显式注入的设备操作；尚无经过现场晋升的真实硬件 adapter。

## Offline verified

F1～F7 的 129 项历史白名单测试仅覆盖合成反馈、临时 JSON/JSONL 和注入的设备行为。F8 的新增测试及同一白名单最终运行结果、精确 Python/HEAD/命令/文件哈希，以[测试审计](docs/test_tiers_audit.md)和仓库外《19 F8 Runtime 收口与迁移完成报告》为准。已运行且通过的明确测试集合才有对应范围的 `OFFLINE_TESTED` 结论；**Full test discovery = UNKNOWN_SIDE_EFFECT**，不能称整个实验测试套件已离线验证。

## Hardware not verified

软件架构收口不等于硬件能力晋升。真实 V121 反馈新鲜性/时基、`move_j` 发布与 SDK clamp 后目标、真实 stop 映射、Drag enter/exit 反馈及物理响应、READY/PARK 点位和承托、Start/Park/Return 扫过区、Teach v1 和 Replay 均待 Field Validation。测试中的合成 Profile `validated` 是资产字段，不是现实站点批准；负 X 或失能/故障观测也不会自动产生运动许可。厂家保护、环境标定和路线许可须分别核对，未经批准的旧通用法兰 X≥0.15 m / X 非负假设不能重新晋升为 Runtime 通用规则。

## Locked

旧 `supported_home` 擦碰路线、高位旧入口及事故相关入口保持锁定；独立真实 Drag 未晋升。旧实验唯一的 H0/S1 现场路线、故障/恢复证据及真实 adapter 尚不能安全删除。[迁移表](docs/legacy_runtime_migration.md)逐项记录保留原因与现场依赖。旧实验代码不因 F8 自动获得新使用许可。

## Field Validation backlog

先在新的、明确授权的现场任务中确认设备当前状态、机器人/安装/工具身份和环境版本，再逐项验证只读来源、反馈时间语义、发布/停止/Drag 映射、真实点位和路径扫过区；按操作分别形成证据与晋升决定。不能沿用过期实时采样作今日许可。

历史风险：2026-09-27 07:26 UTC S1→H0 的报告为 5/7 后中断、`completed=false`，随后存在一次 reset 记录及 H0/S1 预检拒绝。用户批准的后续低位回 S1 试验和 13:02 UTC 状态是各自时点证据；现场桌边/线缆观察仍待确认。**此刻设备姿态、使能、实体承托与周边状态未知**。原始证据见 `experiments/single_arm/lab/data/`、`experiments/single_arm/can_setup/data/recoveries/` 和既有实验报告；不得把它们当作当前现场许可。

## Git checkpoint

F8 在离线回归、变更分类及原始实验数据核对通过后创建一个本地 Git checkpoint；精确 commit SHA、文件清单与哈希记录在仓库外第 19 号报告及审阅 ZIP。未 push。

## Next action

软件架构整改到 F8 结束。下一阶段为独立授权的 **Field Validation / Hardware Promotion**，按上方待办逐项验证；没有自动的 F9/F10 架构阶段，也没有因本软件 checkpoint 而解锁任何真实运动或恢复操作。
