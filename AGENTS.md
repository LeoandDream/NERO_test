# NERO Agent 工作规则

本文件约束本仓库的后续 Agent。正式契约看 [PROJECT_DESIGN.md](PROJECT_DESIGN.md)，文件职责看 [PROJECT_MAP.md](PROJECT_MAP.md)，项目续接看 [CURRENT_STATE.md](CURRENT_STATE.md)，新实验记录和测试等级分别看 [实验规范](docs/experiment_run_standard.md)与[测试审计](docs/test_tiers_audit.md)；本文件只写工作门槛。用户本次任务授权优先于本文件的普通流程建议。

## 修改前

1. 读 `PROJECT_DESIGN.md`、`PROJECT_MAP.md` 和当前任务，找出所属 responsibility、现有文件 owner 与已批准操作契约。
2. 涉及机器人、SDK 或框架能力时，先查项目已有能力、官方 SDK 对应版本的语义与限制；不能只因为自写更快就重造。
3. 判定工作是 extend、refactor、replace 还是 remove。新需求放不进现有结构时，先检查旧责任边界是否错误；优先修正边界，不默认叠加 wrapper、manager、helper、adapter、common 或 v2。
4. 现有代码是当前行为证据，**不是自动批准的架构**；代码能工作也不证明边界合理。

## 设计变更与文件归属

- 未经当前任务明确授权，不自行改变 Public API、顶层模块、依赖方向、核心 abstraction、operation contract、环境治理或安全标准。
- 确需改变时先提交 `Design Change Proposal`：冲突事实、原因、影响、最简单替代与验证；等用户批准，先更新 `PROJECT_DESIGN.md` 和 `PROJECT_MAP.md`，再改代码。
- 每个正式实现文件只有一个主要 owner。新增/拆分前先在 `PROJECT_MAP.md` 登记 `File | Responsibility | Public/Internal | Allowed Dependencies`。批准后的模块、责任或依赖变化，必须同步维护两份设计文档，不能只改代码。
- `experiments` 可以依赖正式 Runtime；正式 Runtime 不得依赖 `experiments` 或 CLI。实验成功不能自动晋升正式能力；locked 入口不因重构或兼容解锁。

## 硬件与证据

- 未获任务明确授权，不连接硬件、不创建实机动作：enable、disable、运动、Drag、Replay、reset、软件急停测试；不改安全阈值或解锁历史入口。安全拒绝不能为了测试通过而降低门槛。
- 新 Run 按[实验规范](docs/experiment_run_standard.md)记录 `DOCUMENTED`、`CODE_CONFIRMED`、`OFFLINE_TESTED`、`HARDWARE_EXECUTED`、`HARDWARE_VERIFIED` 的具体命题、范围和缺项。代码存在、测试源码存在、命令执行或目标已发送，均不等于实机成功。
- 验证按[测试审计](docs/test_tiers_audit.md)的 T0～T4 实际副作用分级；导入、SDK 工厂或测试发现链未核清时为 `UNKNOWN_SIDE_EFFECT`，不能把历史整套 discover 命令当作已批准离线入口。
- 实时状态、环境和路线许可要按当前操作重新验证；未知状态不推定安全。危险清理动作不得藏在 catch/finally 中自动执行。

## 会话边界与交付

一个 session 围绕一个明确工作单元；完成后停止，不因尚有上下文而自动进入下一阶段。每次执行的关键尝试、失败/放弃原因、验证和续接进入对应持久报告或 Run Summary；有实质项目状态变化的执行者再更新 `CURRENT_STATE.md`，不向 `CONTINUE_PROMPT.md` 追加“最新状态”。交付说明依次写清：目标、实际修改、验证、未验证、新风险、剩余问题、下一步。若遇到材料性现场事实与批准设计冲突，记录事实、停止依赖冲突的工作并请求用户重新决定，不自行改写设计。
