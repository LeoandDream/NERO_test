# 单臂旧入口与 Runtime 迁移表

更新：2026-09-30，F8 静态依赖审计。此表分类**代码入口**，不晋升其历史运行结果、环境或操作许可。正式软件 owner 为 `src/nero_runtime/`；旧入口未因 F8 自动改接或解锁。当前真机适配与现场验证尚不足以安全删除旧控制路径。

| Legacy path | Old responsibility | Runtime replacement | Current status | Why retained / retired | Next field-validation dependency |
| --- | --- | --- | --- | --- | --- |
| `experiments/single_arm/lab/api.py` | `LabAPI` 汇集状态、点位/录制、示教、回放、起点、回位和探测。 | F1～F7 各责任模块；无同名总对象。 | `ACTIVE_EXPERIMENT` | `lab/cli.py` 和旧测试仍实际调用；若直接转发会绕过尚未验证的硬件适配与旧专用门禁。 | 各命令逐项核对真实 adapter、现场许可和新旧结果语义，再迁调用者。 |
| `experiments/single_arm/lab/cli.py` | Quick 人工确认、预览、旧实验执行入口。 | `nero_runtime.cli` 只读/离线入口；运动 CLI 未晋升。 | `ACTIVE_EXPERIMENT` | `LabAPI` 的真实调用者；旧菜单与专用实验无法由当前 Runtime CLI 完整替代。 | 先完成相应能力的 Field Validation，再逐命令退役；不得打开旧 locked 菜单。 |
| `experiments/single_arm/lab/device.py` | 旧连接、状态和设备侧 `ready_for_motion`。 | F1 `device`/`acquisition`/`snapshot`。 | `RUNTIME_CONSUMER_CANDIDATE` | 旧 `lab` 仍调用，且反向借用 `continuous_replay` 工厂；状态语义未逐项迁移。 | 真实 V121 只读连接、反馈时间基与多帧缓存核对。 |
| `experiments/single_arm/lab/planned_return.py` | 从实时姿态生成回 S1 **候选**、专用低位识别、旧计划与执行锁。 | F7 只处理已登记的显式 Return READY 路线；不提供任意姿态规划器。 | `ACTIVE_EXPERIMENT` | `teach_session.py` 和多条 `lab` 专用脚本仍导入 `_assess`/`require_return_execution_basis`；Step 3C 候选锁与历史低位证据不可删除。 | 单独核定每条现场低位路线及扫过区；禁止把候选和 X 条件当通用许可。 |
| `experiments/single_arm/lab/optimized_home.py` | S1↔新 H0 专用路线及历史运行/故障证据。 | F7 命名 Start/Park 的结构，尚无现站点 validated H0/路线。 | `HARDWARE_EVIDENCE_ONLY` | `LabAPI`、`starts.py` 和部分旧测试仍引用；曾有特定现场完成记录，但不能据此晋升 PARK。 | 重新核对当前安装、工具、桌边、线缆、承托与双向扫过区。 |
| `experiments/single_arm/lab/supported_home.py` | 旧 H0 往返路线，读取 `route_blocked` 并拒绝。 | 无可执行 Runtime 对应物；F7 可表达 locked 路线，但没有迁入有效资产。 | `LOCKED` | 旧 `supported_home.json` 记有碰撞及阻断依据；`LabAPI`、旧测试和专用脚本仍引用。 | 事故路线独立复核与新的路线设计；不得借迁移解除锁定。 |
| `experiments/single_arm/teaching/teach_session.py` | 真实 S1 拖动、CSV 采样、停止/回位及报告。 | F4/F5/F7 离线内核组合；尚无真实 V121 Drag exit/Return adapter。 | `ACTIVE_EXPERIMENT` | `continuous_replay`、`start_transfer`、`lab`、Cartesian 实验等仍导入其校验/配置；历史 Ctrl+C 与回位语义不同于未来 v1。 | 真实 Drag enter/exit 反馈、记录时基、Return 路线与取消/故障验收。 |
| `experiments/single_arm/start_transfer/go_to_start.py` | 实机路线发布、稳定反馈和中断处理。 | F2 `motion` 顺序执行内核与 F7 Start 计划。 | `RUNTIME_CONSUMER_CANDIDATE` | `continuous_replay`、`lab`、控制接口与其他实验仍调用其反馈/执行函数；Runtime 尚无真实发布/停止 adapter。 | `move_j` 模式/速度/钳制、逐轴刷新、停止映射及站点路线实测。 |
| `experiments/single_arm/continuous_replay/` | 旧 CSV 连续曲线、运行与诊断/恢复报告。 | F3 JSONL 记录和 F6 **关节路点** Replay；不等同高保真定时流。 | `ACTIVE_EXPERIMENT` | `lab/device.py` 借用 `robot_instance`，其他实验复用流式停止/轨迹函数；历史源和失败证据不可直接转换。 | 源格式/时序单独设计、真实发布/停止与源资格核验后再迁。 |
| `experiments/single_arm/pose_recording/` | 实机 CSV 位姿采样、候选提取与工作空间分析。 | F1 快照/点位与 F3 JSONL 记录只覆盖部分责任。 | `RUNTIME_CONSUMER_CANDIDATE` | CSV/法兰分析与历史数据解释仍有独立价值，格式不兼容；未发现无风险的完整替代。 | 真实只读采集、采样率与历史 CSV 语义比对。 |
| `experiments/single_arm/can_setup/` | CAN 建链、直接帧、拖动起点锁、监测和受监护恢复。 | F1 只读来源与 F4 模式注入内核；无正式 reset/enable/stop adapter。 | `HARDWARE_EVIDENCE_ONLY` | `drag_start_guard`、恢复报告、监测和直接帧是当前唯一部分实机风险证据；不得改成通用控制捷径。 | 只读现场核验、V121 Drag 模式与真实退出、受监护恢复单独评估。 |
| `experiments/single_arm/control_interfaces/` | 关节/笛卡尔目标发布、IK 与 `move_l`/`move_p` 试验。 | F2 只覆盖已给定的关节路点；Cartesian/IK 尚无正式能力。 | `ACTIVE_EXPERIMENT` | `publishers.py` 等仍服务 Cartesian 实验，`move_l`/`move_p` 的控制器中间路径未知；不是可删除重复实现。 | 专门验证笛卡尔路径/环境/发布与反馈；不能由 F2 关节路点结果外推。 |
| `experiments/single_arm/lab/recordings.py`、`lab/replay.py`、`lab/smooth_replay.py` | 旧固定目录记录、资格校验与实验 Replay。 | F3/F6 提供新 JSONL 和关节路点候选/执行内核。 | `RUNTIME_CONSUMER_CANDIDATE` | `LabAPI` 仍调用，旧格式与资格/速度模式不同；不能把旧 recording 自动解释为 F3 合格记录。 | 历史资产只读转换规范、现场 Replay 路线许可及真实适配。 |

## 去重和退役判断

新功能只在 Runtime owner 继续维护：`snapshot/acquisition` 管解析与单轮采集，`waypoints` 管新点位 JSON，`motion` 管正式关节路点执行，`recording` 管新 JSONL，`replay` 管其回放候选，`environment` 管新 Profile，`workflows` 管命名操作。旧代码继续按上表承担实验/证据角色，不得在文档中称为已晋升正式实现；F8 不建立兼容层。

静态调用搜索显示 `lab.cli → LabAPI`、`teach_session → planned_return`、`lab.device → continuous_replay.robot_instance`、`continuous_replay → go_to_start/teach_session`、Cartesian/探测脚本 → `can_setup` 等活跃关系。没有一项待删旧入口同时满足“完整替代、无活跃调用者、无唯一现场证据、无原始数据解释损失、对应 Runtime 测试、无需猜硬件语义”六项条件，故本轮**删除旧代码 0 个，改接旧入口 0 个**。原始 JSON/CSV/报告保持原样。未来逐入口迁移应在对应 Field Validation 后重新核对六项条件。
