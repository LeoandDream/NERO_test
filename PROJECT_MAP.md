# NERO 项目职责地图

本文件定位**当前代码在哪里、以后责任归谁**。批准的操作语义和安全边界见 [PROJECT_DESIGN.md](PROJECT_DESIGN.md)。当前路径只代表 `main@060d3491fd195542f8be0f20d6304f66ba66e7ed` 的静态代码事实，不表示这些模块已经晋升正式 Runtime；下表也不预设未来文件路径。

## 当前架构地图

状态词：`active` 为当前实验主流程，`experimental` 为受条件限制的试验，`locked` 为执行拒绝，`historical` 为证据/旧说明，`migration-candidate` 为后续可评估提取。一个文件可同时是 active 与 migration-candidate；**不得凭迁移候选状态解锁**。

| 当前路径 | Current Responsibility | Known Problem | Future Responsibility Owner | Current Status |
| --- | --- | --- | --- | --- |
| `experiments/single_arm/lab/api.py` | `LabAPI` 汇集状态、示教、回放、起点、立方体与探测。 | 对外接口和业务分流混在实验包；旧接口没有新契约保证。 | Robot API / Workflows，具体设备、许可、执行分别交其 owner。 | active; migration-candidate |
| `experiments/single_arm/lab/cli.py` | quick/子命令、确认、显示和部分路线/流程分流。 | CLI 含业务选择；部分旧拖动 quick 仍有菜单但会被门禁拒绝。 | CLI 只管 Human Flow；语义归 Workflows，许可归 Environment + Safety。 | active; migration-candidate；独立拖动 quick locked |
| `experiments/single_arm/lab/device.py` | 连接工厂、状态读取、设备侧 `ready_for_motion`。 | 工厂反向导入 `continuous_replay`；设备侧就绪不等于现场许可。 | Device；逐操作许可归 Environment + Safety。 | active; migration-candidate |
| `experiments/single_arm/teaching/teach_session.py` | S1 限时拖动采样、退出、从实时姿态回 S1、报告。 | 录制阶段 Ctrl+C 仍可能进入自动回位；模式、采样、返回和许可耦合。 | Workflows 协调 Drag/Teach；Device 管模式；Teaching / Recording 管样本；Motion 管运动。 | active experimental; migration-candidate |
| `experiments/single_arm/lab/recordings.py`、`complete_recording.py` | 新记录来源/哈希/完成与回位凭证校验。 | 固定目录、整配置哈希、V121 和停止原因绑定，历史原件不能直接迁移。 | Teaching / Recording；环境适用性由 Environment + Safety 判定。 | active; migration-candidate |
| `experiments/single_arm/lab/replay.py`、`smooth_replay.py` | 逐点/连续回放准备、校验、执行与报告。 | 回放与现有执行器、实验目录耦合；源资格和单次运行需继续分开。 | Workflows 管 Replay；Teaching / Recording 管源资格；Motion 管发布/反馈。 | active experimental; migration-candidate |
| `experiments/single_arm/lab/planned_return.py` | 从实时姿态生成回 S1 关节**候选**，按可选显式法兰条件做有限采样；新旧计划分版本，执行前识别候选与旧假设。 | `X ≥ 0.15 m` 与“X 非负”无通用批准或标定依据；候选、局部法兰检查与整条路线执行许可必须分开。低位完整路线的桌边/线缆实体验收仍缺。 | Environment + Safety 管经证据核定的站点路线/许可；Motion 管执行。 | active experimental；Step 3C 通用执行待补环境证据；migration-candidate |
| `experiments/single_arm/lab/optimized_home.py` | 当前 S1↔新 H0 专用路线与执行，保持使能。 | 新 H0 仅特定安装受监护候选，不能默认为通用 PARK。 | Environment + Safety 管站点路线；Workflows 管 Park/Start；Motion 管执行。 | active experimental; migration-candidate |
| `experiments/single_arm/lab/start_from_h0_candidate.py` | 已失能、精确 H0 条件下单次使能并到 S1。 | 特例散落在实验入口；未来只许 validated PARK 使用。 | Workflows 管 Start 阶段；Environment + Safety 管 D5 条件；Device 管使能；Motion 管运动。 | experimental; migration-candidate |
| `experiments/single_arm/lab/disable_at_h0_candidate.py` | H0 轻触候选现场确认后单次软件失能。 | 当前现场观察不能单靠角度自动证明承托；不是物理断电。 | Workflows 管 Disable；Environment + Safety 管 PARK/支撑前置；Device 管指令。 | experimental; migration-candidate |
| `experiments/single_arm/can_setup/drag_start_guard.py`、`can_drag_teach.py` | S1 新鲜令牌拖动授权与直接帧；独立启动无授权即拒绝，停止可单独请求。 | 低位模式切换有历史风险；底层帧不能因新设计被绕开。 | Environment + Safety 管许可；Device 管模式帧；Workflows 管独立 Drag。 | 独立拖动 locked; migration-candidate |
| `experiments/single_arm/can_setup/recover_emergency_stop.py` | 默认只读观察；显式受监护 reset。 | reset 后可能失电下落，不能进入普通 Quick 恢复。 | 高级 Workflows 管恢复；Device 管单次 reset 与状态。 | experimental / supervised; migration-candidate |
| `experiments/single_arm/start_transfer/go_to_start.py`、`continuous_replay/run_stream.py` | 关节目标发布、时序/反馈/异常停止。 | 存在多个实验运动执行路径，统一入口确认不能覆盖所有直达脚本。 | Motion 成为单次运动唯一发布 owner；Device 提供 I/O。 | active experimental; migration-candidate |
| `experiments/single_arm/lab/cube.py`、`workspace_cube/`、`reachability/` | 几何采样/拟合、极值和路线探测证据。 | 几何候选没有完整运动区域许可；六面结果未完成。 | Environment + Safety 的标定数据与证据工具；运动探测仍归 Motion/许可。 | experimental; migration-candidate |
| `config/nero_teach.json`、`experiments/single_arm/lab/config/starts.json` | S1、安装和实验参数；命名点绑定配置哈希。 | 环境与实验参数混存，旧记录依赖原件。 | Environment + Safety 的未来 Profile 输入；历史配置保持原样。 | active historical inputs |
| `experiments/single_arm/lab/config/supported_home.json`、`lab/supported_home.py`、`lab/high_y_home.py` | 旧 H0 路线和历史直接入口。 | 旧路线曾有桌边风险，`route_blocked` / 执行锁不得消失。 | Environment + Safety 保留 locked 证据；不作为可用路线迁移。 | locked; historical |
| `README.md`、`docs/quickstart_cli.md`、`TASK_PROMPT.md`、`CONTINUE_PROMPT.md`、各实验报告与 `data/` | 当前/历史使用说明、任务、运行及失败证据。 | 文档日期和入口状态不一致；历史记录不可当当前实时许可。 | 文档/证据责任按批准设计逐项更新；原始历史数据保留。 | historical / active references |

Step 2B 的 [CURRENT_STATE.md](CURRENT_STATE.md) 是唯一可替换的当前**项目**状态摘要，由造成实质变化的执行者维护；[实验规范](docs/experiment_run_standard.md)由新 Experiment/Run 执行者使用并维护相应记录；[测试审计](docs/test_tiers_audit.md)由执行验证者在依赖或入口变化后更新。三者是文档/证据责任，不是新的 Runtime 模块、依赖或现场许可。`CONTINUE_PROMPT.md` 仅保留历史正文，不再接收新检查点。

当前已知反向链：`lab/device.py → continuous_replay/continuous_session.py → start_transfer/go_to_start.py → teaching/teach_session.py`，示教代码又延迟导入 `lab`。它是需要分阶段消除的**现状**，不是允许的正式依赖。历史记录固定路径和哈希在迁移前须单独设计读取策略；当前不建立泛化兼容层。

## 目标责任地图

| Responsibility | Owns | Does Not Own |
| --- | --- | --- |
| Robot API / Workflows | start、drag、teach、replay、park、disable 等操作语义、阶段和结果 | CAN、环境数值、CLI 提示 |
| Device | 官方 SDK 与必要底层 I/O、设备快照、模式/使能/失能/软件急停指令 | 路线、用户 workflow、示教数据 |
| Motion | 唯一运动目标发布、反馈、超时、中止与结果 | 用户意图、环境验证 |
| Environment + Safety | 版本化站点事实、点位/路线、锁定状态、逐操作许可 | CAN 命令、自动晋升 |
| Teaching / Recording | 样本、原始数据/哈希、完整性、回放资格与证据 | 运动许可和运动目标发布 |
| CLI | 显示、输入、具体风险确认、准备时间与结果展示 | 安全策略、路线算法、运动逻辑 |

## 依赖与文件责任规则

- 允许：`CLI → Runtime`、`Experiments → Runtime`、`Future Integrations → Runtime`。正式 Runtime 内由 Workflows 调用各责任，Device 只面对 SDK/硬件；Motion 接收已核准目标并经 Device 发布。
- 禁止：`Runtime → Experiments`、`Runtime → CLI`；Device 依赖 Teaching/Replay/Experiments；Motion 自行选择用户意图或批准 Environment；CLI 另写安全/路线/运动逻辑。
- **每个正式实现文件必须有一个主要责任 owner。** 新增或拆分正式文件前先在本图登记 `File | Responsibility | Public/Internal | Allowed Dependencies`，同时核对 [PROJECT_DESIGN.md](PROJECT_DESIGN.md) 的职责和依赖。若改变架构，先获用户批准并更新两份设计文档，再创建代码。
- `new_manager.py`、`common2.py`、`utils_new.py`、`lab_v2/` 等仅凭泛名且无上述责任记录的新结构不得直接出现。职责可以由现有文件承担；不要求为表中每一行建立类或目录。

### 未来正式文件登记表

Step 3A/3B 只批准下列内部状态解析、有限采集及对应离线测试；`nero_runtime` 包名不表示公共 SDK 已发布。后续文件仍须先登记。

| File | Responsibility | Public/Internal | Allowed Dependencies |
| --- | --- | --- | --- |
| `src/nero_runtime/__init__.py` | Device：纯包标识，不装配或发现设备。 | Internal | 无项目依赖；不得导入 SDK/CAN/experiments。 |
| `src/nero_runtime/snapshot.py` | Device：调用方提供的内存反馈解析、来源时间与数据质量；不采集、不判动作许可。 | Internal | 仅 Python 标准库；不得依赖 SDK/CAN/experiments/CLI。 |
| `tests/test_runtime_snapshot.py` | Offline Verification：合成反馈与纯模块导入隔离的 T1 测试。 | Internal test | Python 标准库和 `nero_runtime.snapshot`；不得导入旧实验模块或 SDK/CAN。 |
| `src/nero_runtime/acquisition.py` | Device：从调用方提供的反馈源各读取一次、复制所需数据并交给现有快照解析；记录读取窗口与预期读取失败，不管理连接或许可。 | Internal | Python 标准库、`nero_runtime.snapshot`；不得导入 SDK/CAN/experiments/CLI。 |
| `tests/test_runtime_acquisition.py` | Offline Verification：合成反馈源的次数、异常、时基、复制及采集到解析组合测试。 | Internal test | Python 标准库、`nero_runtime.acquisition`/`snapshot`；不得导入真实 SDK/CAN/旧实验模块。 |
