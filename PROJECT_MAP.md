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

### v1 Runtime 文件责任与使用面

下表登记 Step 3A～F7 已形成的 Runtime 文件责任及 F8 使用面；`nero_runtime` 包名不表示真实机器人控制能力已晋升。只有 `__init__.__all__` 中的纯数据/文件/计划接口为顶层 public；执行函数须从对应子模块显式导入并注入受信适配。

F1 功能依赖：官方固定版本 SDK 的显式连接/反馈 → `collect_snapshot`/`RobotSnapshot` → 单条命名点位 JSON → 薄 CLI。旧连续 CSV 录制器和 `lab.api` 状态入口保留其原职责，本轮不迁移或改写。

F2 功能依赖：F1 点位 JSON 或显式七轴 rad → `motion.prepare_joint_route` → 上层显式前置检查/合成设备发布与反馈/停止依赖 → `motion.execute_joint_route`。CLI 只增加离线 `route show`，不连接真实发布或旧实验执行入口；旧调用方仍未切换。

F3 功能依赖：显式反馈来源 → F1 `collect_snapshot`/`RobotSnapshot` → `recording.record_trajectory` → 新 JSONL 原始序列和摘要。CLI 仅在显式 `trajectory record` 时打开 F1 来源；show/list 纯文件。F3 不依赖 Motion、Drag、Teach 或 Replay，不筛掉故障/失能/负 X 事实。

F4 功能依赖：上层显式许可及反馈/模式请求 callable → F1 `AcquisitionResult`/`RobotSnapshot` → `drag.enter_drag`/`inspect_drag_state`/`exit_drag`。固定 V121 SDK 的 `set_normal_mode()` 是无动作接口，故本轮不建立真实退出适配或普通 Drag CLI；请求边界只在注入的合成设备上验证，不能解除旧独立 Drag 锁定。

F5 功能依赖：调用方已完成的上层许可/注入来源 → F4 确认进入 → F3 流式记录 → F4 显式退出/反馈确认 → F1 单轮终点观测。`teaching.py` 仅组合已有函数并表达阶段、取消/故障和证据；不导入 Motion、不规划 Return READY、不接真实 V121 模式适配或普通 Teach CLI。

F6 功能依赖：F3 已保存的 JSONL/摘要 → `replay.prepare_replay` 完整性及候选资格校验 → 不可变关节路点计划 → F2 `JointRoute`/`execute_joint_route`。Replay 不决定现场运动许可，不保留记录时间节奏；CLI 仅可离线展示候选，不提供真实执行入口。

F7 功能依赖：F1 点位原件及哈希 → 版本化 Environment Profile 中的点位/定向路线资产 → `workflows.py` 根据当前身份/反馈选择 Start、Park 或显式 Return READY 计划 → F2 `execute_joint_route`。环境状态为该版本声明，不自动证明现场适用；工作流不自行使能、失能或连接 SDK，真实动作仍缺现场证据和上层授权。

F8 使用面：`__init__.py` 惰性导出纯数据、存储、候选/计划与查询函数；执行函数继续通过其已登记子模块显式导入，要求调用方注入前置检查、反馈、发布/模式请求及停止映射。惰性包导入不装配设备。白名单 runner 仅调用下表已经审计的 Runtime 测试；旧实验路径分类见 [迁移表](docs/legacy_runtime_migration.md)。

| 需求 | 子功能 | 基本能力来源 | 复用/新增位置 | 离线验收 |
| --- | --- | --- | --- | --- |
| 看真实反馈 | 显式打开一次只读来源，采集并展示诊断 | 固定 SDK Nero V121 连接/getter；3A/3B 解析 | 新 `device.py` 生命周期；复用 `acquisition.py`/`snapshot.py`；新 `cli.py` 展示 | 合成来源全链、异常关闭、只读导入隔离 |
| 记一个命名点 | 用一次快照捕获七轴与来源质量 | `RobotSnapshot` 的关节质量/时间 | 新 `waypoints.py`；CLI capture | 非 READY/负 X 仍记录，缺失/过期/坏值拒绝 |
| 保存、重看、列举 | 一条普通 JSON，原件不覆盖 | 标准库 JSON/文件系统 | 新 `waypoints.py`；CLI show/list | 往返、重名、损坏/未知版本、无 SDK 展示 |

| File | Responsibility | Public/Internal | Allowed Dependencies |
| --- | --- | --- | --- |
| `src/nero_runtime/__init__.py` | Public Python surface：惰性导出明确列出的纯数据、文件与计划接口；包导入本身不装配或发现设备。 | Public selected exports | 仅 Python 标准库；只有访问导出名称时才导入已登记纯 Runtime 模块；不得导入 SDK/CAN/experiments。 |
| `src/nero_runtime/snapshot.py` | Device：调用方提供的内存反馈解析、来源时间与数据质量；不采集、不判动作许可。 | Public selected data / internal parser | 仅 Python 标准库；不得依赖 SDK/CAN/experiments/CLI。 |
| `tests/test_runtime_snapshot.py` | Offline Verification：合成反馈与纯模块导入隔离的 T1 测试。 | Internal test | Python 标准库和 `nero_runtime.snapshot`；不得导入旧实验模块或 SDK/CAN。 |
| `src/nero_runtime/acquisition.py` | Device：从调用方提供的反馈源各读取一次、复制所需数据并交给现有快照解析；记录读取窗口与预期读取失败，不管理连接或许可。 | Internal | Python 标准库、`nero_runtime.snapshot`；不得导入 SDK/CAN/experiments/CLI。 |
| `tests/test_runtime_acquisition.py` | Offline Verification：合成反馈源的次数、异常、时基、复制及采集到解析组合测试。 | Internal test | Python 标准库、`nero_runtime.acquisition`/`snapshot`；不得导入真实 SDK/CAN/旧实验模块。 |
| `src/nero_runtime/device.py` | Device：固定 Nero V121 SDK 的显式工厂与连接生命周期；只在调用方请求时导入 SDK、打开与关闭，不发布控制指令。 | Internal | Python 标准库、调用时固定版本 `pyAgxArm`；不得依赖 experiments、CLI、运动或点位存储。 |
| `src/nero_runtime/waypoints.py` | Environment + Safety 的观测资产：从现有快照建立命名关节点位、校验并以单条 JSON 保存/读取/列举；不授予路线或运动许可。 | Public selected pure/file API | Python 标准库、`nero_runtime.snapshot`；不得依赖 SDK/CAN/experiments/CLI。 |
| `src/nero_runtime/cli.py` | CLI：只读 status、waypoint capture/show/list、离线 route show、trajectory record/show/list、replay show、environment show 的参数、组合和展示；不复制解析、记录存储或运动逻辑。 | Internal entry | Python 标准库及上述 Runtime 模块；只有显式 status/capture/trajectory record 才调用 `device.py`，show/list 与 route/replay/environment show 不连接或发布目标。 |
| `tests/test_waypoints.py`、`tests/test_readonly_entry.py` | Offline Verification：合成来源到真实采集/存取/分派与导入、关闭边界。 | Internal test | Python 标准库及 `nero_runtime`；不得导入真实 SDK/CAN 或旧实验链。 |
| `src/nero_runtime/motion.py` | Motion：准备调用者指定的七轴关节路线；在上层显式提供前置检查、单点发布、实时反馈、取消及停止处理后，顺序发布并按新反馈验收到位，返回尝试/发布/到达与停止证据。不连接或配置设备，不授予现场许可。 | Public selected data/plan; advanced injected execution | Python 标准库、`nero_runtime.snapshot`、`nero_runtime.waypoints`；不得依赖 SDK/CAN、experiments 或 CLI。 |
| `tests/test_motion.py` | Offline Verification：F1 点位至路线准备及合成设备顺序执行、拒绝/取消/故障/停止证据与离线 CLI 预览的 T2 测试。 | Internal test | Python 标准库与 `nero_runtime`；不导入 SDK/CAN 或旧实验模块。 |
| `src/nero_runtime/recording.py` | Teaching / Recording：从调用方提供的 F1 `AcquisitionResult` 按时长/频率流式保存每轮原始快照及读取异常，生成和验证摘要；仅结束采样，不执行 Drag、运动或 Replay 许可。 | Public selected data/read; advanced injected recording | Python 标准库、`nero_runtime.acquisition`/`snapshot`；不得依赖 SDK/CAN、experiments、Motion 或 CLI。 |
| `tests/test_recording.py` | Offline Verification：合成来源经真实 F1 单轮采集至 F3 JSONL/摘要的 T2 正反向测试，含取消、写入故障、摘要校验和离线 CLI。 | Internal test | Python 标准库与 `nero_runtime`；不得导入真实 SDK/CAN 或旧实验模块。 |
| `src/nero_runtime/drag.py` | Device：调用方已批准后的 Drag enter/inspect/exit 模式请求与反馈证据、取消及退出回退；不选择现场许可、记录、路线或真实 SDK 适配。 | Advanced injected execution only | Python 标准库、`nero_runtime.acquisition`/`snapshot`；不得依赖 SDK/CAN、experiments、Motion、Recording 或 CLI。 |
| `tests/test_drag.py` | Offline Verification：真实 F1 采集与 F4 内核在合成模式请求/反馈上的正反向 T2 验证。 | Internal test | Python 标准库与 `nero_runtime.acquisition`/`snapshot`/`drag`；不得导入真实 SDK/CAN/旧实验模块。 |
| `src/nero_runtime/teaching.py` | Robot API / Workflows：组合确认 Drag、F3 轨迹记录、显式退出及终点观测，保留阶段与完整结果；不是完整 Teach v1。 | Advanced injected execution only | Python 标准库、`nero_runtime.drag`/`recording`/`acquisition`/`snapshot`；不得依赖 Device 真实来源、SDK/CAN、Motion、CLI 或 experiments。 |
| `tests/test_teaching.py` | Offline Verification：F1→F4→F3→F4→F1 的真实合成全链与取消、故障、退出未确认及原件保留 T2 测试。 | Internal test | Python 标准库与 `nero_runtime` 纯注入模块；不得导入真实 SDK/CAN 或旧实验链。 |
| `src/nero_runtime/replay.py` | Teaching / Recording：核对 F3 原件并提取有序关节路点候选，组合 F2 路线与执行结果；不授予现场许可或另建运动循环。 | Public selected data/plan; advanced injected execution | Python 标准库、`nero_runtime.recording`/`motion`；不得依赖 SDK/CAN、Device、Drag、CLI 或 experiments。 |
| `tests/test_replay.py` | Offline Verification：F1→F3→F6→F2 合成全链、原件篡改与故障/取消边界 T2 测试。 | Internal test | Python 标准库与 `nero_runtime` 纯注入模块；不得导入真实 SDK/CAN 或旧实验链。 |
| `src/nero_runtime/environment.py` | Environment + Safety：严格读取/保存单文件版本化 Profile，核对 F1 点位原件哈希、身份和定向路线引用；不推断现场许可或生成运动。 | Public selected data/file API | Python 标准库、`nero_runtime.waypoints`；不得依赖 SDK/CAN、Motion、CLI 或 experiments。 |
| `tests/test_environment.py` | Offline Verification：合成 F1 点位至 Profile 存储/校验、篡改、身份与引用边界 T2 测试。 | Internal test | Python 标准库、`nero_runtime.snapshot`/`waypoints`/`environment`；不得导入真实 SDK/CAN 或旧实验链。 |
| `src/nero_runtime/workflows.py` | Robot API / Workflows：根据显式环境身份与实时快照准备 Start/Park/Return READY，组合 F5 终点与 Return，并委托 F2 执行；不提供真实适配或通用规划。 | Public selected data/plan; advanced injected execution | Python 标准库、`nero_runtime.environment`/`motion`/`snapshot`/`teaching`；不得依赖 SDK/CAN、CLI 或 experiments。 |
| `tests/test_workflows.py` | Offline Verification：F1 原件→F7 Profile/命名计划→F2 FakeRig 执行及 F5 合成终点组合 T2 测试。 | Internal test | Python 标准库与 `nero_runtime` 纯注入模块和已审合成测试辅助；不得导入真实 SDK/CAN 或旧实验链。 |
| `tests/test_public_surface.py` | Offline Verification：惰性 package export、隔离导入、Runtime 依赖方向及状态/迁移边界 T1/T2 测试。 | Internal test | Python 标准库与 `nero_runtime`；不导入真实 SDK/CAN 或旧实验模块。 |
| `tests/test_runtime_cli.py` | Offline Verification：CLI 帮助、离线分支设备隔离和未晋升命令缺席 T2 测试。 | Internal test | Python 标准库、纯 Runtime 与已审合成测试辅助；不导入真实 SDK/CAN 或旧实验模块。 |
| `scripts/test_runtime_offline.sh` | Offline Verification：只按已审白名单逐文件执行 Runtime 测试，失败即停止；不发现旧实验测试。 | Internal runner | POSIX shell、当前 Python、`src/nero_runtime` 与显式 `tests/test_*.py` 白名单；不得使用 discover 或连接设备。 |
