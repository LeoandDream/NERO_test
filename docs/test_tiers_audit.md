# 测试副作用分级与静态候选清单

更新：2026-09-27；基线 `main@060d3491fd195542f8be0f20d6304f66ba66e7ed`，工作树含未跟踪设计文档和运行数据。**本轮只做 T0 静态检查，未运行测试、项目导入、测试发现或设备查询。** 此表是候选入口审计，不是可运行白名单；源码门禁和 Fake 不等于实际隔离已验证。每次后续执行仍需按 [实验规范](experiment_run_standard.md) 记录解释器、版本、范围和结果。

## 分级边界

| 等级 | 实际副作用边界 |
| --- | --- |
| T0 | 文件、文本/AST、Git/哈希；不导入项目代码。 |
| T1 | 已证实离线的纯函数/单元测试和替身，零实体设备连接与命令。 |
| T2 | 已隔离设备边界的多模块/工作流测试，零实体设备连接与命令。 |
| T3 | 经审定的实机查询或被动监听；查询发送须显式注明，不改模式、不发运动。 |
| T4 | 使能、失能、Drag、运动、回放、reset、软件急停等可能改变设备状态的操作。 |

`UNKNOWN_SIDE_EFFECT` 是待核状态，**不是**可自动运行的级别。层级按真实调用和依赖决定，不按文件名、`--help`、`status` 或 `discover` 决定。T3/T4 需要当前具体现场授权；本任务无此授权。

## 依赖与审计方法

用 AST 逐文件读取 34 个 `test_*.py` 的导入、`setUp`/`setUpClass`、工厂调用和补丁点；文本核对项目的 `robot_instance → AgxArmFactory.create_arm`、执行器 `can.Bus`/`connect` 路径。没有导入任何项目模块。34 文件中静态有 178 个 `test_*` 定义，**不是发现数或通过数**。`environment.yml` 固定 Python 3.10 和 pyAgxArm 提交 `841a625f5f4920e776f20b934eb13048b747e6d0`；本机 `nero-py310` 的 `pyagxarm-1.0.0.dist-info/direct_url.json` 静态记录同一提交。实际运行时解释器、其余依赖和源码仍须再核。

已静态核到 SDK `AgxArmFactory.create_arm()` 调用驱动构造器，驱动构造器再创建 `DriverContext`；该路径尚未穷尽 parser、模型和所有配置的初始化副作用。项目的另一常见链为测试 → `continuous_replay.continuous_session.robot_instance("can0")` → 同一 SDK 工厂；`lab` 测试又常经 `lab.api` → `lab.device` → `continuous_session` 导入这条链，执行器 `start_transfer.go_to_start` 有 `can.Bus` 与运动/急停分支。是否走到这些分支取决于 fixture 和 patch 作用域。即使某测试只调用 FK，也不据此判定构造绝无 CAN 副作用。动态导入和所有异常路径尚未完成运行时隔离证明。历史 `lab/report.md` 的 110 项通过属于当时的 `OFFLINE_TESTED` 记载，不是当前 HEAD/工作树的新通过证据。

下表路径均相对 `experiments/single_arm/`。`C`：项目导入/调用链待完整确认；`F`：真实 SDK 工厂构造，`UNKNOWN_SIDE_EFFECT`；`M`：Fake/patch 的范围和失败路径待核；`D`：读取仓库历史数据/配置，其身份和哈希须在运行时记录。`T1?`/`T2?` 是**预期目标**，问号表示尚未获准运行；出现 `F` 的整文件保持 `UNKNOWN_SIDE_EFFECT`。行内依据来自文件 AST 和所列测试调用，未声称每个测试方法同风险。

| 测试文件 | 预期等级 / 当前风险 | 静态依据与待核调用链 |
| --- | --- | --- |
| `can_setup/tests/test_drag_start_guard.py` | T2? / C,M | CLI `main` 与启动门禁，部分工厂 patch；核对所有拒绝与停止帧路径。 |
| `can_setup/tests/test_recover_emergency_stop.py` | T1? / C,M | Fake reset/状态；核对恢复模块导入及替身覆盖。 |
| `cartesian_x/tests/test_cartesian_x_session.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 直接 `AgxArmFactory.create_arm`，用于 FK。 |
| `continuous_replay/tests/test_analysis.py` | T2? / C,D | 导入分析器/连续会话，读固定记录并生成临时分析；核对导入链。 |
| `continuous_replay/tests/test_trajectory.py` | T2? / C,D | 固定源、计划重算和返回推导；`setUpClass` 读配置。 |
| `control_interfaces/tests/test_cartesian_linear_session.py` | UNKNOWN_SIDE_EFFECT / F,C,M | `setUpClass` SDK 工厂；CLI 与执行路径另有 patch，整文件不能视作离线。 |
| `control_interfaces/tests/test_publishers.py` | T1? / C,M | 发布器替身；核对项目导入和 publisher 构造。 |
| `lab/tests/test_approach_h0_j1.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，路线 FK。 |
| `lab/tests/test_cube_face_sweep.py` | T2? / C,M,D | 分面会话/CLI 与临时文件；核对模拟 CAN 边界。 |
| `lab/tests/test_cube_sweep.py` | T2? / C,M | API/CLI 与 Fake，检查 patch 作用域及退出路径。 |
| `lab/tests/test_edge_release.py` | T1? / C,M | 门禁/路线评估 patch，并导入同目录测试替身。 |
| `lab/tests/test_edge_y_release_current.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，路线 FK。 |
| `lab/tests/test_go_home_candidate.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，路线 FK。 |
| `lab/tests/test_high_y_home.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，锁定路线预检。 |
| `lab/tests/test_home_demo.py` | T2? / C,M | API/CLI 拖动演示替身；核对启动和异常收尾。 |
| `lab/tests/test_lab.py` | UNKNOWN_SIDE_EFFECT / F,C,M,D | 跨模块记录/CLI；两个测试方法直接 `robot_instance("can0").fk`，其余方法不能代表整文件。 |
| `lab/tests/test_low_pose_escape.py` | T1? / C,D | 静态路线门禁/拒绝，`setUp` 读配置；核对模块导入。 |
| `lab/tests/test_optimized_home.py` | UNKNOWN_SIDE_EFFECT / F,C,M,D | `setUpClass` 调 `robot_instance`；部分执行路径 patch。 |
| `lab/tests/test_planned_return.py` | UNKNOWN_SIDE_EFFECT / F,C,M,D | `setUpClass` SDK 工厂、路线 FK，部分 API patch。 |
| `lab/tests/test_quick_init_h0.py` | T2? / C,M,D | 命名入口、Mock/patch、临时计划；核对重新连接与重规划分支。 |
| `lab/tests/test_recover_h0_after_drag_jump.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` SDK 工厂、恢复路线 FK。 |
| `lab/tests/test_recover_teach_limit.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` SDK 工厂、限位恢复 FK。 |
| `lab/tests/test_release_h0_j1.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，路线 FK。 |
| `lab/tests/test_relieve_h0_contact.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，接触解除候选。 |
| `lab/tests/test_start_from_h0_candidate.py` | UNKNOWN_SIDE_EFFECT / F,C,D | `setUpClass` 调 `robot_instance`，失能起点路线。 |
| `lab/tests/test_supported_home.py` | T2? / C,M,D | 旧锁路线与 CLI Fake；核对 `connected` 替身完整性。 |
| `local_probe/tests/test_local_joint_probe.py` | T2? / C,M,D | 局部探测及 `execute_route` patch；核对 CAN Bus 与发布器分支。 |
| `replay/tests/test_replay_session.py` | T1? / C,D | 固定源与计划，`setUpClass` 读配置；核对旧模块导入。 |
| `start_transfer/tests/test_controller_joint_return.py` | T1? / C,D | 配置/既有反馈文件与静态规划；核对导入链。 |
| `start_transfer/tests/test_go_to_start.py` | T2? / C,M | `execute_route` 多分支用 Fake/patch；核对 CAN Bus、急停和异常路径。 |
| `start_transfer/tests/test_probe_j7_coupled.py` | T1? / C,D | `setUpClass` 读配置，关节候选；核对导入链。 |
| `start_transfer/tests/test_recover_to_s1.py` | T2? / C,M,D | 恢复路由及 Fake；核对所有连接/命令分支。 |
| `start_transfer/tests/test_urdf_origins.py` | UNKNOWN_SIDE_EFFECT / F,C,D | 测试方法直接 SDK 工厂，比较 URDF 与 FK。 |
| `teaching/tests/test_teach_session.py` | T2? / C,M,D | 示教/返回/CLI 的 Fake 与 patch；核对 `run_session` 异常及清理路径。 |

**当前入口结论**：`python -m unittest discover -s experiments/single_arm -p 'test_*.py' -q` 不能无条件称为离线安全命令，整套保持 `UNKNOWN_SIDE_EFFECT`，本轮没有运行。后续先对固定 SDK 版本和每个选中测试的导入、fixture、设备工厂、CAN 构造、异常路径做完整隔离审查，再在已授权的 T1/T2 任务中执行选定子集并留日志；未核完的文件不纳入该子集。若任何所谓查询会连接实机，则按 T3 审核；可能改变设备状态则按 T4 审核。

## Step 2C：首个实际验证的离线子集（2026-09-27）

本节追加新证据；上方 Step 2B 文字保留当时审计状态，不能把其中“本轮没有运行”读作 Step 2C 结论。范围是 `main@060d3491fd195542f8be0f20d6304f66ba66e7ed`，解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21，工作目录 `/home/leo/nero_dh116_ws`。执行时仓库还有前一阶段未提交/未跟踪文档，但本节被测 `test_publishers.py` 和 `publishers.py` 均与 HEAD 一致，SHA-256 分别为 `3dbb44e9a6ba482ccc4427248477cb45cc6a789a16d754e569c62a8abcf030ff` 和 `8e6430746983223a9f766e21c2af93886cdc3036db337c2e492b2de4fce07ac8`。完整运行记录见仓库外整改记录《06 离线测试边界验证 20260927 163256.md》。

| File | 静态分类与理由 | 导入链证据、fixture/调用 | 实际执行、命令与结果 | 证据等级、剩余风险 |
| --- | --- | --- | --- | --- |
| `control_interfaces/tests/test_publishers.py` | `SAFE_CANDIDATE`，T1：本地 Fake，测试文件和目标模块只导入标准库。 | `experiments` → `single_arm` → `control_interfaces` → `tests` 的 `__init__.py` 均空或仅文档字符串；测试 → `publishers.py` → `math`,`typing`。无 `setUp`、patch、动态导入、SDK/CAN/工厂；三种 `publish` 只调用传入的 `FakeRobot` 方法。 | **是**；`/home/leo/miniconda3/envs/nero-py310/bin/python -B -m unittest experiments.single_arm.control_interfaces.tests.test_publishers.PublisherTests -v`；2026-09-27 08:32:15 UTC，exit 0，2 pass / 0 fail / 0 error / 0 skip，未发现新文件。 | 对这两个明确测试为 `OFFLINE_TESTED`；不代表其他 publisher 调用路径或全套安全。 |
| `can_setup/tests/test_recover_emergency_stop.py` | `BLOCKED_BY_IMPORT`：虽仅调用纯门禁函数，模块顶层导入 SDK 和 CAN 相关项目模块。 | 测试 → `recover_emergency_stop.py` → `pyAgxArm`、`can_drag_teach.py` → `can`、`pyAgxArm`、`drag_start_guard.py`；本轮未证明这些外部导入的完整副作用。`setUp` 仅构造字典。 | **否**；无命令/结果。 | `CODE_CONFIRMED` 的阻塞原因；后续须核完整导入链，不能从函数纯度推断测试可运行。 |
| `start_transfer/tests/test_controller_joint_return.py` | `BLOCKED_BY_IMPORT`：文件门禁函数虽只读 JSON，目标模块顶层导入运动执行与 SDK/CAN 链。 | 测试 → `controller_joint_return.py` → `pyAgxArm`、`can_drag_teach.py`、`go_to_start.py`、`teach_session.py`；`setUp` 调 `load_config`，目标 `main` 内有 SDK 工厂/连接，导入与执行边界尚未完整核实。 | **否**；无命令/结果。 | `CODE_CONFIRMED` 的阻塞原因；不得把历史文件检查当安全执行许可。 |
| `start_transfer/tests/test_probe_j7_coupled.py` | `BLOCKED_BY_IMPORT`：测试前修改 `sys.path` 并导入实机脚本。 | 测试 → `probe_j7_coupled.py` → `can`、`pyAgxArm`、`go_to_start.py`、`teach_session.py`；其 `main` 可构造/连接机器人，外部导入链未核清。 | **否**；无命令/结果。 | `CODE_CONFIRMED` 的阻塞原因；未证明导入即触硬件，也未获准执行。 |
| `lab/tests/test_optimized_home.py` | `BLOCKED_BY_FACTORY`：`setUpClass` 调真实 `robot_instance`。 | 测试 fixture → `continuous_session.robot_instance` → `AgxArmFactory.create_arm`；测试运行会触到 SDK 工厂，不能因后续只做 FK 而视为离线。 | **否**；无命令/结果。 | `CODE_CONFIRMED` 的阻塞原因；本轮未审 SDK 构造全链。 |

该子集运行时未检测到 CAN 输出、SDK 设备创建、接口探测或非预期文件。`-B` 禁用新 `.pyc`；现存 `__pycache__` 的文件时间均早于本次运行。**Full test discovery 仍为 `UNKNOWN_SIDE_EFFECT`。** 未列入本节的 29 个文件沿用上表的 Step 2B 候选状态，不因这 2 个测试通过而升级。

## Step 3A：纯内存快照解析的显式 T1 测试（2026-09-27）

基线 HEAD `caf3c2bd9769546ac31064912fd3ba453eab7284`；以下结果针对该 HEAD **加本轮未提交工作树**，并非该提交已包含新代码。解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21，工作目录 `/home/leo/nero_dh116_ws`。`src/nero_runtime/__init__.py`、`src/nero_runtime/snapshot.py`、`tests/test_runtime_snapshot.py` 的 SHA-256 依次为 `5787847c8459d585874c0d75a41264cc3ca228e4d7c25f45de5a87af7afe87d8`、`1bc19cbfef69163c7804cf453048ca599d06689a0cf5cba89015b55e5e0d3efd`、`64f29039978a6e6c1c2f837980ee4f8af26fe801fb4d7ce896d0341c5229c9de`。

| File | 静态分类与依据 | 实际执行 | 结果与证据范围 | 剩余风险 |
| --- | --- | --- | --- | --- |
| `tests/test_runtime_snapshot.py` | `SAFE_CANDIDATE`，T1；执行前静态解析完整导入链：测试脚本 → `nero_runtime.__init__`（仅文档字符串）→ `snapshot.py` → 标准库 `dataclasses`、`math`、`typing`。测试本身仅导入标准库及新模块；fixture 为合成 dict/list，未调用旧项目模块、SDK、CAN 或设备工厂。唯一子进程明确运行 `python -B -S -c` 导入新包/模块并检查 `sys.modules`；无项目 CLI、动态设备导入或危险异常清理。 | **是**；`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v`；另用相同命令将 stdout/stderr 重定向至 `/tmp/nero_step3a_test_stdout.txt`、`/tmp/nero_step3a_test_stderr.txt` 保存原始输出。 | 两次显式运行各 15 tests，15 pass / 0 fail / 0 error / 0 skip；第二次 2026-09-27 09:37:20 UTC，exit 0，unittest 用时 0.023 s；stdout 0 字节，stderr 仅 15 个 `ok`、`Ran 15 tests`、`OK`。仅对上述字节与合成数据的状态解析及导入隔离为 `OFFLINE_TESTED`。 | 未接 SDK 或真实反馈；`time_basis` 由调用方保证，消息不构成原子同步快照；不覆盖旧 `read_state`、运动许可或整套测试。 |

测试覆盖完整/故障/失能反馈、缺失与坏格式、7 轴身份、过期/未来时间、未解释状态码、输入拷贝、单采样不产生停稳/FK/许可结论和独立进程导入隔离。未导入旧实验代码，未执行 full discover。**Full test discovery 仍为 `UNKNOWN_SIDE_EFFECT`。** 完整离线执行与文件保护记录见仓库外整改记录《08 状态快照内核实现与离线验证 20260927 173740.md》。

## 非 H0 低位候选的本次限定离线检查（2026-09-27）

本次仅对 `lab/planned_return.py` 的选定纯规划函数做 AST 抽取，在内存中以 Fake FK 和合成状态调用；没有 import 该项目模块或 SDK/CAN，也没有构造设备工厂。检查了已记录起点及前两目标后的姿态、偏离起点的拒绝、通用低 X 拒绝；另以注入的 `connected`、`read_state`、`execute_route` 模拟 `run`，确认现场许可缺失时零连接/零目标、有许可时只执行首目标并暂停。结论仅为这些合成分支的 `OFFLINE_TESTED`；实体碰撞、CAN 模式切换、真实到位和急停路径均未测试。随后用 `/home/leo/miniconda3/envs/nero-py310/bin/python -B` 对 `planned_return.py`、`api.py`、`cli.py` 文本执行 `compile(...)`，3/3 通过且无项目导入；`git diff --check` 通过。没有运行 `lab/tests/test_planned_return.py`，其 `setUpClass` 使用 SDK 工厂，继续保持 `UNKNOWN_SIDE_EFFECT`；整套 discover 仍未运行。

用户明确选择“一次命令跑完整路线”后，新增 CLI 显式 `--complete-low-route`。用 `/home/leo/miniconda3/envs/nero-py310/bin/python -B` 仅执行 `quick init --help` 验证参数可见；再在注入的内存 `connected/read_state/execute_route` 下对真实保存的阶段 1 计划模拟两次 `planned_return.run`：默认确认模式只回调 1 个目标并记录 `completed=false`，完整模式回调 5 个目标并记录 `completed=true`，两者均未调用真实设备工厂、CAN Bus 或指令发布器。该结果仅覆盖分支与报告语义；真实轨迹、异常急停和实体避障均未测试。没有执行项目整套测试。

## Step 3B：有限采集适配与快照组合（2026-09-27）

基线为 `main@caf3c2bd9769546ac31064912fd3ba453eab7284` 加当前未提交工作树。执行前 AST 核对 `src/nero_runtime/__init__.py` 无导入，`snapshot.py` 仅导入标准库，`acquisition.py` 仅导入标准库与 `.snapshot`；两个目标测试脚本只导入标准库及这些纯模块。新测试的合成源只在内存返回 `SimpleNamespace`，子进程仅以 `-B -S -c` 导入新包并检查未加载 `pyAgxArm`、`can`、`experiments`；无 SDK patch 时序、设备工厂、CAN 或控制异常清理。两个显式脚本均在该静态链下为 `SAFE_CANDIDATE`，实际运行后仅对各自命题为 T1 `OFFLINE_TESTED`。

| File | 实际命令与范围 | 结果及证据限定 | 剩余风险 |
| --- | --- | --- | --- |
| `tests/test_runtime_snapshot.py` | `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_step3b_snapshot_stdout.txt 2> /tmp/nero_step3b_snapshot_stderr.txt`；2026-09-27 13:35:17 UTC。 | exit 0，15 pass / 0 fail / 0 error / 0 skip；unittest 0.022 s。3A 的 `__init__.py`、`snapshot.py`、测试文件 SHA-256 仍分别为 `5787847c8459d585874c0d75a41264cc3ca228e4d7c25f45de5a87af7afe87d8`、`1bc19cbfef69163c7804cf453048ca599d06689a0cf5cba89015b55e5e0d3efd`、`64f29039978a6e6c1c2f837980ee4f8af26fe801fb4d7ce896d0341c5229c9de`。这是本轮另一次显式回归，不改变上节 Step 3A 原始证据。 | 仍只覆盖合成输入，不覆盖 SDK/真机。 |
| `tests/test_runtime_acquisition.py` | `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_step3b_acquisition_stdout.txt 2> /tmp/nero_step3b_acquisition_stderr.txt`；2026-09-27 13:35:25 UTC。 | exit 0，12 pass / 0 fail / 0 error / 0 skip；unittest 0.022 s。新 `acquisition.py` 与测试 SHA-256 分别为 `f873b1529de24753eac93d68251a85d3635087a96798883fc1d6ddc8a37c322e`、`f268bd56b7673bbca5ea660868bd0994e29a083e50c7024b8444810cfe733ec7`。测试覆盖固定读取顺序/次数、故障事实、缺失/过期/非法值、预期异常与中断、原始时间戳、复制、无许可/控制方法及独立进程导入隔离。 | SDK getter 的缓存并发修改、实际阻塞、真实时间基准和硬件均未验证。 |

工作目录 `/home/leo/nero_dh116_ws`，解释器 Python 3.10.21；两次 stdout 均 0 字节，stderr 仅为 unittest 的测试名、`ok`、计数与 `OK`，无非预期输出。完整原始输出与文件范围检查见仓库外整改记录《09 反馈采集适配与安全触发溯源 20260927 215426.md》。这些测试不累计为全套测试通过；**Full test discovery 仍为 `UNKNOWN_SIDE_EFFECT`**。
