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

## Step 3C：通用回位候选边界的限定离线验证（2026-09-28）

基线为 `main@e7af8e86126eb0fd9c8b92e02014f281316f784d` 加本轮工作树；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21，工作目录 `/home/leo/nero_dh116_ws`。新测试只证明合成 FK、显式候选条件及隔离的 API/CLI 拒绝链。完整哈希、原始输出、文件保护范围见仓库外《10 X 假设整改与离线验证》。

| File | 静态分类与导入/fixture 审计 | 实际执行与结果 | 证据与剩余风险 |
| --- | --- | --- | --- |
| `tests/test_return_constraint_scope.py` | `SAFE_CANDIDATE`，T2。测试在导入 `experiments.single_arm.lab` **之前**预装拒绝型 `lab.recordings` 替身：原包入口会经 `recordings` 导入 `teach_session` 和 SDK，不能直接导入。实际链为测试 → `experiments`/`single_arm` 空包 → `lab.__init__` → 被隔离的 `recordings`，再到真实 `lab.device`（顶层仅标准库）与 `planned_return`（顶层仅标准库）。fixture 用 Fake FK、合成状态、临时 JSON；将非本轮责任的关节与 workspace 检查注入替身，未 patch 本轮候选算法、X 判断或执行门禁。子进程在导入真实 `api`/`cli` **之前**预装其它非目标项目模块和设备拒绝替身，调用真实 API/CLI 分派并检查 `pyAgxArm`、`can` 未加载。`run` 读临时计划后先拒绝，不进入工厂；异常和 fixture 清理无控制动作。 | **是**；`PYTHONPATH="$PWD" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_return_constraint_scope.py -v > /tmp/nero_step3c_test_stdout.txt 2> /tmp/nero_step3c_test_stderr.txt`；9 pass / 0 fail / 0 error / 0 skip，exit 0。 | 仅列出的候选、显式条件、旧计划拒绝、内部专用路线数值检查和部分 API/CLI 分支为 `OFFLINE_TESTED`。直接 `teach_session` 的 SDK 导入链仅静态核对；真实 CLI/SDK、完整执行、连续路径与实物碰撞未测。 |
| `tests/test_runtime_snapshot.py` | `SAFE_CANDIDATE`，T1；测试 → `nero_runtime.__init__` → `snapshot.py` → 标准库，独立子进程仅检查导入隔离；无设备 fixture。五个 3A/3B 相关文件 SHA-256 与既有报告一致。 | **是，显式回归**；`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_step3c_snapshot_stdout.txt 2> /tmp/nero_step3c_snapshot_stderr.txt`；15 pass / 0 fail / 0 error / 0 skip，exit 0。 | 仅纯解析合成数据为 `OFFLINE_TESTED`，不是本次规划测试。 |
| `tests/test_runtime_acquisition.py` | `SAFE_CANDIDATE`，T1；测试 → `nero_runtime.acquisition` → `.snapshot` → 标准库，fixture 仅合成源；无 SDK/CAN/设备工厂。 | **是，显式回归**；`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_step3c_acquisition_stdout.txt 2> /tmp/nero_step3c_acquisition_stderr.txt`；12 pass / 0 fail / 0 error / 0 skip，exit 0。 | 仅有限合成采集为 `OFFLINE_TESTED`，未验证 SDK 多帧缓存的新鲜度或真实设备。 |
| `experiments/single_arm/lab/tests/test_planned_return.py` 等旧实验测试 | `BLOCKED_BY_FACTORY` / `UNKNOWN_SIDE_EFFECT`；旧 fixture 构造 SDK robot，部分旧断言还使用历史 `0.15` 签名。 | **否**；未运行，未做整套发现。 | 旧测试源码不因本轮新用例而升级；需另行隔离和迁移。 |

首次新测试命令未设置 `PYTHONPATH`，在导入前报 `ModuleNotFoundError: experiments`，exit 1；没有进入 fixture 或设备链。纠正路径后的明确命令才形成上述证据。三条成功命令均用 `-B -S`；stdout 均为 0 字节，stderr 仅 unittest 名称、`ok`、计数与 `OK`。`/tmp` 原始输出供核对；未见 SDK/CAN/网络迹象，仓库原始实验 JSON 的起始清单 331/331 哈希未变。**Full test discovery 仍为 `UNKNOWN_SIDE_EFFECT`。**

## F1：状态读取与命名观测点位（2026-09-28）

基线：`main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加 F1 未提交工作树；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21，工作目录 `/home/leo/nero_dh116_ws`。执行前确认新测试仅导入标准库及 `nero_runtime`：`tests/test_waypoints.py → snapshot/waypoints`；`tests/test_readonly_entry.py → cli → acquisition/snapshot、device、waypoints`。`device.py` 顶层仅导入标准库，真实 `pyAgxArm` 导入位于显式 `connected_nero()` 且两个合成 factory 必须同时注入；测试的真实组合调用均注入 FakeRobot，设备控制方法为拒绝型替身。临时 JSON 只写标准库临时目录；异常、KeyboardInterrupt 与连接失败均检查 `disconnect`，无动作清理。子进程只运行 `-B -S` 的包导入和 `--help`。因此新测试为隔离 T2 `SAFE_CANDIDATE`；真实 SDK 来源只做静态核对，**未**在本轮执行或升级为硬件证据。

| 显式命令 | 实际结果 | 证据边界 |
| --- | --- | --- |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v > /tmp/nero_f1_waypoints_stdout.txt 2> /tmp/nero_f1_waypoints_stderr.txt` | exit 0；5 pass / 0 fail / 0 error / 0 skip；0.014 s | 真正的捕获/JSON 写入/读取/列举；故障、失能、负 X 的非误拒绝；缺失、长度错误、NaN/Inf、过期、重名、损坏/未知 schema。仅合成快照 `OFFLINE_TESTED`。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v > /tmp/nero_f1_entry_stdout.txt 2> /tmp/nero_f1_entry_stderr.txt` | exit 0；5 pass / 0 fail / 0 error / 0 skip | 合成工厂的 `connect`/getter/`disconnect` → 真正 `collect_snapshot` → 点位保存与人可读 show/list；参数错误先拒绝、读取失败/中断清理、离线导入无 SDK/CAN。真实 SDK 工厂未运行。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_f1_snapshot_stdout.txt 2> /tmp/nero_f1_snapshot_stderr.txt` | exit 0；15 pass / 0 fail / 0 error / 0 skip；0.023 s | 3A 合成解析独立回归；T1。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_f1_acquisition_stdout.txt 2> /tmp/nero_f1_acquisition_stderr.txt` | exit 0；12 pass / 0 fail / 0 error / 0 skip；0.025 s | 3B 合成采集独立回归；T1。 |

新用例和两份回归均只对列明命题为 `OFFLINE_TESTED`；最终原始输出与文件 SHA-256 见仓库外《F1 状态与点位功能交付》。stdout 均 0 字节，stderr 仅 unittest 测试名/计数/`OK`；未发现 SDK/CAN/网络或非预期仓库文件。SDK Nero V121 `connect()` 的整体墙钟超时、后台线程退出、真实反馈时基/多帧缓存仍待现场核验，不能因合成测试通过声称设备已验证。**Full test discovery 继续为 `UNKNOWN_SIDE_EFFECT`。**

## F1 审阅修正：只读状态与命名点位收尾（2026-09-28）

沿用 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加未提交 F1 工作树；本节是修正后证据，不覆盖上节的原始 37 次运行。外部审阅所述 Python 3.13.5 独立容器 37/37 是另一环境的记录，本节没有据此升级本环境证据。静态导入链仍为 `test_waypoints → snapshot/waypoints → 标准库`、`test_readonly_entry → cli → acquisition/snapshot、device、waypoints → 标准库`；`device` 只有显式真实来源分支才导入 `pyAgxArm`。新测试注入双 factory/FakeRobot、假时钟与 sleep，独立子进程只做导入和 help；无真实 SDK、CAN、控制方法或项目旧实验导入。3A/3B 两份测试仍为纯标准库/合成数据链。四文件执行前均为 `SAFE_CANDIDATE`（F1 T2；3A/3B T1）。

工作目录 `/home/leo/nero_dh116_ws`；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21；最终文件执行区间 2026-09-28 15:47:42～15:51:46 UTC（entry 补充断言并保留原 Fake 默认值后单独复跑）。每条命令均显式指定单个测试文件，没有 discover。stdout 各 0 字节；stderr 仅 unittest 用例名、`ok`、计数与 `OK`，原始输出和源码哈希收入仓库外《12 F1 审阅修正与验证》及同序号审阅 ZIP。

| 文件与精确命令 | 结果 | `OFFLINE_TESTED` 的限定范围、剩余风险 |
| --- | --- | --- |
| `tests/test_waypoints.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v > /tmp/nero_f1_review_waypoints_stdout.txt 2> /tmp/nero_f1_review_waypoints_stderr.txt` | exit 0；6 pass / 0 fail / 0 error / 0 skip；0.015 s | R4 的 valid 来源缺/负/不一致年龄拒绝、浮点差与历史时间读回；仅合成 JSON，不证明真实反馈时钟。 |
| `tests/test_readonly_entry.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v > /tmp/nero_f1_review_entry_stdout.txt 2> /tmp/nero_f1_review_entry_stderr.txt` | exit 0；10 pass / 0 fail / 0 error / 0 skip；0.112 s | R1 首轮无数据、截止部分/全缺、可选来源缺失即返回、capture 等待与取消；R2 真/假/未知逐轴显示；R3 已知连接 RuntimeError、关闭失败、主因+关闭因、取消+关闭因及未知编程错误原样抛出。仅 Fake 生命周期，不证明真实 SDK 连接/关闭上界。 |
| `tests/test_runtime_snapshot.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_f1_review_snapshot_stdout.txt 2> /tmp/nero_f1_review_snapshot_stderr.txt` | exit 0；15 pass / 0 fail / 0 error / 0 skip；0.023 s | 3A 纯解析回归；未改 `snapshot.py`。 |
| `tests/test_runtime_acquisition.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_f1_review_acquisition_stdout.txt 2> /tmp/nero_f1_review_acquisition_stderr.txt` | exit 0；12 pass / 0 fail / 0 error / 0 skip；0.024 s | 3B 单轮采集回归；未改 `acquisition.py`。 |

最终仅上述 43 个显式用例在该解释器、工作树和命令下为 `OFFLINE_TESTED`。真实 status/capture、SDK 工厂、现场反馈及完整测试发现均未运行；**Full test discovery = `UNKNOWN_SIDE_EFFECT`**。

## F2：指定关节路点内部执行与离线预览（2026-09-29）

基线为 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加未提交 F1 文件和本轮 F2 文件；F1 原件哈希见《12 F1 审阅修正与验证》，本轮变化及完整清单见仓库外《13 F2 指定路点执行与离线验证》。执行前静态链：`tests/test_motion.py → nero_runtime.motion → snapshot/waypoints → 标准库`；`test_motion.py → nero_runtime.cli → motion、acquisition/snapshot、device、waypoints → 标准库`。`device.py` 的 SDK 导入只位于未调用的显式真实来源分支；route show 在设备分支之前返回。测试使用内存 FakeRig、临时 F1 JSON、可注入单调时钟/sleep 和独立 `-B -S` 导入/help 子进程；Fake 只有发布目标/停止替身，没有 SDK/CAN/旧实验导入或真实设备工厂。`test_motion.py` 为 T2 `SAFE_CANDIDATE`；F1 两项仍为 T2，3A/3B 两项为 T1。

解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21；工作目录 `/home/leo/nero_dh116_ws`；最终文件执行区间 2026-09-28 16:45:51～16:47:34 UTC（motion 加强第 2 点发布异常计数断言后单独复跑）。各命令 exit 0，stdout 0 字节，stderr 仅 unittest 用例名、`ok`、计数与 `OK`；原始输出收入第 13 号 ZIP。没有 full discover、硬件 CLI、真实 SDK 实例/连接或控制动作。

| 显式命令 | 实际结果 | 限定证据与未覆盖项 |
| --- | --- | --- |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_motion.py -v > /tmp/nero_f2_motion_stdout.txt 2> /tmp/nero_f2_motion_stderr.txt` | 10 pass / 0 fail / 0 error / 0 skip；0.094 s | F1 临时 JSON 保存/读回→真实路线准备→真实内核→Fake 逐点发布/多次反馈→完成；末点非法、单位/身份冲突、输入修改、起点错、前置拒绝/预取消、状态/驱动故障、缺失/过期/重复反馈、表面 idle 但未到位、首次和第 2 点发布异常的计数、运动中取消/KeyboardInterrupt、超时与停止失败；CLI 纯离线预览。只对合成路径为 `OFFLINE_TESTED`。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v > /tmp/nero_f2_waypoints_stdout.txt 2> /tmp/nero_f2_waypoints_stderr.txt` | 6 pass / 0 fail / 0 error / 0 skip；0.010 s | F1 点位文件正反向回归；真实点位来源未测。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v > /tmp/nero_f2_readonly_entry_stdout.txt 2> /tmp/nero_f2_readonly_entry_stderr.txt` | 10 pass / 0 fail / 0 error / 0 skip；0.129 s | F1 状态/采集/点位 CLI 合成回归；未调用真实 status/capture。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_f2_runtime_snapshot_stdout.txt 2> /tmp/nero_f2_runtime_snapshot_stderr.txt` | 15 pass / 0 fail / 0 error / 0 skip；0.024 s | 3A 纯解析回归。 |
| `PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_f2_runtime_acquisition_stdout.txt 2> /tmp/nero_f2_runtime_acquisition_stderr.txt` | 12 pass / 0 fail / 0 error / 0 skip；0.024 s | 3B 合成采集回归。 |

这五个指定文件合计 **53/53**；仅对上述 HEAD、工作树哈希、解释器与命令范围授予 `OFFLINE_TESTED`。F1 组合时间戳不能证明真实七轴逐帧刷新；SDK `move_j` 的模式设置、关节限位处理及真实停止路径尚未接入/核实；站点适用路线和逐操作许可未完成。旧实验运动入口未切换，完整测试发现仍为 **`UNKNOWN_SIDE_EFFECT`**。

## F3：独立轨迹记录的限定离线验证（2026-09-29）

基线 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加保留的 F1/F2 未提交工作树及本轮 F3 文件；暂存区为空。工作目录 `/home/leo/nero_dh116_ws`；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21。执行前静态核对：`test_recording → recording → acquisition → snapshot → 标准库`；CLI 测试还经 `cli → device/waypoints/motion/recording`，这些模块的顶层不构造 SDK/CAN 设备，`device.connected_nero` 只有未调用的默认真实工厂分支才导入 `pyAgxArm`。测试中的 `trajectory record` 注入两个内存工厂和 `FakeSource`；show/list 仅打开临时文件；子进程仅导入模块或运行 help。旧四份回归测试的 F1/3A/3B 导入链不变；F2 `test_motion` 仍只用合成设备。六个文件均为执行前 `SAFE_CANDIDATE`：新 F3 和 F1/F2 为隔离 T2，3A/3B 为 T1。无真实 SDK 工厂、CAN 接口、设备状态查询或控制动作。

最终显式运行开始于 2026-09-29 12:26:51 UTC；每条 exit 0、stdout 0 字节、stderr 仅 unittest 用例名/`ok`/计数/`OK`，没有非预期输出或仓库生成文件。原始输出及完整哈希见仓库外《14 F3 独立轨迹记录与离线验证》同序号审阅 ZIP。

| 文件与精确命令 | 实际结果 | `OFFLINE_TESTED` 的限定范围和剩余风险 |
| --- | --- | --- |
| `tests/test_recording.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_recording.py -v > /tmp/nero_f3_recording_stdout.txt 2> /tmp/nero_f3_recording_stderr.txt` | 11 pass / 0 fail / 0 error / 0 skip；0.146 s | FakeSource → 真 F1 单轮采集 → 真记录循环 → 多行 JSONL/摘要 → 哈希/计数读回；stale、缺失、故障、失能、负 X 与读取错误保留，取消/故障/时钟倒退/部分写入与冲突、CLI 注入及离线 show/list。真实采样节拍、SDK 多帧时间基与设备退出未验证。 |
| `tests/test_motion.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_motion.py -v > /tmp/nero_f3_motion_stdout.txt 2> /tmp/nero_f3_motion_stderr.txt` | 10 pass / 0 fail / 0 error / 0 skip；0.113 s | F2 合成路线回归；不代表运动实机验收。 |
| `tests/test_waypoints.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v > /tmp/nero_f3_waypoints_stdout.txt 2> /tmp/nero_f3_waypoints_stderr.txt` | 6 pass / 0 fail / 0 error / 0 skip；0.011 s | F1 临时点位 JSON 回归；真实反馈未测。 |
| `tests/test_readonly_entry.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v > /tmp/nero_f3_readonly_entry_stdout.txt 2> /tmp/nero_f3_readonly_entry_stderr.txt` | 10 pass / 0 fail / 0 error / 0 skip；0.160 s | F1 status/capture 合成 CLI 回归；真实连接未调用。 |
| `tests/test_runtime_snapshot.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_f3_runtime_snapshot_stdout.txt 2> /tmp/nero_f3_runtime_snapshot_stderr.txt` | 15 pass / 0 fail / 0 error / 0 skip；0.025 s | 3A 内存解析回归；不证明真实反馈同步。 |
| `tests/test_runtime_acquisition.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_f3_runtime_acquisition_stdout.txt 2> /tmp/nero_f3_runtime_acquisition_stderr.txt` | 12 pass / 0 fail / 0 error / 0 skip；0.025 s | 3B 合成单轮采集回归；SDK 缓存新鲜性未验证。 |

合计 **64/64**，只对上述 HEAD 加本工作树哈希、Python 3.10.21、六条显式命令和合成命题为 `OFFLINE_TESTED`。关键新字节 SHA-256：`recording.py` 为 `0227290ad6d002515be69600b60ebc046dad00542784ea410ea59329e9055fc0`，`cli.py` 为 `0c21841c2acf05fd1c28283a034819d9aac00be39f661e926e6e392e4087615a`，`test_recording.py` 为 `76fb33cc914a9182faf59b0facc92692bf87217544cf958638f624570b243db6`。无 full discover；旧实验测试未因 F3 升级，**Full test discovery 继续为 `UNKNOWN_SIDE_EFFECT`**。

## F4：可注入 Drag 模式内核的限定离线验证（2026-09-29）

基线 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加保留的 F1/F2/F3 未提交工作树及 F4 新文件；暂存区为空。工作目录 `/home/leo/nero_dh116_ws`，解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21。执行前静态链：`test_drag → drag → acquisition → snapshot → 标准库`。F4 不导入 `device`、SDK/CAN、Recording、Motion、CLI 或旧 experiments；测试用 `FakeNero` 交给真实 F1 `collect_snapshot`，以合成 callable 请求模式、预测状态、提供时钟。子进程只导入 F4 并检查未加载 SDK/CAN/旧实验/Recording/Motion。既有六份回归导入链见 F3 节且源码未因 F4 改变，均继续为执行前 `SAFE_CANDIDATE`；F4/F1/F2/F3 为隔离 T2，3A/3B 为 T1。`set_normal_mode()` 的真实 V121 继承实现为 no-op，因此本轮**没有**调用或导入真实 SDK，也没有真实 Drag 适配/CLI。

最终显式运行开始于 2026-09-29 13:47:49 UTC；每条 exit 0、stdout 0 字节、stderr 仅 unittest 名称/`ok`/计数/`OK`。测试只在临时目录内使用合成文件，未见设备或 CAN 副作用。原始输出及完整工作树哈希收入仓库外《15 F4 Drag Mode 与离线验证》同序号 ZIP。

| 文件与精确命令 | 实际结果 | `OFFLINE_TESTED` 的限定范围、剩余风险 |
| --- | --- | --- |
| `tests/test_drag.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_drag.py -v > /tmp/nero_f4_drag_stdout.txt 2> /tmp/nero_f4_drag_stderr.txt` | 16 pass / 0 fail / 0 error / 0 skip；0.032 s | 真 F1 采集→F4 enter→Fake leader 请求一次→新合成 active 反馈→inspect→exit→Fake normal 请求一次→新合成 inactive 反馈；故障/失能/关节缺失过期在控制前拒绝，负 X 不拒绝，模式请求失败/超时/取消/位移显式上限触发单次退出回退，退出未确认仍标 unconfirmed。真实 SDK 退出 API 与零力状态码未核实。 |
| `tests/test_recording.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_recording.py -v > /tmp/nero_f4_recording_stdout.txt 2> /tmp/nero_f4_recording_stderr.txt` | 11 pass；0.168 s | F3 合成记录回归；F4 未导入或调用 Recorder。 |
| `tests/test_motion.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_motion.py -v > /tmp/nero_f4_motion_stdout.txt 2> /tmp/nero_f4_motion_stderr.txt` | 10 pass；0.113 s | F2 合成运动回归；F4 不发布目标。 |
| `tests/test_waypoints.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v > /tmp/nero_f4_waypoints_stdout.txt 2> /tmp/nero_f4_waypoints_stderr.txt` | 6 pass；0.032 s | F1 点位 JSON 回归。 |
| `tests/test_readonly_entry.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v > /tmp/nero_f4_readonly_entry_stdout.txt 2> /tmp/nero_f4_readonly_entry_stderr.txt` | 10 pass；0.181 s | F1 合成来源/CLI 回归。 |
| `tests/test_runtime_snapshot.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_f4_runtime_snapshot_stdout.txt 2> /tmp/nero_f4_runtime_snapshot_stderr.txt` | 15 pass；0.025 s | 3A 内存解析回归。 |
| `tests/test_runtime_acquisition.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_f4_runtime_acquisition_stdout.txt 2> /tmp/nero_f4_runtime_acquisition_stderr.txt` | 12 pass；0.025 s | 3B 合成单轮采集回归。 |

合计 **80/80**，只对本 HEAD 加本工作树文件哈希、指定解释器、七条显式命令及合成命题为 `OFFLINE_TESTED`。F4 新文件 SHA-256：`drag.py` `269dab5ad4c78d9d75f1449722fdcb1aa74f097c15c5119fce3dbf5b3ee56e71`，`test_drag.py` `b3d7d4fc0aba46fca2b80117c9f2fd0cf25cb02caf78d3b818a52668584b112d`。无 full discover；旧实验测试、真实 SDK 反馈和独立 Drag 入口未升级，**Full test discovery 继续为 `UNKNOWN_SIDE_EFFECT`**。

## F5：Guided Recording Core 限定离线验证（2026-09-29）

基线 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加保留的 F1～F4 未提交工作树及 F5 新文件；暂存区为空。工作目录 `/home/leo/nero_dh116_ws`；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21。执行前静态链：`test_teaching → teaching → drag/recording → acquisition → snapshot → 标准库`；新测试另导入已审 `test_drag` 的合成 Fake/时钟，该模块只导入标准库及 F1/F4。F5 不导入 Device 真实来源、SDK/CAN、Motion、CLI 或旧 experiments；子进程只导入 `nero_runtime.teaching` 并检查这些模块未加载。既有七份回归的导入、fixture 与设备工厂隔离见 F4 节，生产依赖未因 F5 修改。全部八文件执行前为 `SAFE_CANDIDATE`；F5/F1～F4 为隔离 T2，3A/3B 为 T1。

最终八条显式命令同批开始于 2026-09-29 15:04:20 UTC；每条 exit 0、stdout 0 字节、stderr 仅 unittest 测试名/`ok`/计数/`OK`。测试产物只在临时目录；没有 SDK/CAN、网络或真实设备调用。原始输出与完整文件哈希收入仓库外《16 F5 Guided Recording Core 与离线验证》同序号 ZIP。

| 文件与精确命令 | 实际结果 | `OFFLINE_TESTED` 的限定范围、剩余风险 |
| --- | --- | --- |
| `tests/test_teaching.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_teaching.py -v > /tmp/nero_f5_teaching_stdout.txt 2> /tmp/nero_f5_teaching_stderr.txt` | 16 pass / 0 fail / 0 error / 0 skip；0.153 s | 真 F1 采集→真 F4 enter→真 F3 JSONL/摘要→真 F4 exit→新终点反馈。参数/重名/进入拒绝均零记录；取消、读故障、Drag 丢失、控制器/驱动故障均保留已写样本并尝试一次退出；退出失败/未确认不报完成；终点缺失/过期不伪造。仅 Fake 模式请求和合成反馈。 |
| `tests/test_drag.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_drag.py -v > /tmp/nero_f5_drag_stdout.txt 2> /tmp/nero_f5_drag_stderr.txt` | 16 pass；0.030 s | F4 模式内核合成回归；真实 V121 退出适配仍缺。 |
| `tests/test_recording.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_recording.py -v > /tmp/nero_f5_recording_stdout.txt 2> /tmp/nero_f5_recording_stderr.txt` | 11 pass；0.177 s | F3 流式记录回归；`effective_rate_hz` 仍为样本数/整段实际时长。 |
| `tests/test_motion.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_motion.py -v > /tmp/nero_f5_motion_stdout.txt 2> /tmp/nero_f5_motion_stderr.txt` | 10 pass；0.120 s | F2 合成运动回归；F5 未导入或调用 Motion。 |
| `tests/test_waypoints.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v > /tmp/nero_f5_waypoints_stdout.txt 2> /tmp/nero_f5_waypoints_stderr.txt` | 6 pass；0.021 s | F1 点位 JSON 回归。 |
| `tests/test_readonly_entry.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v > /tmp/nero_f5_readonly_entry_stdout.txt 2> /tmp/nero_f5_readonly_entry_stderr.txt` | 10 pass；0.196 s | F1 合成来源/CLI 回归。 |
| `tests/test_runtime_snapshot.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v > /tmp/nero_f5_runtime_snapshot_stdout.txt 2> /tmp/nero_f5_runtime_snapshot_stderr.txt` | 15 pass；0.026 s | 3A 纯内存解析回归。 |
| `tests/test_runtime_acquisition.py`：`PYTHONPATH="$PWD/src" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v > /tmp/nero_f5_runtime_acquisition_stdout.txt 2> /tmp/nero_f5_runtime_acquisition_stderr.txt` | 12 pass；0.026 s | 3B 合成单轮采集回归。 |

合计 **96/96**，仅对该 HEAD 加本工作树字节、Python 3.10.21、八个显式命令和合成命题为 `OFFLINE_TESTED`。F5 新字节 SHA-256：`teaching.py` `5e4752c20e4176497bac147dc112fec54e6662ef19effa93861a2fad0370b8e2`，`test_teaching.py` `e96d9cf3b51665320061ee3d8b1975e722fdf322c30e6451c77572f0e073d115`。无 full discover；旧实验测试、真实 V121 Drag 退出/反馈、Return READY 与正式 Teach 均未升级。**Full test discovery 继续为 `UNKNOWN_SIDE_EFFECT`。**

## F6：Replay Core 限定离线验证（2026-09-29）

基线 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加保留的 F1～F5 未提交成果及 F6 文件；暂存区为空。工作目录 `/home/leo/nero_dh116_ws`；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21。执行前静态链：`test_replay → replay → recording / motion → acquisition / snapshot / waypoints → 标准库`；新测试复用已审 `test_recording` 的 FakeSource/FakeTime 和 `test_motion` 的 FakeRig/feedback，二者还导入 CLI → device，但 `device` 顶层只导入标准库，SDK 工厂只在未调用的 `connected_nero()` 内部。新测试 fixture 仅为合成反馈、时钟和临时目录；不构造真实 Driver、CAN 或 SDK。CLI `replay show` 在任何来源连接分支之前完成。既有八份回归的导入/fixture 链见 F5 节；因 CLI 新增纯 Replay 导入，回归的顶层链多一条上述纯模块链，不触达 SDK。九文件执行前均为 `SAFE_CANDIDATE`；F6/F1～F5 为隔离 T2，3A/3B 为 T1。

九条显式命令于 2026-09-29 15:47:40 UTC 起逐一执行；各 exit 0、stdout 0 字节，stderr 仅 unittest 名称/`ok`/计数/`OK`；0 fail、0 error、0 skip。产物只在临时目录；无 CAN、SDK 设备创建、网络或真实机器人迹象。原始流、开始时间和墙钟耗时见仓库外第 17 号审阅 ZIP 的 `test_runs.json` 及同名 stdout/stderr。

| 文件与精确命令 | 实际结果 | `OFFLINE_TESTED` 范围及剩余风险 |
| --- | --- | --- |
| `tests/test_replay.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_replay.py -v` | 14 pass；exit 0；0.303 s | 真实 F1→F3 JSONL/摘要→F6 候选/路线→F2 合成反馈执行；故障、取消、篡改、负 X、起点和前置检查拒绝。未验证真实来源/发布/停止。 |
| `tests/test_teaching.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_teaching.py -v` | 16 pass；exit 0；0.145 s | F5 合成 Guided Recording 回归；真实 Drag 适配未验证。 |
| `tests/test_drag.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_drag.py -v` | 16 pass；exit 0；0.067 s | F4 合成 Drag 回归；真实 V121 退出未验证。 |
| `tests/test_recording.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_recording.py -v` | 11 pass；exit 0；0.242 s | F3 合成 Recording/原件回归；真实采样未验证。 |
| `tests/test_motion.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_motion.py -v` | 10 pass；exit 0；0.157 s | F2 合成路点执行回归；真实发布/停止未验证。 |
| `tests/test_waypoints.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v` | 6 pass；exit 0；0.043 s | F1 点位 JSON 回归。 |
| `tests/test_readonly_entry.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v` | 10 pass；exit 0；0.210 s | F1 合成来源/CLI 回归；真实连接未运行。 |
| `tests/test_runtime_snapshot.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v` | 15 pass；exit 0；0.057 s | 3A 纯内存解析回归。 |
| `tests/test_runtime_acquisition.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v` | 12 pass；exit 0；0.060 s | 3B 合成单轮采集回归。 |

`src/nero_runtime/replay.py` SHA-256：`1702bf1fb89490bdd898b0fad6b6387391320f35c50dc40c97eb43e09c95e972`。`tests/test_replay.py` SHA-256：`d8dc99d2a915287080ae5ce48cc7840ca9019de835f5b3b3dc9373397f23a4a2`。合计 **110/110**，仅对该 HEAD 加本轮工作树字节、Python 3.10.21、九个显式命令及合成命题为 `OFFLINE_TESTED`。负 X 法兰值本身不拒绝候选，记录 elapsed 仅保留来源证据而不作执行节拍。原件 SHA-256 是同目录完整性校验，不是外部真实性认证。**Full test discovery 继续为 `UNKNOWN_SIDE_EFFECT`**。

## F7：Environment 与命名工作流限定离线验证（2026-09-30）

基线 `main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加保留的 F1～F6 未提交成果及 F7 文件；暂存区为空。工作目录 `/home/leo/nero_dh116_ws`；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` 3.10.21。执行前静态链：`test_environment → environment → waypoints → snapshot → 标准库`，其离线 CLI 测试额外经 `cli → device`，SDK 只在未调用的 `connected_nero()` 中延迟导入；`test_workflows → workflows → environment/motion/teaching → waypoints/drag/recording/acquisition/snapshot → 标准库`。新工作流测试复用已审 `test_environment/test_motion/test_drag/test_teaching` 的纯 Fake/时钟，未调用真实 SDK factory 或 CAN；全部 fixture 限合成反馈和临时文件。CLI 新增 `environment show` 位于设备来源分支之前；既有九份回归仅多出 CLI→environment 纯导入链。十一文件在执行前为 `SAFE_CANDIDATE`；F7/F1～F6 为隔离 T2，3A/3B 为 T1。

最终 11 条显式命令于 2026-09-30 12:03:16 UTC 起逐一执行；各 exit 0、stdout 0 字节，stderr 仅 unittest 用例名/`ok`/计数/`OK`；0 fail、0 error、0 skip。预期生成物仅在临时目录及仓库外审阅包；未见 SDK/CAN、网络、真实设备或非预期仓库文件。原始输出、开始时间与墙钟耗时收入仓库外第 18 号审阅 ZIP。

| 文件与精确命令 | 实际结果 | `OFFLINE_TESTED` 范围及剩余风险 |
| --- | --- | --- |
| `tests/test_environment.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_environment.py -v` | 5 pass；exit 0；0.157 s | F1 合成点位→版本 Profile、SHA/身份/引用严格核对与离线 CLI；无真实环境批准。 |
| `tests/test_workflows.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_workflows.py -v` | 14 pass；exit 0；0.323 s | F1→Profile→F7 Start/Park/Return→真实 F2 FakeRig，以及真实 F5 合成终点→Return；候选/锁定/失能/错配/取消/故障等。 |
| `tests/test_replay.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_replay.py -v` | 14 pass；exit 0；0.269 s | F6 合成 Replay 回归；未接真实发布。 |
| `tests/test_teaching.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_teaching.py -v` | 16 pass；exit 0；0.153 s | F5 合成 Guided Recording 回归；真实 Drag 退出未证实。 |
| `tests/test_drag.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_drag.py -v` | 16 pass；exit 0；0.066 s | F4 合成 Drag 回归。 |
| `tests/test_recording.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_recording.py -v` | 11 pass；exit 0；0.232 s | F3 合成轨迹记录回归。 |
| `tests/test_motion.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_motion.py -v` | 10 pass；exit 0；0.164 s | F2 合成运动回归；真实 stop 未证实。 |
| `tests/test_waypoints.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_waypoints.py -v` | 6 pass；exit 0；0.045 s | F1 点位原件回归。 |
| `tests/test_readonly_entry.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_readonly_entry.py -v` | 10 pass；exit 0；0.234 s | F1 来源/CLI 合成回归。 |
| `tests/test_runtime_snapshot.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_snapshot.py -v` | 15 pass；exit 0；0.059 s | 3A 内存解析回归。 |
| `tests/test_runtime_acquisition.py`：`PYTHONPATH="$PWD/src:$PWD/tests" /home/leo/miniconda3/envs/nero-py310/bin/python -B -S tests/test_runtime_acquisition.py -v` | 12 pass；exit 0；0.062 s | 3B 合成单轮采集回归。 |

`src/nero_runtime/environment.py` SHA-256：`8b76715cc6e91dd4fbc10aabbe0ef42a031b42ee7b499ed5f3577bce4e26a0ec`。`src/nero_runtime/workflows.py` SHA-256：`ac32468bf97810a3e82c17162664d813e21520638ca8ba99853c346d1c6f1664`。`tests/test_environment.py` SHA-256：`d9a62617a6e445f7731d6faaa770312c204afeb988fe48a56ce1a9ff07d568fe`。`tests/test_workflows.py` SHA-256：`d42c478fc7c405a674ee95c336b3f3d13b6b5be5b64744782c788f886505a234`。合计 **129/129**，仅对该 HEAD 加本工作树字节、指定 Python、11 个显式命令和合成命题为 `OFFLINE_TESTED`。本轮未创建现实站点 validated Profile；合成测试中的 validated 仅为 fixture 声明。无 full discover；真实 SDK 运动、Drag、停止、enable、现场 Environment/路线核定及现场 Teach/Replay 均未升级。**Full test discovery 继续为 `UNKNOWN_SIDE_EFFECT`**。

## F8：Runtime 收口白名单回归（2026-09-30）

执行前静态审计 `src/nero_runtime/*.py` 与本节 13 个显式测试文件及其跨测试导入。`nero_runtime.__init__` 仅建立惰性导出表；运行时源文件不导入 `experiments`，不引用旧实验绝对路径或旧 JSON 默认配置。`device.connected_nero()` 内的 `pyAgxArm` 工厂是**函数内延迟导入**，白名单测试中触及它的 F1/F3 用例均注入合成 config/driver factory；CLI 离线分支在设备来源分支前返回。其余 `motion/drag/teaching/replay/workflows` 执行回调由 Fake 注入，测试的 setUp/fixture 只构造合成反馈、时钟及临时资产。`test_runtime_cli` 使用 `test_environment/test_motion/test_replay` 的已审合成 fixture；`test_public_surface` 的隔离子进程在 `-B -S` 下审计 package import 的写操作和 SDK/CAN/experiments 模块加载。既有 11 文件的链与边界见 F7 节；F8 CLI 只新加说明，没有真实执行分支。13 文件执行前均为 `SAFE_CANDIDATE`：解析 T1，其余隔离 T2。真实来源默认分支及旧单臂测试发现不在白名单。

执行基线：`main@34b4dc827fb67f1eef4bc8b038a8048c90af3444` 加待提交 F1～F8 工作树；工作目录 `/home/leo/nero_dh116_ws`；解释器 `/home/leo/miniconda3/envs/nero-py310/bin/python` **3.10.21**；精确命令 `PYTHON=/home/leo/miniconda3/envs/nero-py310/bin/python scripts/test_runtime_offline.sh`。脚本逐文件运行 `"$PYTHON" -B -S "tests/$file" -v`，显式 `PYTHONPATH="$repo_root/src:$repo_root/tests${PYTHONPATH:+:$PYTHONPATH}"`，`set -eu`，没有 discover。开始 **2026-09-30 13:30:31.802713 UTC**，结束 **13:30:33.587866 UTC**，墙钟约 **1.787 s**，脚本 exit **0**。每个文件 exit 0，stderr 仅 unittest 用例/计数/OK，stdout 仅 `[OFFLINE]` 文件名；0 fail、0 error、0 skip。Git status 前后相同；仅测试临时目录内生成合成 JSON/JSONL，无非预期仓库文件、CAN/SDK 设备创建、网络或硬件迹象。原始 stdout/stderr、每文件用例名、前后 status 与源码 SHA-256 收入仓库外第 19 号审阅 ZIP。

| 显式文件（脚本顺序） | 结果 | 证据限定 / 剩余风险 |
| --- | --- | --- |
| `tests/test_public_surface.py` | 6 pass | `-B -S` 包导入无写文件/设备模块，导出表、依赖方向、迁移表和 runner 白名单；源码静态检查不证明真实设备语义。 |
| `tests/test_runtime_cli.py` | 3 pass | 帮助、无真实执行命令、全部七种 show/list 离线分支在禁止来源 Fake 下成功；真实 status/capture/record 未调用。 |
| `tests/test_environment.py` | 5 pass | 合成 Profile、SHA/身份、离线展示；现实站点未批准。 |
| `tests/test_workflows.py` | 14 pass | 合成 Start/Park/Return/Teach 组合；真实路线与承托未知。 |
| `tests/test_replay.py` | 14 pass | 合成 F3→F6→F2；真实 Replay 未验证。 |
| `tests/test_teaching.py` | 16 pass | 合成 F4/F3/F1；真实 Drag exit 未验证。 |
| `tests/test_drag.py` | 16 pass | 合成模式请求/反馈；V121 物理响应未验证。 |
| `tests/test_recording.py` | 11 pass | 合成采集写临时 JSONL/摘要；真实采样未验证。 |
| `tests/test_motion.py` | 10 pass | 合成路点发布/反馈/停止；真实映射未验证。 |
| `tests/test_waypoints.py` | 6 pass | 临时点位 JSON；真实反馈未验证。 |
| `tests/test_readonly_entry.py` | 10 pass | 注入 SDK 形状 Fake 的只读 CLI；真实连接未执行。 |
| `tests/test_runtime_snapshot.py` | 15 pass | 3A 内存解析；不证明硬件同步。 |
| `tests/test_runtime_acquisition.py` | 12 pass | 3B 合成单轮采集；真实 SDK 缓存未核验。 |

合计 **138/138**。`OFFLINE_TESTED` 只覆盖上述 13 文件在该 HEAD 加工作树具体字节、Python 3.10.21、所列命令下的合成/离线路径。提交后精确 Git SHA、manifest 与输出关联见第 19 号报告及 ZIP。**Full test discovery = UNKNOWN_SIDE_EFFECT**；旧 `experiments/single_arm/**/tests` 未运行也未升级。
