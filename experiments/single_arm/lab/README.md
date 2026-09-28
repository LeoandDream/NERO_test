# 单臂统一实验接口

**当前操作从[根 README](../../../README.md)开始。**本页保留 API 和历史实验细节；其中 `quick home-demo/cube-sweep/cube-faces` 和无时限手动拖动仍不可作为当前实机入口。旧 `supported_home` 桌边路线仍锁定；`quick park` 现为统一 H0 命名目标入口的别名。

**S1 限时示教有条件解锁：**2026-09-26 从桌边低位进入拖动时机械臂突然大幅移动。`teach --run` 和 `quick teach/demo` 现在要求实时反馈为正常、七轴使能、停稳且位于已验证 S1；两次模式帧之间重新核对姿态，并监测启动后前 0.5 秒。S1 的基本静止 5 秒会话已实机平稳，零目标收尾已验证；真正拖动后的自动回位仍需按现场试验核对。`quick home-demo/cube-sweep/cube-faces` 和无时限手动拖动仍锁定。软件监测不能保证防止瞬时异动。详见[CAN 拖动事故说明](../can_setup/README.md#四通过-can-拖动示教)。

本目录提供 Python 入口 `LabAPI` 和相同规则的命令行入口 `python -m experiments.single_arm.lab.cli`。导入模块不会连接 CAN。人用 `quick` 命令在**同次调用**显示预检、键盘确认并执行；开发者仍可分别调用 plan/run，或用 `quick_replay()` / `quick_start()` 一次完成。命令不自动复位、使能或把未知姿态送到全零位。所有关节角为 **rad**，法兰位置为 **m**，时长为 **s**；只有 `pose-candidate` 特意以整数 **度**显示候选角。路径在工作区根目录下使用。完整入门流程见[根 README](../../../README.md)。

## 能力矩阵

| 能力 | Python API | 统一 CLI | 既有 CLI / 状态 |
| --- | --- | --- | --- |
| CAN、状态、七轴、法兰、驱动、停稳 | `status()` | `status` | `can_setup.nero_can_activate` 只读；状态异常拒绝运动 |
| 位姿记录 | `record_poses()` | `poses` | `pose_recording.record_poses` 保持可用；仅采样 |
| 停机回零姿态演示 | `home_demo()` | `quick home-demo` | **当前锁定**；历史实现曾进入拖动并记录路线，不自动回位或失能 |
| 桌边支撑位 H0 往返 | `plan_start("H0"/"S1")` / `run_start()` | `quick init --target H0/S1`、`quick park`；高级 `starts plan/run` | 精确匹配已标定两端点时生成经双向验收的 `move_j` 路线；到 H0 后仍保持使能 |
| 限时示教 | `teach_preflight()` / `teach_run()` | `teach` / `teach --run` | 退出拖动后从实时姿态新规划回 S1；原始轨迹只保存 |
| 新示教发现与校验 | `recordings()` / `validate_recording()` | `records` | 只收录新版本封存且完成回位的记录，旧记录不被改写 |
| 新示教关节回放 | `quick_replay()` 或 `plan_replay()` / `run_replay()` | `quick replay` / `quick demo`；高级 `replay plan/run` | `point_move_j` 逐点或 `timed_move_j` 定时连续；一份新 5 秒记录已完成实机往返 |
| 旧固定 20 秒连续回放 | `plan_continuous_archive()` / `run_continuous_archive()` | `continuous-archive plan/run` | 定时关节流；**不能**把任意新记录静默当连续源 |
| 命名目标 | `quick_start()` 或 `starts()` / `plan_start()` / `run_start()` | `quick init`；高级 `starts list/plan/run` | S1、C2、H0 使用同一规划入口；已标定 H0↔S1 选专用路线。2026-09-27 非 H0 低位有窄范围受监护候选，每次只执行一个低位目标并暂停；其它正常姿态→S1 用通用规划。S1→C2 保留已验证去程 |
| 实时重规划回 S1 | `plan_return()` / `run_return()` | `return plan/run` | 从当前七轴角生成少量 `move_j` 目标；不倒放示教记录；当前左侧装要求法兰 X 关节盒不低于 0.15 m |
| 整数度外观候选 | `plan_pose_candidate()` / `run_pose_candidate()` | `pose-candidate plan/run` | 仅 S1 邻域；[0,−33,0,−17,0,12,−27]° 已到位，公共默认尚未更改 |
| 法兰小段直线探测 | `plan_probe()` / `run_probe()` | `probe plan/run` | 每段 2～10 mm、单次 `move_l`；复用 `reachability.controlled_probe` |
| 局部关节探测 | `plan_joint_probe()` / `run_joint_probe()` | `joint-probe plan/run` | 仅已满足 J4>+0.02 rad 的姿态，J4 +0.05 rad 往返 |
| 可达数据读取 | `reachability_summary()` | `map` | 旧 `reachability.build_map` 保持可用；不同来源分级 |
| 边界拖动录制与极值候选 | `cube_sweep()` / `cube_extrema_candidate()` / `cube_extrema_from_recording()` | `quick cube-sweep`；离线 `cube extrema --map ...` 或 `--recording ...` | **拖动入口当前锁定**；已有正常轨迹可离线提取极值候选，不等于运动许可 |
| 六面分段拖动 | `cube_face_sweep()` | `quick cube-faces` | **当前锁定**；两次实机采样中途终止，未形成有效六面拟合 |
| 六面几何立方体 | `cube_new/cube_capture/cube_status/cube_fit` | `cube guide` 或 `cube new/capture/status/fit` | 人示教六面点位，程序反推边长和中心；不写 `flange_workspace_m`，无运动许可 |
| `move_p` 点位与其它故障诊断 | 暂无通用高层执行器 | 保留原专用 CLI | `control_interfaces.cartesian_point_session` 等需单独现场验证；发生桌面接触的旧演示计划已停用 |

上述“统一”是调用与校验边界；它不把所有控制方式改成一种。**复位到姿态、示教后回位及将来的停机支撑位均采用 `move_j`，验收七轴目标。**`move_p` 只作为 VLA 运行时的法兰点位控制方式；到达同一法兰位姿不代表回到同一七轴姿态。`point_move_j` 到位后才发下一目标，`timed_move_j` 按 10/20 Hz 定时发布，按源速度/加速度自动伸缩时间；速度百分比不能当作时间伸缩。**历史记录没有新封存元数据，不会显示在 `records latest`；历史回放仍用旧入口。**

2026-09-26 六面异常后的低位恢复属于一次性现场诊断：`python -m experiments.single_arm.lab.low_pose_escape --run` 在同一命令内读取实时状态、验证限定姿态并执行一次 J2 `move_j` 离桌；若姿态不同会退出。其后使用公共 `quick init --target S1` 从新姿态实时规划回位。该一次性模块不在公共 VLA API 矩阵内，也不能从其他未知低位套用。

## 最短 Python 调用

```python
from experiments.single_arm.lab.api import LabAPI

lab = LabAPI()                         # 尚未连接 CAN
print(lab.starts())                    # 离线列出 S1/C2/H0
state = lab.status()                   # 只读连接，返回 ready_for_motion/reasons
print(state["joint_rad"], state["flange_pose_m_rad"])
print(lab.recordings())                # 仅完整且哈希匹配的新示教

# 在示教完成且处于该记录起点后：
result = lab.plan_replay("latest", mode="point", speed_percent=5)
print(result["plan_path"], result["plan"]["flange_xyz_envelope_m"])
# 现场核对后才显式调用：lab.run_replay(result["plan_path"])
# 一次调用并自动重复实时校验：lab.quick_replay("latest", mode="point", speed_percent=5)
# 新封存记录也可离线计划连续发布；先检查计划时长与完整活动范围：
# smooth = lab.plan_replay("latest", mode="continuous", frequency_hz=20,
#                          min_time_scale=6)
# lab.run_replay(smooth["plan_path"])  # 执行前仍须核对实时状态与整条路线
# S1 与 H0 共用命名目标入口；现场确认路线后使用：
# lab.quick_start("S1")  # 已标定 H0→S1；失能 H0 会先单次使能
# lab.quick_start("H0")  # S1→轻触桌边 H0，到位后仍保持使能
# 停在其他正常姿态时，按实时七轴角生成新的少目标回 S1 路线：
planned = lab.plan_return(min_flange_x_m=0.15)
# 现场核对 planned["plan"]["joint_box_flange_ranges_m"] 后显式执行：
# lab.run_return(planned["plan_path"])
```

回放和命名起点执行会使用已保存的计划文件；`quick_replay()` / `quick_start()` 在本次调用内创建该文件并随后重新检查配置、源记录、实时姿态、状态与完整路线。接口返回报告路径或抛出明确异常。CLI 执行失败以非零退出码和“实验未完成”说明。报告包含时间、目标、反馈、最后到位小步与失败原因。连接及机器人对象可经 `robot_factory` 注入，用于离线替身测试。`status()` 的 `ready_for_motion` 是**设备状态检查**，仍须现场核对桌面、支撑、工具、安装方向、线缆和整条连杆活动空间。

`quick teach`、`quick replay`、`quick demo`、`quick init` 会在交互终端要求键入 `开始`。非交互执行必须给完整参数并加 `--yes`；它仍不能跳过源文件、配置或实时状态校验。`quick demo` 先做限时示教并回位，再在同一命令中展示刚生成的记录和回放预检，第二次确认后才回放。`quick replay` 直接列出可选记录并支持回车选 `latest`。未指定 `--mode` 时，交互终端显示连续与逐点两种选项，直接回车选连续；其默认 20 Hz、最低时间倍率 6。脚本调用须显式给 `--mode`；连续方式可指定 `--frequency-hz 10/20` 及 `--min-time-scale 6～30`，不传 `--speed-percent`。实际倍率可能因关节速度/加速度限制更大，须查看计划总时长。新 5 秒源的一次 20 Hz 连续往返已实机完成，结果见[报告](report.md)；这不自动验证其它源或现场环境。

以下三段记载拖动入口的历史设计，**当前 `quick cube-sweep`、`quick home-demo`、`quick cube-faces` 均在执行前被锁定**。已录得的正常边界轨迹可用根 README 的 `cube extrema` 离线分析；不要按下列历史步骤再次启动拖动。

`quick cube-sweep` 的历史设计要求键入 `开始`，在同一次调用中录制边界并提取立方体极值候选。无论采样是否成功都会尝试退出拖动；失败报告与原始 CSV 保留。录制结束后**不自动回位**，回 S1 使用实时规划 `move_j`。此前 30 秒录制及随后从实时姿态规划的 5 目标回 S1 均已完成，见[报告](report.md)。

`quick home-demo` 专门用于由人在拖动模式下演示“实验姿态→实体支撑的回零位”路线：`python -m experiments.single_arm.lab.cli quick home-demo --duration-s 30`。在提示后键入 `开始`，沿拟定路线手拖，并在终点停稳至少 2 秒。命令内检查实时状态、开启 CAN 拖动、以 20 Hz 记录关节和法兰、退出拖动，并把起终点、最大移动量、最后 1 秒变化量保存到 `lab/data/home_demos/`。退出拖动后若为模式 2、示教状态 0/2/6、整机正常且七轴使能，记录可验收；**模式 2 不能直接用于 `move_j`**。如采样中断也会尝试退出拖动并保留失败报告和部分 CSV。**采样终点只作为待核对的回零候选**；程序不倒放这条路线、不自动发送 `move_j`、不失能。`poses` 及 `pose_recording.record_poses` 只读，不会进入拖动模式。

原 H0 候选来自[拖动演示](data/home_demos/nero_home_demo_20260926T093257864574Z.json)的终点，法兰最终由桌面朝人一侧边缘支撑。[H0→S1](data/supported_home_runs/nero_supported_home_run_20260926T094711424592Z.json)首次实机回程 5/5 到位且现场平稳；[S1→H0](data/supported_home_runs/nero_supported_home_run_20260926T094856415806Z.json)也报告 5/5 到位、控制器正常，但用户随后报告**最后下降接近桌边时发生擦碰**。最终接触是预期支撑，中途擦碰则使路线验收失败。到位反馈只能证明关节目标，不能证明路线与桌边接触安全。旧 [H0 配置](config/supported_home.json)继续标记 `route_blocked`，其规划器仍拒绝执行；`quick park` 现已改用新路线。旧计划不再适用。

2026-09-26 另试[高位先沿 Y 负向绕行](high_y_home.py)：从 S1 到外侧点的 7 个 `move_j` 目标到位，现场确认该段平稳；接着外侧点到旧 H0 的 2 个低速目标也由控制器报告到位，但现场反馈**有擦碰或承托位置不对**。因此该入口的所有实机执行阶段已再次锁定，H0→S1 尚未运行，H0 仍未获停机失能许可。只读状态或关节到位不能替代桌边间隙和实体承托验收；先查明接触发生在中间点还是最终贴边阶段，再改路线。

一次据旧手拖样本设计的基座 X 负向约 42 mm 外移虽[到位](data/edge_releases/nero_edge_release_20260926T100204135987Z.json)，用户观察却怀疑该方向朝向支架。`edge_release` 已锁定，不能重复使用；后续方向应以新的现场拖动示范为准。

`quick cube-faces` 用于真正的六面采样：按 `+X,-X,+Y,-Y,+Z,-Z` 顺序逐面提示；每面在设想的平面上拖到至少三个分散位置，各停稳约 0.3 秒。程序从各段停留反馈自动选三点，保留六段原始 CSV、源行和配置哈希并运行等边拟合；缺面或不等边如实失败，不授予运动许可。交互时每面先扶稳并按回车，再由底层启动命令核对实时控制器状态和七轴驱动，随后开启该面拖动且立即录制；录完就退出拖动，等待下个面的确认期间不保持拖动。输入 `q` 会保存已采数据并取消；非交互须显式 `--seconds-per-face 12 --rate-hz 20 --yes`，每面开启拖动前有 5 秒准备时间。失败后可用 `quick cube-faces --session <原会话目录>` 只补少于三个点的面；续录会核对配置与原始 CSV 哈希，六面已有点时可纯离线重拟合。两次实机尝试分别停在 `−X` 与 `−Y` 面开始处，均未产生有效六面拟合；用户现选完整轨迹六极值候选，暂停逐面采集。

## 数据契约

### 缩短后的 S1 ↔ 轻触桌边候选位路线

`python -m experiments.single_arm.lab.optimized_home --direction start|home` 在当前实时起点上搜索少量 `move_j` 中间目标；加 `--run` 才执行。默认使用已双向实机验收的 `reduced-j3`：7 个目标，J3 最大弯曲约 −0.7 rad，累计关节行程约 4.072 rad。保留高位基座 Y 负向绕行，以及靠近桌边时仅改变 J1 的低速贴边/离桌动作；通过关节限位、关节盒 FK、低 X 区域 Y 余量和法兰 Z 下限筛选。原先 J3≈−0.9 rad 的路线可显式用 `--strategy validated` 查询。两条路线只接受已标定的 S1 与轻触桌边候选位，不能从任意姿态直接套用，也不证明控制器实际中间路径是全局最短或与实体桌面、线缆无碰撞。到位后保持使能，不自动失能。

若 `home --run` 报“第 1 步未执行；机械臂仍停在上一小步附近”，先现场确认状态，再用 `--resume-report <失败报告.json>` 运行同一入口。不加 `--run` 只读复核剩余目标；通过后加 `--run` 才续行。它会重算当前标定路线，并要求实时关节角与报告的最后到位目标一致、控制器正常且未急停；姿态已变化时直接拒绝，须重新规划。正常 5% 中间目标若确认完全未动且控制器状态正常，程序只允许重发相同目标一次；2% 桌边单轴动作不自动重发。

2026-09-26 减小 J3 偏转的路线完成 S1→H0→S1 双向试验，分别 7/7 目标到位；现场确认回家终点轻触承托可靠、回程顺利离桌，全程无异常。旧 `supported_home` 过压路线仍锁定；`quick park` 已改由统一命名目标入口调用本路线。详见[实验报告](report.md)。

- 新示教 CSV 与同名 JSON 共存。JSON `schema_version=2`，含 `recording_id`、`recording_sha256`、`config_sha256`、型号/固件、起点、示教结束/回位状态。`records` 列出经全部校验的完整记录；`latest` 检查最近一次示教尝试，失败时直接说明原因并拒绝回放，不会退选旧记录。仅在完整限时采样和停稳样本已保存、自动回位预检失败且未急停时，可独立用 `return plan/run` 回 S1，再运行 `python -m experiments.single_arm.lab.complete_recording --recording <CSV> --return-report <成功报告>` 生成哈希绑定的只读完成凭证；原始失败记录不改写。更改任一原始字节或回位报告均会使凭证失效。
- `replay plan` 明确选择逐点或定时连续模式，保存源/配置哈希、起点和完整目标。连续模式还保存采样频率、时间伸缩、总时长和法兰包络审计；执行前重算所有目标，拒绝篡改。`replay run --plan` 每次只执行一轮正向加原路返回，新计划不自动执行。连续流的实时跟踪、CAN 帧、时钟误差和异常停止使用旧执行器；一份新封存源已完成实机验收。
- S1/C2 基础目录在 [config/starts.json](config/starts.json)，H0 是另行标定的桌边支撑目标；公共默认仍是 S1。`quick init`、`quick park`、Python `quick_start()` 与高级 `starts plan/run` 统一按命名目标分流：已使能 H0→S1 调用 `optimized_home`，已失能 H0→S1 调用 `start_from_h0_candidate`，S1→H0 调用 `optimized_home`；其他正常姿态→S1 调用法兰 X≥0.15 m 的通用实时 `move_j` 规划器。保存的命名计划在运行前重算并核对实时姿态；交互 `quick` 还要求键入“开始”。急停、未停稳、未知低位或范围不符时拒绝运动。S1→C2 保留已验收的关节去程；历史路线不证明当前现场清空。
- `return plan/run` 从当前姿态独立规划 S1，不要求起点在 S1/C2 附近；它只接受正常、七轴使能、停稳的左侧安装裸法兰场景，逐段抽查关节限位及关节盒 FK。默认法兰 X≥0.15 m，**不能从桌边 H0 的负 X 姿态直接规划**。已使能 H0 使用 `optimized_home --direction start`；失能 H0 使用 `start_from_h0_candidate`。2026-09-27 非 H0 低位候选只通过交互 `quick init --target S1` 分流，默认 `return` 和高级 `starts run` 不自动获得该现场许可。默认每次只发一个 2% 目标并暂停；首目标已现场平稳验收后，用户明确选择完整扫过范围受监护时，可用 `quick init --target S1 --complete-low-route` 一次命令逐目标执行到 S1。该选项不接受 `--yes`，完整路线尚未实机验收。S1 回轻触候选位使用 `optimized_home --direction home`，现场确认承托后可另用 `disable_at_h0_candidate` 单次失能；这两步不构成无人值守自动停机。
- 立方体会话有 manifest、每点单独 JSON 和每次拟合 JSON。六面各至少 3 个不共线法兰点，三边偏差≤5%、面偏差≤5 mm、法向误差≤8°才标 `geometry_valid=true`。`motion_region_verified` 永远为 `false`；必须另有实体环境及路径证据才能设运动区域。
- `cube extrema --recording` 把正常拖动原始 CSV 转为带来源索引的地图，然后提取 XYZ 极值；`--map` 则从已有地图的一种证据级别提取。两者按内缩余量提出轴对齐候选，并给出六个确实出现在原始轨迹中的内缩极值姿态、来源行及关节角。左侧安装现场确认基座 X 正向离桌；`--bottom-margin-m 0.03` 可让桌面侧 `−X` 比普通 `--margin-m 0.01` 多留余量。候选面中心到实测轨迹的距离另行报告。产物标 `EXTREMA_CANDIDATE_ONLY`；轨迹极值不能证明六面、内部或角点可达，也不允许直接发布候选面中心运动。
- 候选还提供基座轴对齐立方体的中心、等长边、实际候选边界和八个角点。边长取内缩后最窄轴跨度，其余两轴居中；角点为几何构造值，非实测点。2026-09-26 用户选择以一条完整边界轨迹直接构建此候选，暂停六面分段采样；[本次产物](../workspace_cube/data/candidates/nero_cube_extrema_20260926T082126485145Z.json)保留原始 CSV 哈希与六个极值来源行。

## 失败时

运动途中停滞、越界或状态异常时，旧执行器可能请求阻尼电子急停。停止后机械臂可能下垂；先现场支撑与只读 `status`，看报告中最后发布的目标和反馈。不要直接重放旧计划或推定已回 S1。见[急停恢复](../../../docs/emergency_stop_recovery.md)。

当前这次从非命名低位回 S1 的现场辅助位于 `lab.site_recovery`，与通用 `starts` API **分开**。它依据当前实时姿态、左侧装与“基座 X 正向离桌”的现场条件，先只读生成四段单目标计划；每段需再次复核实际起点、桌面/线缆范围，并单独显式运行。计划只适用于生成时的姿态；任何接触、反馈偏离或停机后都停止，不跨段，不自动继续。当前[计划和验收状态](report.md)不能当作未来位置的模板。

整套 `unittest discover` 的设备副作用仍待核，不应作为无条件离线验证入口。选定测试前看[逐文件静态审计](../../../docs/test_tiers_audit.md)，实际运行结果须另行记录。

本接口新增功能的实机状态和待办见[报告](report.md)。
