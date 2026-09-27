# 立方体几何标定实验报告

版本：`cube_faces_v1`，2026-09-26（UTC 时间以会话 manifest 和拟合 JSON 为准）。配置是 Nero 左侧安装、裸法兰、零工具偏移。`lab.cube` 使用六面平面拟合与等边检查；不会把几何结果写入运动许可边界。

离线单元测试：旋转的 0.20 m 立方体通过并给出中心、三边和八角；缺面与 0.20×0.20×0.30 m 长方体被拒绝。具体运行命令、通过项和当前实机状态见[接口报告](../lab/report.md)。

已在 2026-09-26T05:09:58Z 用 `python -m experiments.single_arm.lab.cli cube new` 建立[空会话](data/sessions/nero_cube_20260926T050958778754Z/manifest.json)。该会话尚无六面点；创建 manifest 不等于标定通过。

`cube status` 报六面均 0 点；同一会话的[缺面拟合记录](data/sessions/nero_cube_20260926T050958778754Z/fit_20260926T051029319044Z.json)正确给出 `geometry_valid=false`、`motion_region_verified=false` 和六面缺失原因。它是失败样例，不是一次真实局部立方体采样。

用户要求程序从示教/运动轨迹自行推算虚拟立方体，而不预设尺寸。05:54Z 创建[新会话](data/sessions/nero_cube_20260926T055434969074Z/manifest.json)，在现场确认后进入左侧装 CAN 拖动模式。用户指出手工固定六面点不便，提出从历史轨迹直接提取极值；本次会话没有把未确认的拖动位置冒充面点。06:04Z 在用户确认已停稳并扶稳后显式退出拖动；独立反馈 `ctrl_mode=2, arm_status=0, teach_status=0`、错误码 0、七轴使能且停稳。停下后实际法兰约 `[0.372329,0.013069,0.563461] m`，七轴已明显偏离 S1；法兰位置靠近 S1 不表示关节姿态自动回了 S1。当前未切回 CAN 运动控制模式。

06:06Z 离线读取[旧实测地图](../reachability/data/maps/nero_workspace_20260925T164948469376Z.csv)中 `new_full_continuous_path_sample` 的 9445 条完整连续回放反馈，运行 `python -m experiments.single_arm.lab.cli cube extrema --map experiments/single_arm/reachability/data/maps/nero_workspace_20260925T164948469376Z.csv --level new_full_continuous_path_sample`，保存[极值候选 JSON](data/candidates/nero_cube_extrema_20260926T060633142691Z.json)。X/Y/Z 单轴跨度为 197.289/140.161/175.933 mm；每轴中点约 `[0.341837,-0.063868,0.522557] m`，按最窄轴两端各内缩 10 mm 的候选边长为 120.161 mm。六个极值的时间、源行、当时法兰和七轴角都在 JSON。

候选六个面中心到旧轨迹最近距离分别为 `+X 4.0、−X 40.1、+Y 27.9、−Y 24.7、+Z 7.8、−Z 46.7 mm`。六个极值不是同一时刻，也不构成六个已测平面；尤其多个候选面中心远离已记录位置。因此结果标为 `EXTREMA_CANDIDATE_ONLY`、`geometry_valid=false`、`motion_region_verified=false`，不会写入运动配置或发布运动。

06:28Z 首次新 30 秒拖动[原始记录](../pose_recording/data/recordings/nero_poses_20260926T062809Z.csv)虽然有 599 条正常状态样本，X/Z 均仅约 0.02 mm 跨度、Y 几乎不变；没有形成边界，未将其用于空间估计。现场再次明确“开始”后，于 06:29Z 重录[599 条正常拖动反馈](../pose_recording/data/recordings/nero_poses_20260926T062951Z.csv)，转换为[带来源索引的极值地图](data/candidates/nero_drag_map_20260926T063042397610Z.csv)和[新候选 JSON](data/candidates/nero_cube_extrema_20260926T063042404787Z.json)。法兰实测 X `−0.154610～+0.512052 m`、Y `−0.219569～+0.238384 m`、Z `+0.409249～+0.701574 m`，单轴跨度约 666.7/458.0/292.3 mm。以最窄的 Z 跨度两侧各内缩 10 mm，候选边长 272.325 mm、轴对齐中心 `[0.178721,0.009408,0.555412] m`。

新候选六面中心距这条实际拖动轨迹的最近距离为 `+X 44.2、−X 119.8、+Y 86.5、−Y 39.7、+Z 22.5、−Z 215.7 mm`。因此尤其不能把下表面附近当成已经触达。599 条原始记录均为 `TEACHING_MODE/NORMAL/START_RECORDING`；法兰反馈与用同一行关节角计算的 FK 极值跨度相差不到 0.4 mm，但少数高速拖动样本的关节/法兰反馈时间差最高约 19 ms，逐行位置差最高约 14.7 mm。极值 XYZ 应按原始法兰反馈解读，相应七轴角只是当时近邻反馈，不可直接用于发布极值姿态。录制结束后拖动已退出，控制模式仍为 2，机械臂停在记录末端、并非 S1。

**六面拟合状态：UNVERIFIED。**截至本报告，没有完整的本次六面法兰样本或拟合产物。历史关节轨迹与可达地图只支持候选几何，不证明六面或内部运动区域。若以后采齐 18 个以上的面点，再把六面数量、面残差、边长误差与拟合 JSON 附在此处。

新增 `lab.cli quick cube-sweep` 单次交互入口：键入 `开始` 后，依次启动 CAN 拖动、录制新边界、退出拖动、离线生成极值候选；任一步失败会尝试退出拖动并保存报告，不自动回位。开始/停止次序、录制失败仍结束拖动、交互取消和非交互参数门槛已由离线替身测试覆盖。

07:26Z 现场人员确认扶稳机械臂并准备拖动后，实机运行 `python -m experiments.single_arm.lab.cli quick cube-sweep --duration-s 30 --rate-hz 20 --margin-m 0.01 --yes`。[运行报告](data/sweeps/nero_cube_sweep_20260926T072610405628Z.json)显示开始/结束拖动指令均返回 0、录得[原始 CSV](../pose_recording/data/recordings/nero_poses_lab_20260926T072611012480Z.csv) 599 条，哈希与时间见报告。程序生成[来源地图](data/candidates/nero_drag_map_20260926T072641370333Z.csv)及[极值候选](data/candidates/nero_cube_extrema_20260926T072641377963Z.json)。X/Y/Z 范围分别为 `0.172944～0.534201`、`−0.149502～0.281000`、`0.311131～0.666264 m`，跨度约 361/431/355 mm；内缩后等边候选边长 335.133 mm，中心 `[0.3535725, 0.0657490, 0.4886975] m`。

六个候选面中心到实测轨迹最近距离依次为 `+X 75.5、−X 136.4、+Y 28.5、−Y 29.3、+Z 68.2、−Z 154.8 mm`。因此这次实机录制验证了**采集与极值提取入口**，并未提供足以拟合六面的真实面点，状态仍是 `EXTREMA_CANDIDATE_ONLY`、`geometry_valid=false`、`motion_region_verified=false`。录制结束后控制模式留在 2，机械臂停在实际终点；随后从实时七轴角另行规划 `move_j` 回 S1，[回位证据](../lab/data/planned_return_runs/nero_planned_return_run_20260926T072847168319Z.json)见接口报告。

新增 `quick cube-faces` 分段拖动入口：六个局部面各录一段真实反馈，自动取每段三个分散的停留点并拟合；若缺少停留点则保留缺面状态。离线等边六面替身、无停留点拒绝和取消后退出拖动的测试通过；**尚未进行实机六面采样**，不能以当前极值轨迹或替身测试宣称标定完成。

07:45Z 该入口首次实机尝试产生[失败报告](data/face_sweeps/nero_cube_faces_20260926T074556653540Z.json)：`+X` 段[原始 CSV](../pose_recording/data/recordings/nero_poses_lab_20260926T074604856119Z.csv)有 239 条、提取 3 个停留点；操作者准备 `-X` 面约 31 秒后，第二段[原始 CSV](../pose_recording/data/recordings/nero_poses_lab_20260926T074647416080Z.csv)仅有表头，连续一秒无有效拖动反馈，流程发出结束拖动命令并停止。操作者报告当时触发了电子急停。07:47Z 后只读状态为控制模式 2、`JOINT_BRAKE_NOT_RELEASED(0x6)`、七轴失能、错误码 0、短时停稳；这不是可继续采样或运动的状态。脚本只发送拖动开始/结束帧，没有主动发送电子急停帧；从现有日志仍无法判定控制器退出拖动的起因。六面拟合未运行，状态保持 `UNVERIFIED`；不复用此次部分点作完整标定。

一次现场确认后单次使能成功；07:53Z 独立只读反馈为 `NORMAL`、七轴使能且停稳，法兰位于 `[-0.077,-0.023,0.715] m` 附近，**不在 S1**。使能前后关节反馈有较大变化，现场未注意到异常运动；原因未明。六面入口已改为每面单独进入、退出拖动，等待下一面时保持非拖动，并加入失败后停止拖动的离线测试；此修订尚未经实机复测。

按现场“基座 X 正向离桌”约定，从 599 条正常拖动记录生成[额外底部余量候选](data/candidates/nero_cube_extrema_20260926T075952478126Z.json)：普通六向内缩 10 mm，桌面侧 `−X` 内缩 30 mm。所得建议范围为 X `0.203～0.524`、Y `−0.140～0.271`、Z `0.321～0.656 m`，并附六个**确实在记录中出现过**的内缩极值姿态及来源行。候选轴对齐立方体边长约 321 mm，但面中心距轨迹仍有约 21～146 mm，故仅是快速探索参考，`geometry_valid=false`、`motion_region_verified=false`；不得把立方体面、内部或这些点之间的控制器路径当作已验证可达区域。

中断后的[六面部分会话](data/sessions/nero_cube_20260926T074556653307Z/manifest.json)可通过 `cube status --session <目录>` 只读查看。08:14Z 对真实部分样本运行离线 `cube fit` 得到[缺面拟合记录](data/sessions/nero_cube_20260926T074556653307Z/fit_20260926T081436351673Z.json)：`+X=3`，其余五面为零，`geometry_valid=false`、`motion_region_verified=false`，原因是缺五面。新增 `quick cube-faces --session <目录>`，在启动实际拖动前核对配置与原始录制哈希，并只补点数不足三个的面；每次 `can_drag_teach --start` 在同一命令内核对实时状态和七轴驱动。离线替身验证中断续录、缺失原始录制拒绝和六面齐备后的无设备重拟合；旧会话续录尚未实机尝试。已有 `+X` 三个停留点的 X 值跨度约 204 mm，是否同属一个目标面未核实，故续录不会自动把该旧会话升级为有效标定。

08:17Z 现场从新会话重做分面录制：[第二次失败报告](data/face_sweeps/nero_cube_faces_20260926T081700410097Z.json)保留 `+X=3`、`−X=3`、`+Y=0` 个停留点；准备 `−Y` 时底层启动命令因七轴驱动反馈暂时不完整而拒绝进入拖动，程序已发结束拖动指令。08:18Z 独立只读状态为控制模式 2、`NORMAL`、七轴使能、错误码 0、0.25 秒最大关节变化约 0.000017 rad；当前法兰约 `[0.425719,-0.048180,0.528317] m`，**不在 S1**。现场用户决定停止六面逐面采样，改由一条完整轨迹的六个单轴极值直接构造候选；这些未完成的分面数据不参与候选计算。

08:21Z 读取[07:26Z 完整 30 秒拖动 CSV](../pose_recording/data/recordings/nero_poses_lab_20260926T072611012480Z.csv)的 599 行正常反馈，运行 `python -m experiments.single_arm.lab.cli cube extrema --recording <CSV> --margin-m 0.01 --bottom-margin-m 0.03` 生成新的[来源地图](data/candidates/nero_drag_map_20260926T082044601204Z.csv)；再对该地图运行相同余量的 `cube extrema --map` 保存[最终快速候选](data/candidates/nero_cube_extrema_20260926T082126485145Z.json)。六个原始单轴极值来自六个不同的源行，源 CSV SHA-256 为 `def8192e161af9a1a51b9554775a3c0f9c6d57588c340870521b287380865f95`。基座轴对齐等边候选中心 `[0.3635725,0.0657490,0.4886975] m`、边长 `0.321257 m`；候选边界 X `[0.202944,0.524201]`、Y `[−0.0948795,0.2263775]`、Z `[0.328069,0.649326] m`，八个角点及六面中心均在 JSON。最近实测点到六面中心的距离约 21～146 mm，所有构造角点均非实测点。结果严格为 `EXTREMA_CANDIDATE_ONLY`、`geometry_valid=false`、`motion_region_verified=false`，不能把此体积当作自动运动许可。
