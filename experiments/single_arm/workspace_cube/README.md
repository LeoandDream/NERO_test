# 法兰中心六面立方体几何标定

本实验以**机器人基坐标系中的法兰中心**为点，工具偏移固定为 `[0,0,0] m`。用户不必先提供边长或中心。可以先从已有的完整实测轨迹提取六个坐标极值，自动提出轴对齐立方体候选；若要完成六面几何标定，还要拖动示教每面的多个实际点。单轴极值不等于一整面已被采到，候选不构成运动许可。

当前按现场选择使用[一条完整拖动轨迹生成的候选](data/candidates/nero_cube_extrema_20260926T082126485145Z.json)：普通方向内缩 10 mm，左侧装桌面侧基座 `−X` 内缩 30 mm，按最窄内缩跨度构造等边、基座轴对齐的立方体。JSON 含 599 行源记录哈希、六个极值的源行和关节角、候选中心/边长、六面中心与八角。六面分段采样暂不继续；`geometry_valid=false` 与 `motion_region_verified=false` 表示此几何是轨迹包络推断，不表示六个面或体积经过实测验证。

## 先从轨迹提取候选

需要录制新边界时，在已使能且停稳的姿态、现场扶稳并清空范围后，只运行一条交互命令：

```bash
python -m experiments.single_arm.lab.cli quick cube-sweep
```

它先只读检查状态，键入 `开始` 后才按当前安装方向进入拖动，默认记录 30 秒、20 Hz；结束时退出拖动模式，并把原始 CSV、极值地图、候选 JSON 和运行报告分别保存。录制失败也会尝试退出拖动并保留失败报告。它**不会回 S1**；需要回位时从停稳的实时七轴状态另外规划 `move_j`。脚本环境需完整指定 `--duration-s`、`--rate-hz`、`--margin-m`、`--yes`。这条命令只生成几何候选，不会把轨迹包围盒变为运动安全边界。

在工作区根目录运行离线命令，不连接机械臂：

```bash
python -m experiments.single_arm.lab.cli cube extrema --map experiments/single_arm/reachability/data/maps/nero_workspace_20260925T164948469376Z.csv --level new_full_continuous_path_sample
# 对新录制的正常 CAN 拖动原始 CSV，可直接使用：
python -m experiments.single_arm.lab.cli cube extrema --recording experiments/single_arm/pose_recording/data/recordings/nero_poses_20260926T062951Z.csv
# 用最近的实测边界，桌面侧 −X 多内缩 30 mm；无需重新进入拖动
python -m experiments.single_arm.lab.cli cube extrema --map experiments/single_arm/workspace_cube/data/candidates/nero_drag_map_20260926T072641370333Z.csv --level new_hand_guided_sweep --margin-m 0.01 --bottom-margin-m 0.03
```

程序按 `--map` 的指定证据级别或 `--recording` 的 `new_hand_guided_sweep` 级别处理，保存每轴最大/最小值的原始行、当时七轴角、来源、内缩后确实录到的六个推荐姿态、候选立方体中心、边长和六个面中心。拖动记录必须有连续样本，且每行都是正常拖动反馈；近乎静止的记录因跨度不足会拒绝生成候选。默认普通方向各内缩 10 mm；现场确认基座 X 正向离桌时，可用 `--bottom-margin-m` 对桌面侧 `−X` 加大内缩。`+X` 面位于基座 Y–Z 平面，`±Y` 面位于 X–Z 平面。面中心离轨迹仍可能很远，脚本输出最近实测距离供补采决策。产物始终是 `EXTREMA_CANDIDATE_ONLY`，`geometry_valid=false`、`motion_region_verified=false`，不写入运动安全配置。见[本次候选报告](report.md)。

## 六面实测拟合

若要通过六面几何拟合，面标签是目标虚拟立方体的局部 `±X/±Y/±Z`，并非要求立方体轴一定与基坐标轴重合。每面至少采三个分散、不共线且停稳的点；推荐四个角区点。若某个面受桌面阻挡，保持“缺面”结果，不能用别的面冒充。

若不想逐点按回车，可以运行 `python -m experiments.single_arm.lab.cli quick cube-faces`：一条命令引导六面，每面各录一段。准备该面时机械臂不处于拖动模式；扶稳后按回车，底层启动命令核对实时状态和七轴驱动，通过才开启拖动并立即录制，录完即结束该面拖动。在该平面上三个分散位置各停留约 0.3 秒；程序自动从停留段取三点，存原始 CSV、源行与拟合报告。失败后 `quick cube-faces --session <原会话目录>` 可只补缺面；配置或原始 CSV 哈希不符会拒绝续录。此入口不从极值伪造面点，也不自动回位。首次实机尝试在第二面前异常失能，仅保留了 `+X` 面的部分数据；其共面含义尚待核对，六面标定仍待完成。完整步骤见[新手指南](../../../docs/quickstart_cli.md#c-六面立方体几何标定)。

现场扶稳机械臂、确认左侧安装重力补偿和完整扫过范围后，交互采样可逐条输入：

```bash
python -m experiments.single_arm.can_setup.can_drag_teach --start --mount left
python -m experiments.single_arm.lab.cli cube guide
python -m experiments.single_arm.can_setup.can_drag_teach --stop
```

`cube guide` 在同一个终端依次提示六个面。把法兰放到该面不同位置、停稳后按回车采样；输入 `q` 保留部分会话，之后可用 `cube guide --session <会话路径>` 继续。它只读采样，不控制拖动或自动回位。**退出或中断引导后，都需要显式发送 `--stop`**。六面各三点完成时自动拟合并显示边长、中心、缺面或无效原因。

在工作区根目录运行：

```bash
python -m experiments.single_arm.lab.cli cube new
python -m experiments.single_arm.lab.cli cube capture --session <session_path> --face +X
python -m experiments.single_arm.lab.cli cube status --session <session_path>
python -m experiments.single_arm.lab.cli cube fit --session <session_path>
```

把法兰移到一个计划点并停稳后，运行对应 `cube capture`；同一面重复至少 3 次，六面共至少 18 次。采样命令只读，不发送拖动、复位、使能或运动命令。移动方法和活动空间由现场人员另行确认；若在 CAN 拖动模式采样，须可靠扶稳，留意左侧安装的重力补偿与线缆。每点记录七轴 rad、六维法兰 m/rad、反馈时间、控制器示教状态、配置哈希。一个点一个不可覆盖的 JSON；`status` 显示缺面，`fit` 保存带时间戳的拟合 JSON。

拟合先独立拟合六个平面，再比较对面距离、正交、等边和面内残差。六面少于 3 点、单面近共线、三边不等长、面残差超过 5 mm 都标 `geometry_valid=false` 并给出原因。若成功则给中心、边长、局部轴在基坐标系中的方向和八个顶点；结果仍是 `GEOMETRY_ONLY_UNVERIFIED_MOTION`。面点只提供表面几何，不足以证明内部全部点位可达，更不能证明连杆、支撑、桌面或线缆无碰撞。本程序不会自动改写 `config/nero_teach.json` 的 `flange_workspace_m`。

本次实现的离线验证与实机采样状态见[报告](report.md)。
