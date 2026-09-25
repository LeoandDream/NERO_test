# Nero 机器人实验工作区

这是使用 Python 3.10、SocketCAN 和 `pyAgxArm` 的机器人实验工作区。当前已完成的实验均属于[单臂实验](experiments/single_arm/README.md)。后续双臂、夹爪、灵巧手实验可在 `experiments/` 下各建独立目录，不与单臂历史数据混放。厂家资料保存在 [Nero 用户指南](docs/user_guide.md)。

20 秒示教记录先完成旧式逐点回放，随后又完成[定时连续回放](experiments/single_arm/continuous_replay/report.md)：完整 1022 行往返、9445 条目标及 CAN 帧通过独立验收，并回到 S1。早前回放失败曾触发电子急停，恢复过程见[电子急停与实验恢复](docs/emergency_stop_recovery.md)。此前多方向 `move_p` 实验发生法兰桌面接触，失效的点位演示脚本已停用；新的 16 段受控探测与空间证据等级见[可达空间实验](experiments/single_arm/reachability/report.md)。非零初始化候选 C2 已完成精确到位、3 秒保持和原路返回的实机遥测验收；外观判断仍待用户确认，公共 S1 配置未改。

## 目录

```text
experiments/
└── single_arm/
    ├── config/            # 单臂共同使用的 S1/P4 配置
    ├── can_setup/         # CAN 连通、状态与使能
    ├── pose_recording/    # 位置采集与法兰范围分析
    ├── teaching/          # 限时拖动示教与原路回位
    ├── start_transfer/    # 从当前姿态规划到 S1
    ├── replay/            # 示教轨迹回放
    ├── cartesian_x/       # 数值逆解的法兰 X 方向运动
    ├── local_probe/       # 急停恢复后的局部关节对照
    ├── control_interfaces/ # move_j、move_p、move_l 发布接口与单次法兰实验
    ├── continuous_replay/  # 完整示教源的定时连续关节目标回放
    └── reachability/      # 可达空间证据、逐段直线探测、初始化候选
docs/user_guide.md         # 厂家指南
environment.yml            # Conda 环境定义
user/                      # 用户需求与开发思路
```

每个实验目录单独保存代码、`README.md`、`report.md` 和该实验的数据。历史 JSON 中记录的旧相对路径保留原样，表示当时运行时的位置；查找文件请按现在的目录结构。

## 环境与测试顺序

在工作区根目录运行：

```bash
conda env create -f environment.yml  # 已有 nero-py310 环境时跳过
conda activate nero-py310
python -m unittest discover -s experiments/single_arm -p 'test_*.py' -q
```

`can-utils` 是 Ubuntu 系统工具，用 `sudo apt install can-utils` 安装。离线单元测试不要求连接机械臂，也不会发送运动指令。

按[单臂实验总览](experiments/single_arm/README.md)依次进行：CAN 连通和状态 → 位姿采集 → 限时示教 → 去起点 → 轨迹回放 → 自主轨迹。每次实机运动都应从当前反馈重新核对起点和路径；各实验 README 给出只读检查和显式执行命令。

第一次操作请从[零基础命令行实验指南](docs/quickstart_cli.md)开始。现场确认后，命令行中须显式加 `--enable` 才使能，运动实验须显式加 `--run`。控制器 IK 的点位和直线命令见[法兰目标发布接口](experiments/single_arm/control_interfaces/README.md)。
