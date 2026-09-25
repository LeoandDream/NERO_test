# 位姿采集实验

在工作区根目录、`nero-py310` 环境中运行。记录脚本只读取 CAN 反馈，不切换机械臂控制模式；手动拖动前需先按 [CAN 实验说明](../can_setup/README.md)进入相应模式。

## 连续记录

```bash
python -m experiments.single_arm.pose_recording.record_poses --channel can0
```

默认以 10 Hz 采样，最长 30 分钟。`Ctrl+C` 提前结束；`--rate`、`--duration`、`--output` 可调整参数。新 CSV 写入 `experiments/single_arm/pose_recording/data/recordings/`，记录 UTC 时间、七关节角和法兰位姿。只有反馈新鲜且完整时才写入一行。

## 提取停留点与法兰范围

```bash
python -m experiments.single_arm.pose_recording.extract_pose_candidates experiments/single_arm/pose_recording/data/recordings/nero_poses_20260923T140013Z.csv
python -m experiments.single_arm.pose_recording.analyze_workspace experiments/single_arm/pose_recording/data/recordings/nero_poses_20260923T140013Z_teaching_only.csv --output experiments/single_arm/pose_recording/data/measurements/新观测范围.json
```

提取工具输出示教状态轨迹和停留候选点；分析工具用 SDK 正运动学复核法兰坐标范围。候选点只说明机械臂曾到达该位置，自动控制前仍需检查完整路径和其他连杆。

实测结果见[实验报告](report.md)，对安全边界的解释见[法兰范围复核](../teaching/flange_workspace_assessment.md)。
