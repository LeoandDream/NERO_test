# 单臂实验

当前设备为 Nero，左侧安装，CAN 通道 `can0`，固件 1.21。所有命令均从工作区根目录执行，并先激活 `nero-py310`。共同配置在 [config/nero_teach.json](config/nero_teach.json)；S1 是现场确认并经回位验证的非零起点。`flange_workspace_m` 目前为 `null`，历史采样范围不能当成自动碰撞保护。

| 顺序 | 实验 | 用途 | 报告 |
| --- | --- | --- | --- |
| 1 | [CAN 连通与状态](can_setup/README.md) | 查询固件、检查驱动、按需使能 | [报告](can_setup/report.md) |
| 2 | [位姿采集](pose_recording/README.md) | 连续记录关节与法兰、提取停留点 | [报告](pose_recording/report.md) |
| 3 | [拖动示教](teaching/README.md) | 限时记录、停稳补录与原路回位 | [报告](teaching/report.md) |
| 4 | [移动到 S1](start_transfer/README.md) | 从实时姿态规划小步关节路线 | [报告](start_transfer/report.md) |
| 5 | [轨迹回放](replay/README.md) | 回放完整示教记录并返回 | [报告](replay/report.md) |
| 6 | [法兰 X 方向运动](cartesian_x/README.md) | 用数值逆解自主计算轨迹 | [报告](cartesian_x/report.md) |
| 7 | [恢复后局部对照](local_probe/README.md) | 当前起点 J4 小行程，比较 5% 与 10% 速度 | [报告](local_probe/report.md) |
| 8 | [控制器 IK 点位与直线](control_interfaces/README.md) | 预检后单次发布法兰目标 `move_p` 或 `move_l` | [报告](control_interfaces/report.md) |
| 9 | [连续示教回放](continuous_replay/README.md) | 完整 20 秒源轨迹往返、20 Hz 目标与 CAN/反馈独立验收通过 | [报告](continuous_replay/report.md) |
| 10 | [可达空间与初始化候选](reachability/README.md) | 16 段新探测及历史边缘接近已验收；候选姿态尚待精确到位和外观判断 | [报告](reachability/report.md) |

离线测试命令：

```bash
python -m unittest discover -s experiments/single_arm -p 'test_*.py' -q
```

实机动作需重新检查底座固定、安装方向、关节零点、整条连杆活动范围，并有人现场监护。已执行的规划文件不能复用。
