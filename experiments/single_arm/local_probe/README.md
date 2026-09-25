# 急停恢复后的局部关节对照实验

本实验从**运行时实测的当前姿态**出发，在 J4 高于 `+0.02 rad` 时让其向正角方向移动 `0.05 rad`，再沿同一关节小段返回实际起点。这样本次外行程不会从当前 J4 正角穿越零位。它不依赖旧 S1，也不重放 20 秒示教。脚本默认只读规划；显式 `--run` 才会发送运动指令。每次运行前重新检查 `CAN_CTRL`、`NORMAL`、七个驱动、静止姿态、关节限位和 SDK 正运动学与法兰反馈，并计算法兰中心预测范围。

先在工作区根目录、`nero-py310` 环境执行只读规划：

```bash
python -m experiments.single_arm.local_probe.local_joint_probe --speed-percent 5
```

确认输出的实时起点、J4 外行程目标和**整条连杆活动范围**，现场保持可靠支撑、底座固定、范围清空并持续监护后，才执行：

```bash
python -m experiments.single_arm.local_probe.local_joint_probe --speed-percent 5 --run
```

如果 5% 往返完成、现场无异常且独立只读检查仍在起点附近，可重新只读规划 10% 对照；再次核对现场条件后才运行：

```bash
python -m experiments.single_arm.local_probe.local_joint_probe --speed-percent 10
python -m experiments.single_arm.local_probe.local_joint_probe --speed-percent 10 --run
```

每步最多等待 5 秒，单关节到位与回位误差上限 `0.005 rad`，脚本记录各步耗时和回位误差到 `data/runs/`。已发送运动指令后若小步超时、状态异常或中止，公用执行器会发送 SDK 阻尼急停；关节可能因失去位置保持而下落，**不能假设仍在起点**。不能在失败后直接运行下一次对照；先按[急停恢复文档](../../../docs/emergency_stop_recovery.md)检查实时状态和现场姿态。

该规划只核对关节限位和法兰中心 FK。配置中的 `flange_workspace_m` 目前为 `null`，脚本无法判断其他连杆与现场物体的碰撞距离。这里的 5% 与 10% 用于短行程对照，不代表已经证明原 20 秒轨迹可以安全提速。实际结果见[实验报告](report.md)。
