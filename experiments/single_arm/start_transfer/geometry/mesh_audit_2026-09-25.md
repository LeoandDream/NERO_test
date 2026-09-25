# 临时起点至 S1：厂商碰撞网格离线初筛

使用 [AgileX Nero URDF 与 STL 碰撞网格](https://github.com/agilexrobotics/agx_arm_urdf/tree/f6642ce0d7872c686f29c99e9e10cd23d1d49313/nero) 的固定版本 `f6642ce0d7872c686f29c99e9e10cd23d1d49313`。八个 STL 网格保存在本机 `/home/leo/.cache/nero_collision_audit/meshes/`，分析依赖安装在 `/home/leo/.cache/nero_collision_audit/python/`，未修改 `nero-py310` 环境。脚本会逐个核对网格的 SHA-256，防止混用其他版本。关节起点取 [2 mm 回程报告](../../control_interfaces/data/runs/nero_move_l_20260924T151836Z.json)的最后实际反馈，终点取配置中的历史 S1。

比较了两条**假定关节线性插值**路线：直接由起点到 S1，以及先到 `J2=+0.200 rad、J4=−0.299516 rad、J7=−0.473438 rad` 的中间姿态、再到 S1。每段含首尾共 101 个采样姿态。对厂商 URDF 中的八个裸臂碰撞网格逐姿态更新变换，使用 FCL 检查所有**非相邻**连杆对；相邻连杆的网格接触属模型结构，不纳入这一判定。

| 路线 | 采样姿态数 | 非相邻连杆网格相交 | 采样中最小网格间距 |
| --- | ---: | ---: | ---: |
| 单个 S1 关节目标 | 101 | 0 | `0.01206 m`（link5 与 link7） |
| 两个关节目标 | 202 | 0 | `0.01206 m`（link5 与 link7） |

两条路线次小间距均约 `0.01910 m`（底座与 link2）。先前包围盒初筛在几组非相邻连杆间出现重叠；FCL 网格检查表明这些**采样姿态**中实体网格没有相交。[完整检查结果](../data/diagnostics/nero_mesh_audit_20260925T063209Z.json)保留了 21 组非相邻连杆的最小间距。

从工作区根目录可使用[审计脚本](audit_self_collision.py)重新生成报告；命令只读取文件，不连接 CAN：

```bash
PYTHONPATH=/home/leo/.cache/nero_collision_audit/python python -m experiments.single_arm.start_transfer.geometry.audit_self_collision \
  --plan experiments/single_arm/start_transfer/data/plans/nero_two_target_offline_20260924T160320Z.json \
  --mesh-dir /home/leo/.cache/nero_collision_audit/meshes
```

本机缓存若被清理，需从上方固定版本的厂商仓库重新获取八个 STL，并在独立路径安装 `numpy`、`trimesh`、`python-fcl`；脚本会拒绝哈希不符的网格。这仍只是模型和离散采样的结果：不能覆盖采样间可能的接触、控制器实际插值、安装支撑、线缆、台面、人员或未来夹爪/工具。两目标候选仍标为 `executable=false`，不能由这份初筛直接发运动命令。
