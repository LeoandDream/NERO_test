# Nero 几何模型来源

`nero_description.urdf` 与 `LICENSE.agx_arm_urdf` 来自 AgileX 的 [agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf/tree/f6642ce0d7872c686f29c99e9e10cd23d1d49313/nero)，固定版本 `f6642ce0d7872c686f29c99e9e10cd23d1d49313`。URDF 的 SHA-256 为 `c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278`。

本目录保留 URDF、许可文件和离线检查脚本；STL 网格放在本机独立缓存，不随项目代码运行。`urdf_origins.py` 读取七个关节原点，计算 `link1` 至 `link7` 的原点位置。[网格审计](mesh_audit_2026-09-25.md)使用厂商 STL 检查离散姿态下的裸臂非相邻连杆。原点范围和离散网格检查都不能单独证明实际运动路线无碰撞；现场支撑、线缆、台面及工具尚未建模。
