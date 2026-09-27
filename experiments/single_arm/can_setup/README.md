# Nero 首次使用指南（CAN）

Nero 机械臂首次使用 CAN 通信的分步指南。项目环境和目录说明见 [README](../../../README.md)。

## 一、环境要求

- Ubuntu 20.04 / 22.04（推荐）/ 24.04。
- Python >= 3.6；本工作区使用 `nero-py310`（Python 3.10）。
- Python SDK `pyAgxArm` 已包含在 [environment.yml](../../../environment.yml) 中。在当前工作站使用现有环境：

```bash
conda activate nero-py310
```

- Ubuntu 的 CAN 工具通过 `apt` 安装，不能通过 `pip` 安装：

```bash
sudo apt install can-utils
```

## 二、硬件连接与操作

准备 Nero 机械臂、24V 适配器、USB-CAN 模块及 Ubuntu PC。将 CAN H/L 接到机械臂并接通电源。按照[官方 CAN 模块手册](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/can_user.md)启用 `can0`，确认接口状态：

```bash
ip -details link show can0
```

[Nero 用户手册](../../../docs/user_guide.md)第 2.1.1 节说明：开机红灯表示失能或异常，绿灯常亮表示已使能；因此不能把绿灯作为发送使能命令的前提。先读取状态和驱动反馈，排除具体故障。第 10 章说明外部 CAN 接口可直接控制机械臂；本机固件 1.21 已在未打开网页上位机时正常推送反馈并响应固件查询，使用 SDK 不需要网页上位机。

### 查询固件版本

如果尚不知道固件版本，在工作区根目录运行：

```bash
python -m experiments.single_arm.can_setup.detect_firmware --channel can0
```

脚本读取 `get_firmware()` 返回的 `software_version`，并通过 `resolve_firmware_profile()` 打印匹配的 SDK 驱动档位。脚本不调用 `enable()` 或运动指令。若没有响应，检查 `can0`、接线、供电，也可从机械臂网页上位机查看固件版本。

| 机械臂固件 | SDK 档位 |
| --- | --- |
| ≥ 1.21 | `NeroFW.V121` |
| 1.20 | `NeroFW.V120` |
| 1.12 | `NeroFW.V112` |
| 1.11 | `NeroFW.V111` |
| ≤ 1.10 | `NeroFW.DEFAULT` |

创建正式连接时，将相应档位传给 `firmeware_version`。该参数名是 SDK 当前使用的拼写。

### 固件 ≥ 1.12

固件 ≥ 1.12 上电后会开始 CAN 推送，无需网页开关或 `set_normal_mode()`。本工作站目前检测到固件 1.21，以下示例使用 `NeroFW.V121`；固件 1.20 改用 `NeroFW.V120`，1.12 改用 `NeroFW.V112`：

```python
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V121,
    channel="can0",
    local_loopback=True,
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

enabled = robot.enable(timeout=1.5)
print("关节使能成功：", enabled)
```

`local_loopback=True` 让同一电脑上的旁路 SocketCAN 监听器看到此进程发出的控制帧，便于[失能诊断](../../../docs/emergency_stop_recovery.md)；它不代表机械臂会自行回传这些帧。SDK 默认关闭本机回环，未显式开启时，`candump` 或监听脚本可能看不到本机发出的命令。

实际操作可先用本工作区的状态脚本检查反馈，再显式执行一次有超时的使能：

```bash
python -m experiments.single_arm.can_setup.nero_can_activate
python -m experiments.single_arm.can_setup.nero_can_activate --enable
```

`--enable` 会释放关节刹车，机械臂可能运动。执行前确认机械臂已固定、工作范围内无人和障碍物；脚本在关节错误、驱动欠压或故障时不会发送使能。使能后现场确认指示灯变绿、各关节使能反馈正常，再发送运动指令。

使能后关节会受电机控制；要手动拖动并记录可达位置，还需进入拖动示教或重力补偿模式，不能直接推已使能的机械臂。

在另一终端执行 `candump can0`，正常情况下能看到 CAN 数据。读取关节角时，可在连接与使能后调用：

```python
import time

while True:
    print(robot.get_joint_angles())
    time.sleep(0.01)
```

### 固件 ≤ 1.11

旧固件需要打开 CAN 推送。可以在网页上位机打开，或在使能循环中调用 `set_normal_mode()`：

```python
import time
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.DEFAULT,  # 固件 1.11 改用 NeroFW.V111
    channel="can0",
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    robot.set_normal_mode()
    time.sleep(0.01)
```

使能后执行 `candump can0` 验证数据流。

## 三、常见错误排查

- `candump can0` 没有数据：检查固件档位、CAN H/L 接线和剥线、接口波特率；固件 ≤ 1.11 还需确认 CAN 推送已打开。
- `connect()` 失败：确认 `can0` 存在且已启用。
- `ip -details link show can0` 显示 `ERROR-PASSIVE`：核对 1 Mbps 波特率、CAN H/L、接头及终端电阻；接口显示 UP 也可能存在总线错误。
- 开机后红灯：可能只是尚未使能；读取关节错误码、驱动故障和电压，确认没有其他异常后再使能。使能失败或绿灯未亮时再排查供电与 CAN 通信。
- `python -m experiments.single_arm.can_setup.nero_can_activate` 默认只读；`JOINT_BRAKE_NOT_RELEASED` 表示刹车尚未释放，单凭这一项不能判定需要重启。

首次上电前确保机械臂周围没有障碍物；未知状态下不要发送运动指令。

### 电子急停后的恢复

电子急停是阻尼停止，抬起的关节可能下落；`reset()` 会立刻失电，可能使抬起的关节立即掉落。先用 `python -m experiments.single_arm.can_setup.recover_emergency_stop` 只读核对状态和静止姿态。现场可靠支撑机械臂、清空下落范围后，才可显式加 `--run` 发送**一次**复位；脚本不会自动使能或回到 S1。完整步骤和 2026-09-24 实际急停记录见[电子急停与实验恢复](../../../docs/emergency_stop_recovery.md)。

## 四、通过 CAN 拖动示教

**2026-09-26 拖动事故后的有限恢复：**一次从桌边低位切入拖动后，机械臂在首条录制反馈前突然大幅上抬。现在只解锁从实测 S1 起步的限时 `teach_session --run`、统一 CLI `teach --run` 和 `quick teach/demo`。每次发帧前必须有新鲜状态、七轴使能和停稳的 S1 关节角与法兰位姿；安装方向帧后再检查一次，拖动切入后的前 0.5 秒若关节突变会请求退出拖动。S1 的 1 秒模式切换已实机平稳，但零目标会话的收尾修复尚未复测；**软件监测不能保证阻止瞬间异动。**桌边低位、`can_drag_teach --start`、手动恢复拖动、`quick home-demo/cube-sweep/cube-faces` 仍被拦截；`--stop` 与只读状态检查仍可用。Web 示教不受本工作区代码保护。

重力补偿可能参与了异动，但当前证据不能确定根因：S1 切入拖动的历史首帧稳定，桌边低位切入时首帧已经大幅偏移；两个模式帧之间缺少同步运动反馈。厂家说明关节零点若未设在直立默认位置，示教/主臂模式的重力补偿可能失常；左侧装以默认零点时网口朝下定义。还需核对安装方向、物理零点和固件行为；不要直接运行会移动至机械限位的自动零点校准。

当前工作区的 `experiments/single_arm/can_setup/can_drag_teach.py` 使用 Nero CAN 协议的 `0x151` 安装方向设置和 `0x150` 拖动示教开始/结束指令。默认只读检查。仅支持协议明确给出的水平正装、左侧装、右侧装；安装方向必须与实际一致，否则重力补偿方向可能错误。左侧装示例：

下面的手动开始命令仅保留协议示例，当前执行 `--start` 会被拦截；它**不会自动限时或回位**。当前仅从 S1 使用第五节的 `teach_session.py --run` 做有限时长试验，同一次示教不要混用两种脚本。

```bash
python -m experiments.single_arm.can_setup.can_drag_teach
python -m experiments.single_arm.can_setup.can_drag_teach --start --mount left
```

开始前确认七个关节已使能、错误码为零，机械臂底座固定、活动范围内无人和障碍物，并在现场扶稳机械臂。脚本会先下发安装方向，再启动拖动示教；仅在确认示教状态与关节角、法兰位姿反馈正常时报告成功。结束手动移动后运行：

```bash
python -m experiments.single_arm.can_setup.can_drag_teach --stop
```

如果未确认安装方向或模式切换后机械臂异常运动，停止操作并使用急停。控制器没有通过当前 SDK 提供安装方向的 CAN 读回接口，因此脚本只能验证下发后的状态反馈，无法证明安装方向设置已被控制器正确保存。

## 五、记录可达位置

需要保存手动移动过程中的状态时，在工作区根目录执行 `python -m experiments.single_arm.pose_recording.record_poses --channel can0`。脚本连续记录关节角及法兰位姿到 `experiments/single_arm/pose_recording/data/recordings/`，操作说明见[位姿采集实验](../pose_recording/README.md)。

结束示教后，可运行 `python -m experiments.single_arm.pose_recording.extract_pose_candidates <原始CSV路径>`，得到仅含正常拖动示教样本的轨迹，以及手动停留时的候选点位。自动回放前仍需检查关节限位和路径上的障碍物。

需要自动限制示教时长并返回示教前位置时，使用 `python -m experiments.single_arm.teaching.teach_session --run --max-seconds 90`。默认非零起点为现场确认的 S1；原 P4 配置保存在 `experiments/single_arm/config/nero_teach_p4.json`。参数见[配置文件](../config/nero_teach.json)和[示教实验说明](../teaching/README.md)。脚本在起点不匹配、反馈异常或返回路径不满足保护条件时停止自动运动。
