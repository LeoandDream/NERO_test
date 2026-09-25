"""监看 Nero 关节反馈；此脚本不会激活 Linux 的 can0 接口。"""

import time
from platform import system
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

# Nero 固件：≤ 1.10 选 DEFAULT；1.11 选 V111；1.12 选 V112；1.20 选 V120；≥ 1.21 选 V121。
platform_system = system()
if platform_system == "Windows":
    interface = "agx_cando"
    channel = "0"
elif platform_system == "Linux":
    interface = "socketcan"
    channel = "can0"
elif platform_system == "Darwin":
    interface = "slcan"
    channel = "/dev/ttyACM0"
else:
    raise RuntimeError("pyAgxArm 当前公开说明包含 Linux `socketcan`、Windows `agx_cando` 与 macOS `slcan`。")

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V121,
    interface=interface,
    channel=channel,
    local_loopback=(interface == "socketcan"),
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

last_timestamp = None
last_notice = time.monotonic()
last_status = None
try:
    while True:
        status = robot.get_arm_status()
        if status is not None:
            current_status = (
                str(status.msg.ctrl_mode),
                str(status.msg.arm_status),
                str(status.msg.teach_status),
            )
            if current_status != last_status:
                print("控制模式 / 机械臂状态 / 示教状态：", *current_status)
                last_status = current_status

        ja = robot.get_joint_angles()
        if ja is not None and ja.timestamp != last_timestamp:
            print(ja.msg)
            print(ja.hz, ja.timestamp)
            last_timestamp = ja.timestamp
        elif ja is None and time.monotonic() - last_notice >= 2:
            print("尚未收到关节角反馈；请检查机械臂供电、CAN 接线与数据推送。")
            last_notice = time.monotonic()
        time.sleep(0.05)
finally:
    robot.disconnect()
