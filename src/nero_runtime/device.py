"""固定 Nero V121 的显式只读来源生命周期；导入本模块不导入 SDK。"""

from contextlib import contextmanager


class DeviceIOError(RuntimeError):
    """预期的设备连接、读取或关闭失败，保留两个阶段的原因。"""

    def __init__(self, phase: str, primary: BaseException,
                 cleanup: BaseException | None = None):
        self.phase, self.primary, self.cleanup = phase, primary, cleanup
        message = f"{phase}失败：{type(primary).__name__}: {primary}"
        if cleanup is not None:
            message += f"；关闭通信失败：{type(cleanup).__name__}: {cleanup}"
        super().__init__(message)


class DeviceCancellation(KeyboardInterrupt):
    """取消读数时关闭失败；取消仍是主结果。"""

    def __init__(self, cleanup: BaseException):
        self.cleanup = cleanup
        super().__init__(f"关闭通信失败：{type(cleanup).__name__}: {cleanup}")


def validate_channel(channel: str) -> str:
    if type(channel) is not str or not channel or channel.strip() != channel:
        raise ValueError("CAN channel must be a non-empty name without surrounding space")
    if any(character.isspace() or ord(character) < 32 for character in channel):
        raise ValueError("CAN channel contains whitespace or control characters")
    return channel


@contextmanager
def connected_nero(channel: str = "can0", *, config_factory=None,
                   driver_factory=None):
    """按明确请求打开 SDK 来源，退出时仅关闭通信，不做控制清理。

    注入的两个 factory 仅供离线测试；真实路径在调用时才导入 pyAgxArm。
    SDK connect() 没有经本项目验证的整体墙钟超时保证。
    """
    channel = validate_channel(channel)
    if (config_factory is None) != (driver_factory is None):
        raise ValueError("config_factory and driver_factory must be supplied together")
    if config_factory is None:
        from pyAgxArm import AgxArmFactory, create_agx_arm_config
        config_factory = create_agx_arm_config
        driver_factory = AgxArmFactory.create_arm
    config = config_factory(
        robot="nero", firmeware_version="v121", channel=channel,
        auto_connect=False, enable_check_can=False, timeout=0.25,
    )
    robot = driver_factory(config)
    phase = "连接"
    try:
        robot.connect()
        phase = "读取"
        yield robot
    except BaseException as primary:
        expected_io = isinstance(primary, OSError) or (
            phase == "连接" and type(primary) is RuntimeError and
            str(primary) == "Failed to establish robot communication.")
        try:
            # 只关闭通信和 SDK 后台线程；不做任何控制清理。
            robot.disconnect()
        except (OSError, RuntimeError) as cleanup:
            if isinstance(primary, KeyboardInterrupt):
                raise DeviceCancellation(cleanup) from primary
            if expected_io:
                raise DeviceIOError(phase, primary, cleanup) from primary
            # 未分类的编程错误仍保持原类型，异常链保留关闭失败。
            raise primary from cleanup
        if expected_io:
            raise DeviceIOError(phase, primary) from primary
        raise
    else:
        try:
            robot.disconnect()
        except (OSError, RuntimeError) as cleanup:
            raise DeviceIOError("关闭通信", cleanup) from cleanup
