"""DaMiao classic-CAN protocol encoding and decoding.

The implementation follows DaMiao's V1.4 control protocol: standard 11-bit CAN,
1 Mbps, with MIT, position/velocity and velocity control modes.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import struct


class ProtocolError(ValueError):
    """Raised when a frame or command cannot be represented safely."""


class ControlMode(Enum):
    MIT = ("MIT", 0x000)
    POS_VEL = ("位置-速度", 0x100)
    VELOCITY = ("速度", 0x200)

    def __init__(self, label: str, can_offset: int) -> None:
        self.label = label
        self.can_offset = can_offset


@dataclass(frozen=True)
class MotorLimits:
    """Mapping ranges stored in the motor drive, not mechanical limits."""

    p_max: float = 12.5
    v_max: float = 30.0
    t_max: float = 10.0
    kp_max: float = 500.0
    kd_max: float = 5.0

    def validate(self) -> None:
        values = (self.p_max, self.v_max, self.t_max, self.kp_max, self.kd_max)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ProtocolError("所有映射范围必须是大于 0 的有限数值")


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes
    is_fd: bool = False
    bitrate_switch: bool = False

    def __post_init__(self) -> None:
        if not 0 <= self.arbitration_id <= 0x7FF:
            raise ProtocolError("经典 CAN 标准帧 ID 必须在 0x000~0x7FF")
        maximum = 64 if self.is_fd else 8
        if len(self.data) > maximum:
            raise ProtocolError(f"该 CAN 帧数据长度不能超过 {maximum} 字节")
        if self.bitrate_switch and not self.is_fd:
            raise ProtocolError("BRS 只能用于 CAN FD 帧")


@dataclass(frozen=True)
class MotorFeedback:
    motor_id: int
    state_code: int
    state_text: str
    position: float
    velocity: float
    torque: float
    mos_temperature: int
    rotor_temperature: int


STATE_TEXT = {
    0x0: "失能",
    0x1: "使能",
    0x8: "过压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "MOS 过温",
    0xC: "线圈过温",
    0xD: "通信丢失",
    0xE: "过载",
}


SPECIAL_DATA = {
    "enable": b"\xFF" * 7 + b"\xFC",
    "disable": b"\xFF" * 7 + b"\xFD",
    "set_zero": b"\xFF" * 7 + b"\xFE",
    "clear_error": b"\xFF" * 7 + b"\xFB",
}


def control_id(motor_id: int, mode: ControlMode) -> int:
    if not 0 <= motor_id <= 0x7FF:
        raise ProtocolError("电机 CAN ID 必须在 0x000~0x7FF")
    result = motor_id + mode.can_offset
    if result > 0x7FF:
        raise ProtocolError("电机 ID 加模式偏移后超过标准帧范围")
    return result


def _require_range(name: str, value: float, low: float, high: float) -> None:
    if not math.isfinite(value) or not low <= value <= high:
        raise ProtocolError(f"{name}={value:g} 超出允许范围 [{low:g}, {high:g}]")


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    _require_range("数值", value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    maximum = (1 << bits) - 1
    if not 0 <= value <= maximum:
        raise ProtocolError(f"{bits} 位原始值越界")
    return value * (high - low) / maximum + low


def pack_mit(
    motor_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
    limits: MotorLimits,
) -> CanFrame:
    limits.validate()
    _require_range("位置", position, -limits.p_max, limits.p_max)
    _require_range("速度", velocity, -limits.v_max, limits.v_max)
    _require_range("Kp", kp, 0.0, limits.kp_max)
    _require_range("Kd", kd, 0.0, limits.kd_max)
    _require_range("扭矩", torque, -limits.t_max, limits.t_max)

    p = float_to_uint(position, -limits.p_max, limits.p_max, 16)
    v = float_to_uint(velocity, -limits.v_max, limits.v_max, 12)
    kp_raw = float_to_uint(kp, 0.0, limits.kp_max, 12)
    kd_raw = float_to_uint(kd, 0.0, limits.kd_max, 12)
    t = float_to_uint(torque, -limits.t_max, limits.t_max, 12)
    data = bytes(
        (
            p >> 8,
            p & 0xFF,
            v >> 4,
            ((v & 0xF) << 4) | (kp_raw >> 8),
            kp_raw & 0xFF,
            kd_raw >> 4,
            ((kd_raw & 0xF) << 4) | (t >> 8),
            t & 0xFF,
        )
    )
    return CanFrame(control_id(motor_id, ControlMode.MIT), data)


def unpack_mit(data: bytes, limits: MotorLimits) -> tuple[float, float, float, float, float]:
    if len(data) != 8:
        raise ProtocolError("MIT 控制帧必须是 8 字节")
    p = (data[0] << 8) | data[1]
    v = (data[2] << 4) | (data[3] >> 4)
    kp = ((data[3] & 0xF) << 8) | data[4]
    kd = (data[5] << 4) | (data[6] >> 4)
    t = ((data[6] & 0xF) << 8) | data[7]
    return (
        uint_to_float(p, -limits.p_max, limits.p_max, 16),
        uint_to_float(v, -limits.v_max, limits.v_max, 12),
        uint_to_float(kp, 0.0, limits.kp_max, 12),
        uint_to_float(kd, 0.0, limits.kd_max, 12),
        uint_to_float(t, -limits.t_max, limits.t_max, 12),
    )


def pack_position_velocity(motor_id: int, position: float, velocity: float) -> CanFrame:
    if not math.isfinite(position) or not math.isfinite(velocity):
        raise ProtocolError("位置和速度必须是有限数值")
    return CanFrame(
        control_id(motor_id, ControlMode.POS_VEL),
        struct.pack("<ff", position, velocity),
    )


def pack_velocity(motor_id: int, velocity: float) -> CanFrame:
    if not math.isfinite(velocity):
        raise ProtocolError("速度必须是有限数值")
    return CanFrame(control_id(motor_id, ControlMode.VELOCITY), struct.pack("<f", velocity))


def pack_special(motor_id: int, command: str) -> CanFrame:
    """Pack a DM special command on the motor's unshifted base CAN ID.

    Motion frames use mode-specific IDs (ID, 0x100+ID, 0x200+ID), while
    enable, disable, zero and clear-error always target the base motor ID.
    """
    try:
        data = SPECIAL_DATA[command]
    except KeyError as exc:
        raise ProtocolError(f"未知特殊命令：{command}") from exc
    return CanFrame(control_id(motor_id, ControlMode.MIT), data)


def parse_feedback(frame: CanFrame, limits: MotorLimits) -> MotorFeedback:
    limits.validate()
    if len(frame.data) != 8:
        raise ProtocolError("反馈帧必须是 8 字节")
    data = frame.data
    state = data[0] >> 4
    motor_id = data[0] & 0x0F
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    return MotorFeedback(
        motor_id=motor_id,
        state_code=state,
        state_text=STATE_TEXT.get(state, f"未知状态 0x{state:X}"),
        position=uint_to_float(p, -limits.p_max, limits.p_max, 16),
        velocity=uint_to_float(v, -limits.v_max, limits.v_max, 12),
        torque=uint_to_float(t, -limits.t_max, limits.t_max, 12),
        mos_temperature=data[6],
        rotor_temperature=data[7],
    )


def pack_feedback(
    feedback_id: int,
    motor_id: int,
    state: int,
    position: float,
    velocity: float,
    torque: float,
    limits: MotorLimits,
    mos_temperature: int = 35,
    rotor_temperature: int = 32,
) -> CanFrame:
    """Build a feedback frame for the offline simulator and tests."""
    if not 0 <= motor_id <= 0xF:
        raise ProtocolError("反馈帧中的电机 ID 只能编码低 4 位")
    p = float_to_uint(position, -limits.p_max, limits.p_max, 16)
    v = float_to_uint(velocity, -limits.v_max, limits.v_max, 12)
    t = float_to_uint(torque, -limits.t_max, limits.t_max, 12)
    data = bytes(
        (
            ((state & 0xF) << 4) | motor_id,
            p >> 8,
            p & 0xFF,
            v >> 4,
            ((v & 0xF) << 4) | (t >> 8),
            t & 0xFF,
            max(0, min(255, mos_temperature)),
            max(0, min(255, rotor_temperature)),
        )
    )
    return CanFrame(feedback_id, data)


def parse_hex_data(text: str) -> bytes:
    compact = text.replace("0x", "").replace(",", " ").replace("-", " ")
    parts = compact.split()
    if not parts:
        return b""
    if len(parts) == 1 and len(parts[0]) > 2:
        token = parts[0]
        if len(token) % 2:
            raise ProtocolError("连续十六进制字符串必须包含偶数个字符")
        parts = [token[index : index + 2] for index in range(0, len(token), 2)]
    try:
        data = bytes(int(part, 16) for part in parts)
    except ValueError as exc:
        raise ProtocolError("数据区必须是十六进制字节，例如 FF FF 00 01") from exc
    if len(data) > 8:
        raise ProtocolError("经典 CAN 最多发送 8 字节")
    return data
