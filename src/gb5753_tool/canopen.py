"""Safe, read-oriented CANopen/CiA-402 helpers for the GB5753.

Until the vendor communication manual is available this module intentionally does
not expose state-machine writes, PDO reconfiguration, parameter storage or NMT.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct

from .protocol import CanFrame, ProtocolError


SDO_ABORTS = {
    0x05030000: "Toggle 位未交替",
    0x05040000: "SDO 协议超时",
    0x05040001: "客户端/服务器命令字无效",
    0x06010000: "不支持访问该对象",
    0x06010001: "尝试读取只写对象",
    0x06010002: "尝试写入只读对象",
    0x06020000: "对象字典中不存在该对象",
    0x06090011: "子索引不存在",
    0x06090030: "数值范围超限",
    0x08000000: "一般错误",
}


@dataclass(frozen=True)
class SdoResponse:
    node_id: int
    index: int
    sub_index: int
    data: bytes = b""
    abort_code: int | None = None
    message: str = ""


@dataclass(frozen=True)
class Cia402Snapshot:
    source: str
    values: dict[str, int | float]


def sdo_upload_request(node_id: int, index: int, sub_index: int) -> CanFrame:
    if not 1 <= node_id <= 127:
        raise ProtocolError("CANopen Node ID 必须在 1~127")
    if not 0 <= index <= 0xFFFF or not 0 <= sub_index <= 0xFF:
        raise ProtocolError("对象索引或子索引越界")
    data = bytes((0x40, index & 0xFF, index >> 8, sub_index, 0, 0, 0, 0))
    # The GB5753 object dictionary explicitly enables CAN FD with BRS by default.
    return CanFrame(0x600 + node_id, data, is_fd=True, bitrate_switch=True)


def parse_sdo_response(frame: CanFrame) -> SdoResponse | None:
    if not 0x581 <= frame.arbitration_id <= 0x5FF or len(frame.data) < 8:
        return None
    node_id = frame.arbitration_id - 0x580
    command = frame.data[0]
    index = frame.data[1] | (frame.data[2] << 8)
    sub_index = frame.data[3]
    if command == 0x80:
        code = struct.unpack("<I", frame.data[4:8])[0]
        return SdoResponse(
            node_id,
            index,
            sub_index,
            abort_code=code,
            message=SDO_ABORTS.get(code, "未知 SDO 中止码"),
        )
    if command & 0xE0 != 0x40:
        return None
    expedited = bool(command & 0x02)
    size_indicated = bool(command & 0x01)
    if not expedited:
        return SdoResponse(node_id, index, sub_index, message="分段 SDO 回包（当前只显示首帧）")
    unused = (command >> 2) & 0x03 if size_indicated else 0
    return SdoResponse(node_id, index, sub_index, frame.data[4 : 8 - unused])


def decode_value(data: bytes, data_type: str) -> int | float | str:
    formats = {
        "Uint8": "<B",
        "Int8": "<b",
        "Uint16": "<H",
        "Int16": "<h",
        "Uint32": "<I",
        "Int32": "<i",
        "Float32": "<f",
    }
    fmt = formats.get(data_type)
    if fmt is None:
        return data.hex(" ").upper()
    size = struct.calcsize(fmt)
    if len(data) < size:
        return data.hex(" ").upper()
    return struct.unpack(fmt, data[:size])[0]


def parse_default_tpdo(frame: CanFrame, node_id: int) -> Cia402Snapshot | None:
    if frame.arbitration_id == 0x180 + node_id and len(frame.data) >= 5:
        error, status, mode = struct.unpack("<HHb", frame.data[:5])
        return Cia402Snapshot("TPDO1", {"错误码": error, "状态字": status, "模式": mode})
    if frame.arbitration_id == 0x280 + node_id and len(frame.data) >= 8:
        position, velocity = struct.unpack("<ff", frame.data[:8])
        return Cia402Snapshot("TPDO2", {"位置(Rev)": position, "速度(Rev/s)": velocity})
    if frame.arbitration_id == 0x380 + node_id and len(frame.data) >= 6:
        torque, drive_temp, motor_temp = struct.unpack("<hhh", frame.data[:6])
        return Cia402Snapshot(
            "TPDO3",
            {"力矩(峰值‰)": torque, "驱动温度(0.1℃)": drive_temp, "电机温度(0.1℃)": motor_temp},
        )
    return None
