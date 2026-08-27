"""Classic SocketCAN and deterministic in-process virtual transports."""

from __future__ import annotations

from abc import ABC, abstractmethod
import queue
import os
import select
import socket
import struct
import termios
import threading
import time

from .protocol import (
    CanFrame,
    ControlMode,
    MotorLimits,
    SPECIAL_DATA,
    pack_feedback,
    unpack_mit,
)


CAN_FRAME = struct.Struct("=IB3x8s")
CANFD_FRAME = struct.Struct("=IBBBB64s")
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CANFD_BRS = 0x01

DM_SERIAL_TX_TEMPLATE = bytes(
    (0x55, 0xAA, 0x1E, 0x03, 0x01, 0, 0, 0, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x08, 0, 0)
    + (0,) * 8
    + (0,)
)


def encode_dm_serial_frame(frame: CanFrame) -> bytes:
    """Wrap one classic CAN frame for the official DM USB2CAN serial bridge."""
    if frame.is_fd:
        raise OSError("DM USB2CAN 串口桥不支持 CAN FD")
    packet = bytearray(DM_SERIAL_TX_TEMPLATE)
    packet[13:17] = frame.arbitration_id.to_bytes(4, "little")
    packet[18] = len(frame.data)
    packet[21:29] = frame.data.ljust(8, b"\0")
    return bytes(packet)


def extract_dm_serial_frame(buffer: bytearray) -> CanFrame | None:
    """Extract one 16-byte adapter RX packet, resynchronizing on AA ... 55."""
    while len(buffer) >= 16:
        try:
            start = buffer.index(0xAA)
        except ValueError:
            buffer.clear()
            return None
        if start:
            del buffer[:start]
        if len(buffer) < 16:
            return None
        if buffer[15] != 0x55:
            del buffer[0]
            continue
        packet = bytes(buffer[:16])
        del buffer[:16]
        arbitration_id = int.from_bytes(packet[3:7], "little") & CAN_SFF_MASK
        return CanFrame(arbitration_id, packet[7:15])
    return None


class Transport(ABC):
    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def send(self, frame: CanFrame) -> None: ...

    @abstractmethod
    def recv(self, timeout: float) -> CanFrame | None: ...


class SocketCanTransport(Transport):
    def __init__(self, channel: str, allow_fd: bool = False) -> None:
        self.channel = channel.strip()
        self.allow_fd = allow_fd
        self._socket: socket.socket | None = None
        self._send_lock = threading.Lock()

    def open(self) -> None:
        if not self.channel:
            raise OSError("SocketCAN 接口名不能为空")
        if not hasattr(socket, "PF_CAN"):
            raise OSError("当前系统不支持 SocketCAN")
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        if self.allow_fd:
            sol_can_raw = getattr(socket, "SOL_CAN_RAW", 101)
            can_raw_fd_frames = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
            can_socket.setsockopt(sol_can_raw, can_raw_fd_frames, 1)
        can_socket.settimeout(0.2)
        try:
            can_socket.bind((self.channel,))
        except Exception:
            can_socket.close()
            raise
        self._socket = can_socket

    def close(self) -> None:
        can_socket, self._socket = self._socket, None
        if can_socket is not None:
            can_socket.close()

    def send(self, frame: CanFrame) -> None:
        if self._socket is None:
            raise OSError("CAN 接口尚未连接")
        if frame.is_fd:
            if not self.allow_fd:
                raise OSError("当前连接未启用 CAN FD")
            flags = CANFD_BRS if frame.bitrate_switch else 0
            packet = CANFD_FRAME.pack(
                frame.arbitration_id,
                len(frame.data),
                flags,
                0,
                0,
                frame.data.ljust(64, b"\0"),
            )
        else:
            packet = CAN_FRAME.pack(frame.arbitration_id, len(frame.data), frame.data.ljust(8, b"\0"))
        with self._send_lock:
            self._socket.send(packet)

    def recv(self, timeout: float) -> CanFrame | None:
        if self._socket is None:
            return None
        self._socket.settimeout(timeout)
        try:
            packet = self._socket.recv(CAN_FRAME.size)
        except socket.timeout:
            return None
        except OSError:
            if self._socket is None:
                return None
            raise
        is_fd = len(packet) == CANFD_FRAME.size
        if is_fd:
            can_id, length, flags, _res0, _res1, data = CANFD_FRAME.unpack(packet)
        elif len(packet) == CAN_FRAME.size:
            can_id, length, data = CAN_FRAME.unpack(packet)
            flags = 0
        else:
            return None
        if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
            return None
        maximum = 64 if is_fd else 8
        return CanFrame(
            can_id & CAN_SFF_MASK,
            data[: min(length, maximum)],
            is_fd=is_fd,
            bitrate_switch=bool(flags & CANFD_BRS),
        )


class DmSerialTransport(Transport):
    """DaMiao USB2CAN CDC serial bridge (`/dev/ttyACM*`, 921600 baud)."""

    def __init__(self, port: str, baudrate: int = 921600) -> None:
        self.port = port.strip()
        self.baudrate = baudrate
        self._fd: int | None = None
        self._rx_buffer = bytearray()
        self._send_lock = threading.Lock()

    def open(self) -> None:
        if not self.port:
            raise OSError("串口路径不能为空")
        if self.baudrate != 921600:
            raise OSError("当前 DM USB2CAN 后端仅支持官方默认的 921600 波特率")
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            attrs[4] = termios.B921600
            attrs[5] = termios.B921600
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIFLUSH)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd
        self._rx_buffer.clear()

    def close(self) -> None:
        fd, self._fd = self._fd, None
        if fd is not None:
            os.close(fd)

    def send(self, frame: CanFrame) -> None:
        fd = self._fd
        if fd is None:
            raise OSError("DM USB2CAN 串口尚未连接")
        packet = encode_dm_serial_frame(frame)
        with self._send_lock:
            position = 0
            while position < len(packet):
                _readable, writable, _errors = select.select([], [fd], [], 0.5)
                if not writable:
                    raise TimeoutError("DM USB2CAN 串口写入超时")
                try:
                    position += os.write(fd, packet[position:])
                except BlockingIOError:
                    continue

    def recv(self, timeout: float) -> CanFrame | None:
        fd = self._fd
        if fd is None:
            return None
        frame = extract_dm_serial_frame(self._rx_buffer)
        if frame is not None:
            return frame
        readable, _writable, _errors = select.select([fd], [], [], timeout)
        if not readable:
            return None
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            return None
        if chunk:
            self._rx_buffer.extend(chunk)
        return extract_dm_serial_frame(self._rx_buffer)


class VirtualTransport(Transport):
    """A small motor simulator used to validate the UI without hardware."""

    def __init__(
        self,
        motor_id: int,
        feedback_id: int,
        limits: MotorLimits,
    ) -> None:
        self.motor_id = motor_id
        self.feedback_id = feedback_id
        self.limits = limits
        self._opened = False
        self._queue: queue.Queue[CanFrame] = queue.Queue()
        self._state = 0
        self._position = 0.0
        self._velocity = 0.0
        self._torque = 0.0
        self._last_update = time.monotonic()

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def send(self, frame: CanFrame) -> None:
        if not self._opened:
            raise OSError("虚拟总线尚未连接")
        offsets = {mode.can_offset: mode for mode in ControlMode}
        offset = frame.arbitration_id - self.motor_id
        mode = offsets.get(offset)
        if mode is None:
            return

        if len(frame.data) == 8 and frame.data[:7] == b"\xFF" * 7:
            if frame.data == SPECIAL_DATA["enable"]:
                self._state = 1
            elif frame.data == SPECIAL_DATA["disable"]:
                self._state = 0
                self._velocity = 0.0
                self._torque = 0.0
            elif frame.data == SPECIAL_DATA["clear_error"]:
                self._state = 0
            elif frame.data == SPECIAL_DATA["set_zero"]:
                self._position = 0.0
        elif self._state == 1:
            self._apply_control(mode, frame.data)
        self._emit_feedback()

    def _apply_control(self, mode: ControlMode, data: bytes) -> None:
        now = time.monotonic()
        elapsed = min(now - self._last_update, 0.1)
        self._last_update = now
        if mode is ControlMode.MIT and len(data) == 8:
            position, velocity, _kp, _kd, torque = unpack_mit(data, self.limits)
            self._position = position
            self._velocity = velocity
            self._torque = torque
        elif mode is ControlMode.POS_VEL and len(data) == 8:
            self._position, self._velocity = struct.unpack("<ff", data)
            self._position = max(-self.limits.p_max, min(self.limits.p_max, self._position))
            self._velocity = max(-self.limits.v_max, min(self.limits.v_max, self._velocity))
        elif mode is ControlMode.VELOCITY and len(data) == 4:
            (self._velocity,) = struct.unpack("<f", data)
            self._velocity = max(-self.limits.v_max, min(self.limits.v_max, self._velocity))
            self._position += self._velocity * elapsed
            self._position = max(-self.limits.p_max, min(self.limits.p_max, self._position))

    def _emit_feedback(self) -> None:
        self._queue.put(
            pack_feedback(
                self.feedback_id,
                self.motor_id & 0xF,
                self._state,
                self._position,
                self._velocity,
                self._torque,
                self.limits,
            )
        )

    def recv(self, timeout: float) -> CanFrame | None:
        if not self._opened:
            return None
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None
