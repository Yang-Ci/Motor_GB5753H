"""PyQt5 desktop interface for the DM-compatible GB5753H sample."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import threading
import time

from PyQt5.QtCore import QObject, QRectF, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .canopen import decode_value, parse_default_tpdo, parse_sdo_response, sdo_upload_request
from .object_dictionary import ObjectEntry, load_xlsx
from .protocol import (
    CanFrame,
    ControlMode,
    MotorLimits,
    ProtocolError,
    pack_mit,
    pack_position_velocity,
    pack_special,
    pack_velocity,
    parse_feedback,
    parse_hex_data,
)
from .transport import DmSerialTransport, SocketCanTransport, Transport, VirtualTransport


ROOT = Path(__file__).resolve().parents[2]
OBJECT_DICTIONARY = ROOT / "华翼关节模组对象字典V11(只读).xlsx"


class CanSession(QObject):
    frame_received = pyqtSignal(object)
    error = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self) -> None:
        super().__init__()
        self.transport: Transport | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def connect_transport(self, transport: Transport) -> None:
        self.disconnect_transport()
        transport.open()
        self.transport = transport
        self._stop.clear()
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        self.connection_changed.emit(True)

    def disconnect_transport(self) -> None:
        self._stop.set()
        transport, self.transport = self.transport, None
        if transport is not None:
            transport.close()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.4)
        if transport is not None:
            self.connection_changed.emit(False)

    def send(self, frame: CanFrame) -> None:
        if self.transport is None:
            raise OSError("尚未连接 CAN 接口")
        self.transport.send(frame)

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            transport = self.transport
            if transport is None:
                return
            try:
                frame = transport.recv(0.15)
                if frame is not None:
                    self.frame_received.emit(frame)
            except Exception as exc:
                if not self._stop.is_set():
                    self.error.emit(str(exc))
                return


def _hex_spin(maximum: int, value: int) -> QSpinBox:
    widget = QSpinBox()
    widget.setRange(0, maximum)
    widget.setDisplayIntegerBase(16)
    widget.setPrefix("0x")
    widget.setValue(value)
    widget.setMinimumWidth(105 if maximum > 0xFFF else 80)
    return widget


def _float_spin(minimum: float, maximum: float, value: float, decimals: int = 4) -> QDoubleSpinBox:
    widget = QDoubleSpinBox()
    widget.setDecimals(decimals)
    widget.setRange(minimum, maximum)
    widget.setValue(value)
    widget.setSingleStep(max((maximum - minimum) / 100.0, 0.001))
    return widget


class TemperatureChart(QWidget):
    """Dependency-free scrolling chart for drive and motor temperatures."""

    def __init__(self) -> None:
        super().__init__()
        self.samples: list[tuple[datetime, float, float]] = []
        self.warning_threshold = 70.0
        self.alarm_threshold = 85.0
        self.window_seconds = 120.0
        self.setMinimumHeight(280)

    def set_samples(self, samples: list[tuple[datetime, float, float]]) -> None:
        self.samples = samples
        self.update()

    def set_thresholds(self, warning: float, alarm: float) -> None:
        self.warning_threshold = warning
        self.alarm_threshold = alarm
        self.update()

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))

        plot = QRectF(52, 28, max(10, self.width() - 70), max(10, self.height() - 66))
        painter.setPen(QPen(QColor("#adb5bd"), 1))
        painter.drawRect(plot)

        maximum_sample = max(
            [self.alarm_threshold + 5.0]
            + [max(drive, motor) for _stamp, drive, motor in self.samples[-6000:]]
        )
        y_max = max(100.0, float((int(maximum_sample / 10.0) + 1) * 10))
        for index in range(6):
            temperature = y_max * index / 5.0
            y = plot.bottom() - plot.height() * temperature / y_max
            painter.setPen(QPen(QColor("#e9ecef"), 1))
            painter.drawLine(int(plot.left()), int(y), int(plot.right()), int(y))
            painter.setPen(QColor("#495057"))
            painter.drawText(4, int(y + 5), f"{temperature:.0f}℃")

        now = self.samples[-1][0].timestamp() if self.samples else time.time()
        start = now - self.window_seconds
        painter.setPen(QColor("#495057"))
        painter.drawText(int(plot.left()), self.height() - 12, f"最近 {int(self.window_seconds)} 秒")
        painter.drawText(int(plot.right() - 28), self.height() - 12, "现在")

        def map_y(value: float) -> float:
            return plot.bottom() - plot.height() * max(0.0, min(y_max, value)) / y_max

        for threshold, color, label in (
            (self.warning_threshold, QColor("#f0ad00"), "预警"),
            (self.alarm_threshold, QColor("#dc3545"), "报警"),
        ):
            y = map_y(threshold)
            pen = QPen(color, 1, Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(int(plot.left()), int(y), int(plot.right()), int(y))
            painter.drawText(int(plot.right() - 78), int(y - 3), f"{label} {threshold:.0f}℃")

        visible = [sample for sample in self.samples if sample[0].timestamp() >= start]
        if not visible:
            painter.setPen(QColor("#6c757d"))
            painter.drawText(plot, Qt.AlignCenter, "等待 DM 反馈温度数据…")
        else:
            def draw_series(value_index: int, color: str) -> None:
                path = QPainterPath()
                for index, sample in enumerate(visible):
                    x = plot.left() + plot.width() * (sample[0].timestamp() - start) / self.window_seconds
                    y = map_y(sample[value_index])
                    if index == 0:
                        path.moveTo(x, y)
                    else:
                        path.lineTo(x, y)
                painter.setPen(QPen(QColor(color), 2))
                painter.drawPath(path)

            draw_series(1, "#dc3545")
            draw_series(2, "#0d6efd")

        painter.setPen(QPen(QColor("#dc3545"), 3))
        painter.drawLine(int(plot.left()), 13, int(plot.left() + 24), 13)
        painter.setPen(QColor("#212529"))
        painter.drawText(int(plot.left() + 30), 18, "驱动温度")
        painter.setPen(QPen(QColor("#0d6efd"), 3))
        painter.drawLine(int(plot.left() + 110), 13, int(plot.left() + 134), 13)
        painter.setPen(QColor("#212529"))
        painter.drawText(int(plot.left() + 140), 18, "电机温度")


@dataclass(frozen=True)
class MotionTarget:
    mode: str
    position: float | None
    velocity: float | None
    torque: float | None


@dataclass(frozen=True)
class MotionSample:
    timestamp: datetime
    mode: str
    target_position: float | None
    actual_position: float
    target_velocity: float | None
    actual_velocity: float
    target_torque: float | None
    actual_torque: float


class MotionChart(QWidget):
    """Scrolling target/actual plots for position, velocity and torque."""

    SERIES = (
        ("位置", "rad", "target_position", "actual_position"),
        ("速度", "rad/s", "target_velocity", "actual_velocity"),
        ("力矩", "Nm", "target_torque", "actual_torque"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.samples: list[MotionSample] = []
        self.window_seconds = 120.0
        self.setMinimumHeight(350)

    def set_samples(self, samples: list[MotionSample]) -> None:
        self.samples = samples
        self.update()

    @staticmethod
    def _format_axis(value: float) -> str:
        magnitude = abs(value)
        if magnitude >= 100:
            return f"{value:.0f}"
        if magnitude >= 10:
            return f"{value:.1f}"
        return f"{value:.2f}"

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))

        left = 70.0
        right = max(left + 10.0, float(self.width() - 18))
        top = 42.0
        bottom = max(top + 30.0, float(self.height() - 30))
        gap = 20.0
        panel_height = max(30.0, (bottom - top - gap * 2) / 3.0)
        now = self.samples[-1].timestamp.timestamp() if self.samples else time.time()
        start = now - self.window_seconds
        visible = [sample for sample in self.samples if sample.timestamp.timestamp() >= start][-6000:]
        maximum_draw_points = max(300, int(right - left) * 2)
        if len(visible) > maximum_draw_points:
            stride = max(1, len(visible) // maximum_draw_points)
            reduced = visible[::stride]
            if reduced[-1] is not visible[-1]:
                reduced.append(visible[-1])
            visible = reduced

        for panel_index, (title, unit, target_attr, actual_attr) in enumerate(self.SERIES):
            panel_top = top + panel_index * (panel_height + gap)
            plot = QRectF(left, panel_top, right - left, panel_height)
            painter.setPen(QPen(QColor("#adb5bd"), 1))
            painter.drawRect(plot)

            values: list[float] = []
            for sample in visible:
                actual = float(getattr(sample, actual_attr))
                target = getattr(sample, target_attr)
                values.append(actual)
                if target is not None:
                    values.append(float(target))
            if values:
                y_min = min(min(values), 0.0)
                y_max = max(max(values), 0.0)
                span = y_max - y_min
                padding = max(span * 0.12, 0.05 if title == "力矩" else 0.1)
                y_min -= padding
                y_max += padding
            else:
                y_min, y_max = -1.0, 1.0

            def map_x(stamp: datetime) -> float:
                return plot.left() + plot.width() * (stamp.timestamp() - start) / self.window_seconds

            def map_y(value: float) -> float:
                clipped = max(y_min, min(y_max, value))
                return plot.bottom() - plot.height() * (clipped - y_min) / (y_max - y_min)

            for grid_index in range(5):
                value = y_min + (y_max - y_min) * grid_index / 4.0
                y = map_y(value)
                painter.setPen(QPen(QColor("#edf0f2"), 1))
                painter.drawLine(int(plot.left()), int(y), int(plot.right()), int(y))
                painter.setPen(QColor("#495057"))
                painter.drawText(3, int(y + 4), self._format_axis(value))

            if y_min <= 0.0 <= y_max:
                zero_y = map_y(0.0)
                painter.setPen(QPen(QColor("#ced4da"), 1, Qt.DashLine))
                painter.drawLine(int(plot.left()), int(zero_y), int(plot.right()), int(zero_y))

            painter.setPen(QColor("#212529"))
            painter.drawText(int(plot.left()), int(plot.top() - 7), f"{title} ({unit})")

            if visible:
                actual_path = QPainterPath()
                for index, sample in enumerate(visible):
                    point = (map_x(sample.timestamp), map_y(float(getattr(sample, actual_attr))))
                    if index == 0:
                        actual_path.moveTo(*point)
                    else:
                        actual_path.lineTo(*point)
                painter.setPen(QPen(QColor("#0d6efd"), 2))
                painter.drawPath(actual_path)

                target_path = QPainterPath()
                target_started = False
                for sample in visible:
                    target = getattr(sample, target_attr)
                    if target is None:
                        target_started = False
                        continue
                    point = (map_x(sample.timestamp), map_y(float(target)))
                    if not target_started:
                        target_path.moveTo(*point)
                        target_started = True
                    else:
                        target_path.lineTo(*point)
                painter.setPen(QPen(QColor("#dc6b19"), 2, Qt.DashLine))
                painter.drawPath(target_path)
            else:
                painter.setPen(QColor("#6c757d"))
                painter.drawText(plot, Qt.AlignCenter, "等待 DM 运动反馈…")

        painter.setPen(QPen(QColor("#dc6b19"), 2, Qt.DashLine))
        painter.drawLine(int(left), 14, int(left + 25), 14)
        painter.setPen(QColor("#212529"))
        painter.drawText(int(left + 31), 19, "目标")
        painter.setPen(QPen(QColor("#0d6efd"), 2))
        painter.drawLine(int(left + 92), 14, int(left + 117), 14)
        painter.setPen(QColor("#212529"))
        painter.drawText(int(left + 123), 19, "实际反馈")
        painter.drawText(int(right - 94), 19, f"最近 {int(self.window_seconds)} 秒")


class MainWindow(QMainWindow):
    GB_CANOPEN = "gb_canopen"
    DM_CLASSIC = "dm_classic"
    DM_SERIAL = "dm_serial"
    DM_VIRTUAL = "dm_virtual"

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("GB5753H 样机 · DM 兼容 CAN 电机测试工具 0.6")
        self.resize(1280, 820)
        self.session = CanSession()
        self.session.frame_received.connect(self._on_frame)
        self.session.error.connect(self._on_bus_error)
        self.session.connection_changed.connect(self._on_connection_changed)
        self.connected = False
        self.dm_enabled = False
        self.temperature_samples: list[tuple[datetime, float, float]] = []
        self.temperature_extrema: dict[str, float | None] = {
            "drive_min": None,
            "drive_max": None,
            "motor_min": None,
            "motor_max": None,
        }
        self.motion_samples: list[MotionSample] = []
        self.last_motion_target: MotionTarget | None = None
        self.motion_extrema: dict[str, float | None] = {
            "position_min": None,
            "position_max": None,
            "velocity_min": None,
            "velocity_max": None,
            "torque_min": None,
            "torque_max": None,
        }
        self.object_entries: list[ObjectEntry] = []
        self._object_lookup: dict[tuple[int, int], ObjectEntry] = {}

        self.periodic_timer = QTimer(self)
        self.periodic_timer.timeout.connect(self._send_current_control)

        self._build_ui()
        self._load_dictionary()
        self._protocol_changed()
        self._update_actions()

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.addWidget(self._build_connection_bar())

        splitter = QSplitter(Qt.Vertical)
        upper = QSplitter(Qt.Horizontal)
        upper.addWidget(self._build_protocol_panel())
        upper.addWidget(self._build_status_panel())
        upper.setSizes([850, 390])
        splitter.addWidget(upper)
        splitter.addWidget(self._build_monitor_panel())
        splitter.setSizes([480, 300])
        layout.addWidget(splitter, 1)
        self.setCentralWidget(root)

        self.statusBar().showMessage("当前样机：DM 标准控制帧，默认 ID=1，经典 CAN 1 Mbps")

    def _build_connection_bar(self) -> QWidget:
        group = QGroupBox("总线连接")
        layout = QHBoxLayout(group)
        self.protocol_combo = QComboBox()
        self.protocol_combo.addItem("GB5753H 样机 · DM USB2CAN 串口桥（推荐）", self.DM_SERIAL)
        self.protocol_combo.addItem("离线模拟 · GB5753H / DM 协议", self.DM_VIRTUAL)
        self.protocol_combo.addItem("达妙 · 经典 SocketCAN（1M）", self.DM_CLASSIC)
        self.protocol_combo.addItem("其他版本参考 · CANopen FD（不适用本样机）", self.GB_CANOPEN)
        self.protocol_combo.currentIndexChanged.connect(self._protocol_changed)
        self.channel_edit = QLineEdit("can0")
        self.channel_edit.setMaximumWidth(100)
        self.bus_hint = QLabel()
        self.bus_hint.setStyleSheet("color: #555;")
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self._toggle_connection)
        self.connection_label = QLabel("● 未连接")
        self.connection_label.setStyleSheet("color: #a22; font-weight: bold;")
        layout.addWidget(QLabel("协议"))
        layout.addWidget(self.protocol_combo, 1)
        layout.addWidget(QLabel("接口"))
        layout.addWidget(self.channel_edit)
        layout.addWidget(self.bus_hint)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.connection_label)
        return group

    def _build_protocol_panel(self) -> QWidget:
        self.protocol_tabs = QTabWidget()
        self.gb_tab = self._build_gb_tab()
        self.dm_tab = self._build_dm_tab()
        self.temperature_tab = self._build_temperature_tab()
        self.motion_tab = self._build_motion_tab()
        self.protocol_tabs.addTab(self.gb_tab, "其他版本 CANopen FD")
        self.protocol_tabs.addTab(self.dm_tab, "GB5753H 样机 / DM 协议")
        self.protocol_tabs.addTab(self.temperature_tab, "温度监测")
        self.protocol_tabs.addTab(self.motion_tab, "运动监测")
        return self.protocol_tabs

    def _build_gb_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        warning = QLabel(
            "厂家已确认：当前 GB5753H 样机不适用华翼 CANopen 通信手册和对象字典。"
            "本页只为其他固件版本保留，只读功能也不得作为当前样机的控制依据。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("background:#fff3cd; color:#664d03; padding:8px; border:1px solid #ffecb5;")
        layout.addWidget(warning)

        read_group = QGroupBox("SDO 只读查询")
        read_layout = QHBoxLayout(read_group)
        self.node_id_spin = QSpinBox()
        self.node_id_spin.setRange(1, 127)
        self.node_id_spin.setValue(1)
        self.sdo_index_spin = _hex_spin(0xFFFF, 0x1000)
        self.sdo_sub_spin = _hex_spin(0xFF, 0)
        self.sdo_read_button = QPushButton("读取对象")
        self.sdo_read_button.clicked.connect(self._read_sdo)
        self.sdo_result = QLabel("—")
        self.sdo_result.setTextInteractionFlags(Qt.TextSelectableByMouse)
        read_layout.addWidget(QLabel("Node ID"))
        read_layout.addWidget(self.node_id_spin)
        read_layout.addWidget(QLabel("Index"))
        read_layout.addWidget(self.sdo_index_spin)
        read_layout.addWidget(QLabel("Sub"))
        read_layout.addWidget(self.sdo_sub_spin)
        read_layout.addWidget(self.sdo_read_button)
        read_layout.addWidget(self.sdo_result, 1)
        layout.addWidget(read_group)

        search_row = QHBoxLayout()
        self.object_search = QLineEdit()
        self.object_search.setPlaceholderText("搜索索引、名称或描述，例如 6040、温度、MIT")
        self.object_search.textChanged.connect(self._filter_dictionary)
        search_row.addWidget(QLabel("对象字典"))
        search_row.addWidget(self.object_search, 1)
        layout.addLayout(search_row)

        self.object_table = QTableWidget(0, 7)
        self.object_table.setHorizontalHeaderLabels(
            ["Index", "Sub", "名称", "访问", "类型", "默认值", "说明"]
        )
        self.object_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.object_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.object_table.doubleClicked.connect(self._dictionary_row_selected)
        header = self.object_table.horizontalHeader()
        for column in (0, 1, 3, 4, 5):
            header.setSectionResizeMode(column, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.Interactive)
        self.object_table.setColumnWidth(2, 270)
        header.setSectionResizeMode(6, QHeaderView.Stretch)
        layout.addWidget(self.object_table, 1)
        return page

    def _build_dm_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        sample_notice = QLabel(
            "当前样机已由厂家确认兼容 DM 标准控制帧：11 位标准帧、经典 CAN 1 Mbps、默认 CAN ID=1。"
            "首次运动前仍必须向厂家确认 PMAX / VMAX / TMAX。"
        )
        sample_notice.setWordWrap(True)
        sample_notice.setStyleSheet("background:#d1e7dd; color:#0f5132; padding:8px; border:1px solid #badbcc;")
        layout.addWidget(sample_notice)

        config = QGroupBox("协议参数（必须与驱动器设置一致）")
        grid = QGridLayout(config)
        self.dm_motor_id = _hex_spin(0x5FF, 1)
        self.dm_feedback_id = _hex_spin(0x7FF, 0)
        self.dm_mode = QComboBox()
        for mode in ControlMode:
            self.dm_mode.addItem(mode.label, mode)
        self.p_max = _float_spin(0.001, 10000, 12.5)
        self.v_max = _float_spin(0.001, 10000, 30.0)
        self.t_max = _float_spin(0.001, 10000, 10.0)
        for widget in (self.p_max, self.v_max, self.t_max):
            widget.valueChanged.connect(self._limits_changed)
        grid.addWidget(QLabel("CAN ID"), 0, 0)
        grid.addWidget(self.dm_motor_id, 0, 1)
        grid.addWidget(QLabel("Master ID / 反馈 ID"), 0, 2)
        grid.addWidget(self.dm_feedback_id, 0, 3)
        grid.addWidget(QLabel("运动帧类型"), 0, 4)
        grid.addWidget(self.dm_mode, 0, 5)
        grid.addWidget(QLabel("PMAX (rad)"), 1, 0)
        grid.addWidget(self.p_max, 1, 1)
        grid.addWidget(QLabel("VMAX (rad/s)"), 1, 2)
        grid.addWidget(self.v_max, 1, 3)
        grid.addWidget(QLabel("TMAX (Nm)"), 1, 4)
        grid.addWidget(self.t_max, 1, 5)
        layout.addWidget(config)

        self.safety_check = QCheckBox("我已确认协议量程正确、电机已架空、急停可用，并愿意解锁运动指令")
        self.safety_check.stateChanged.connect(self._update_actions)
        self.safety_check.setStyleSheet("font-weight:bold; color:#8a3b00;")
        layout.addWidget(self.safety_check)

        self.mode_lock_notice = QLabel(
            "⚠ 电机已使能：当前运动帧类型已锁定。切换 MIT / 位置-速度 / 速度前，必须先点击“失能 / 急停”。"
        )
        self.mode_lock_notice.setWordWrap(True)
        self.mode_lock_notice.setStyleSheet(
            "background:#f8d7da; color:#842029; padding:8px; border:1px solid #f5c2c7; font-weight:bold;"
        )
        self.mode_lock_notice.hide()
        layout.addWidget(self.mode_lock_notice)

        action_row = QHBoxLayout()
        self.enable_button = QPushButton("使能")
        self.disable_button = QPushButton("失能 / 急停")
        self.disable_button.setStyleSheet("background:#b42318; color:white; font-weight:bold; padding:6px;")
        self.clear_button = QPushButton("清除错误")
        self.zero_button = QPushButton("设置当前位置为零点")
        self.enable_button.clicked.connect(lambda: self._special("enable"))
        self.disable_button.clicked.connect(lambda: self._special("disable"))
        self.clear_button.clicked.connect(lambda: self._special("clear_error"))
        self.zero_button.clicked.connect(lambda: self._special("set_zero"))
        for widget in (self.enable_button, self.disable_button, self.clear_button, self.zero_button):
            action_row.addWidget(widget)
        action_row.addStretch()
        layout.addLayout(action_row)

        self.control_tabs = QTabWidget()
        self.control_tabs.addTab(self._build_mit_controls(), "MIT")
        self.control_tabs.addTab(self._build_pv_controls(), "位置-速度")
        self.control_tabs.addTab(self._build_velocity_controls(), "速度")
        self.control_tabs.currentChanged.connect(self._control_tab_changed)
        self.dm_mode.currentIndexChanged.connect(self.control_tabs.setCurrentIndex)
        layout.addWidget(self.control_tabs)

        send_row = QHBoxLayout()
        self.send_once_button = QPushButton("发送一次")
        self.send_once_button.clicked.connect(self._send_current_control)
        self.periodic_check = QCheckBox("周期发送")
        self.periodic_check.stateChanged.connect(self._periodic_changed)
        self.period_spin = QSpinBox()
        self.period_spin.setRange(5, 1000)
        self.period_spin.setValue(20)
        self.period_spin.setSuffix(" ms")
        self.period_spin.valueChanged.connect(self._periodic_changed)
        send_row.addWidget(self.send_once_button)
        send_row.addWidget(self.periodic_check)
        send_row.addWidget(self.period_spin)
        send_row.addStretch()
        layout.addLayout(send_row)
        layout.addStretch()
        return page

    def _build_mit_controls(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        self.mit_position = _float_spin(-12.5, 12.5, 0)
        self.mit_velocity = _float_spin(-30, 30, 0)
        self.mit_kp = _float_spin(0, 500, 0)
        self.mit_kd = _float_spin(0, 5, 0)
        self.mit_torque = _float_spin(-10, 10, 0)
        form.addRow("目标位置 p_des (rad)", self.mit_position)
        form.addRow("目标速度 v_des (rad/s)", self.mit_velocity)
        form.addRow("Kp", self.mit_kp)
        form.addRow("Kd", self.mit_kd)
        form.addRow("前馈扭矩 t_ff (Nm)", self.mit_torque)
        return page

    def _build_pv_controls(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        self.pv_position = _float_spin(-100000, 100000, 0)
        self.pv_velocity = _float_spin(-10000, 10000, 0)
        form.addRow("目标位置 (rad, Float32 LE)", self.pv_position)
        form.addRow("梯形轨迹最高速度 (rad/s)", self.pv_velocity)
        return page

    def _build_velocity_controls(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        self.velocity_value = _float_spin(-10000, 10000, 0)
        form.addRow("目标速度 (rad/s, Float32 LE)", self.velocity_value)
        return page

    def _build_temperature_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        notice = QLabel(
            "温度曲线随 DM 反馈更新：单次发送通常增加一个采样点，周期控制时形成连续曲线。"
            "阈值仅用于上位机预警，不替代驱动器自身保护或厂家规定。"
        )
        notice.setWordWrap(True)
        notice.setStyleSheet("background:#cff4fc; color:#055160; padding:8px; border:1px solid #b6effb;")
        layout.addWidget(notice)

        summary = QGroupBox("温度概览")
        summary_layout = QGridLayout(summary)
        self.temperature_labels: dict[str, QLabel] = {}
        fields = (
            ("驱动当前", 0, 0),
            ("驱动最低/最高", 0, 2),
            ("电机当前", 1, 0),
            ("电机最低/最高", 1, 2),
        )
        for name, row, column in fields:
            value = QLabel("—")
            value.setStyleSheet("font-weight:bold;")
            self.temperature_labels[name] = value
            summary_layout.addWidget(QLabel(name), row, column)
            summary_layout.addWidget(value, row, column + 1)

        self.temperature_warning_spin = QSpinBox()
        self.temperature_warning_spin.setRange(20, 180)
        self.temperature_warning_spin.setValue(70)
        self.temperature_warning_spin.setSuffix(" ℃")
        self.temperature_alarm_spin = QSpinBox()
        self.temperature_alarm_spin.setRange(20, 200)
        self.temperature_alarm_spin.setValue(85)
        self.temperature_alarm_spin.setSuffix(" ℃")
        self.temperature_warning_spin.valueChanged.connect(self._temperature_thresholds_changed)
        self.temperature_alarm_spin.valueChanged.connect(self._temperature_thresholds_changed)
        summary_layout.addWidget(QLabel("预警阈值"), 2, 0)
        summary_layout.addWidget(self.temperature_warning_spin, 2, 1)
        summary_layout.addWidget(QLabel("报警阈值"), 2, 2)
        summary_layout.addWidget(self.temperature_alarm_spin, 2, 3)

        self.temperature_alarm_label = QLabel("等待温度反馈")
        self.temperature_alarm_label.setAlignment(Qt.AlignCenter)
        self.temperature_alarm_label.setStyleSheet(
            "background:#e9ecef; color:#495057; padding:7px; border-radius:3px; font-weight:bold;"
        )
        summary_layout.addWidget(self.temperature_alarm_label, 3, 0, 1, 4)
        layout.addWidget(summary)

        self.temperature_chart = TemperatureChart()
        self.temperature_chart.set_samples(self.temperature_samples)
        layout.addWidget(self.temperature_chart, 1)

        actions = QHBoxLayout()
        clear_button = QPushButton("清空温度记录")
        clear_button.clicked.connect(self._clear_temperature_history)
        export_button = QPushButton("导出温度 CSV")
        export_button.clicked.connect(self._export_temperature_csv)
        self.temperature_sample_count = QLabel("0 个采样点（最多保留 10000 个）")
        actions.addWidget(clear_button)
        actions.addWidget(export_button)
        actions.addWidget(self.temperature_sample_count)
        actions.addStretch()
        layout.addLayout(actions)
        return page

    def _build_motion_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        notice = QLabel(
            "曲线随 DM 反馈帧更新，并使用最后一次实际发送成功的控制目标。"
            "MIT 模式的目标力矩对应前馈力矩 t_ff；"
            "位置-速度模式中的目标速度是梯形轨迹最高速度；速度模式没有位置目标，"
            "位置-速度和速度模式没有前馈力矩目标，因此对应目标显示为—。"
        )
        notice.setWordWrap(True)
        notice.setStyleSheet("background:#cff4fc; color:#055160; padding:8px; border:1px solid #b6effb;")
        layout.addWidget(notice)

        summary = QGroupBox("运动概览")
        grid = QGridLayout(summary)
        for column, title in enumerate(("参数", "目标", "实际反馈", "误差（目标-实际）", "实际最低/最高")):
            header = QLabel(title)
            header.setStyleSheet("font-weight:bold;")
            grid.addWidget(header, 0, column)

        self.motion_labels: dict[str, QLabel] = {}
        metrics = (
            ("position", "位置", "rad"),
            ("velocity", "速度", "rad/s"),
            ("torque", "力矩", "Nm"),
        )
        for row, (key, title, unit) in enumerate(metrics, start=1):
            grid.addWidget(QLabel(f"{title} ({unit})"), row, 0)
            for column, suffix in enumerate(("target", "actual", "error", "range"), start=1):
                label = QLabel("—")
                label.setTextInteractionFlags(Qt.TextSelectableByMouse)
                if suffix in {"target", "actual"}:
                    label.setStyleSheet("font-weight:bold;")
                self.motion_labels[f"{key}_{suffix}"] = label
                grid.addWidget(label, row, column)

        self.motion_mode_label = QLabel("尚未发送运动指令")
        self.motion_mode_label.setStyleSheet("font-weight:bold; color:#495057;")
        grid.addWidget(QLabel("最后目标模式"), 4, 0)
        grid.addWidget(self.motion_mode_label, 4, 1, 1, 4)
        layout.addWidget(summary)

        self.motion_chart = MotionChart()
        self.motion_chart.set_samples(self.motion_samples)
        layout.addWidget(self.motion_chart, 1)

        actions = QHBoxLayout()
        clear_button = QPushButton("清空运动记录")
        clear_button.clicked.connect(self._clear_motion_history)
        export_button = QPushButton("导出运动 CSV")
        export_button.clicked.connect(self._export_motion_csv)
        self.motion_sample_count = QLabel("0 个反馈采样点（最多保留 10000 个）")
        actions.addWidget(clear_button)
        actions.addWidget(export_button)
        actions.addWidget(self.motion_sample_count)
        actions.addStretch()
        layout.addLayout(actions)
        return page

    def _build_status_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        status_group = QGroupBox("实时状态")
        form = QFormLayout(status_group)
        names = ["来源", "节点/电机", "状态", "位置", "速度", "力矩", "驱动温度", "电机温度", "最后反馈"]
        self.status_values: dict[str, QLabel] = {}
        for name in names:
            label = QLabel("—")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.status_values[name] = label
            form.addRow(name, label)
        layout.addWidget(status_group)

        help_group = QGroupBox("接口配置提示")
        help_layout = QVBoxLayout(help_group)
        self.setup_command = QLabel()
        self.setup_command.setWordWrap(True)
        self.setup_command.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.setup_command.setFont(QFont("Monospace"))
        help_layout.addWidget(self.setup_command)
        layout.addWidget(help_group)
        layout.addStretch()
        return panel

    def _build_monitor_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        raw_row = QHBoxLayout()
        self.raw_id = _hex_spin(0x7FF, 0x123)
        self.raw_data = QLineEdit("00 00 00 00 00 00 00 00")
        self.raw_fd = QCheckBox("FD")
        self.raw_brs = QCheckBox("BRS")
        self.raw_send_button = QPushButton("发送原始帧")
        self.raw_send_button.clicked.connect(self._send_raw)
        raw_row.addWidget(QLabel("原始发送 ID"))
        raw_row.addWidget(self.raw_id)
        raw_row.addWidget(QLabel("Data"))
        raw_row.addWidget(self.raw_data, 1)
        raw_row.addWidget(self.raw_fd)
        raw_row.addWidget(self.raw_brs)
        raw_row.addWidget(self.raw_send_button)
        layout.addLayout(raw_row)

        self.log_table = QTableWidget(0, 7)
        self.log_table.setHorizontalHeaderLabels(["时间", "方向", "类型", "ID", "DLC", "Data", "解析"])
        self.log_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.log_table.setSelectionBehavior(QTableWidget.SelectRows)
        header = self.log_table.horizontalHeader()
        for column in (0, 1, 2, 3, 4):
            header.setSectionResizeMode(column, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.Stretch)
        header.setSectionResizeMode(6, QHeaderView.Stretch)
        layout.addWidget(self.log_table, 1)
        clear = QPushButton("清空报文")
        clear.clicked.connect(lambda: self.log_table.setRowCount(0))
        layout.addWidget(clear, alignment=Qt.AlignRight)
        return panel

    def _load_dictionary(self) -> None:
        try:
            self.object_entries = load_xlsx(OBJECT_DICTIONARY)
            self._object_lookup = {(entry.index, entry.sub_index): entry for entry in self.object_entries}
            self._filter_dictionary()
            self.statusBar().showMessage(f"已载入华翼对象字典：{len(self.object_entries)} 项")
        except Exception as exc:
            self.statusBar().showMessage(f"对象字典加载失败：{exc}")

    def _filter_dictionary(self) -> None:
        query = self.object_search.text().strip().lower() if hasattr(self, "object_search") else ""
        entries = []
        for entry in self.object_entries:
            haystack = f"{entry.index:04x} {entry.sub_index:02x} {entry.name} {entry.description}".lower()
            if query in haystack:
                entries.append(entry)
        self.object_table.setRowCount(len(entries))
        for row, entry in enumerate(entries):
            values = (
                f"0x{entry.index:04X}",
                f"0x{entry.sub_index:02X}",
                entry.name,
                entry.access,
                entry.data_type,
                entry.default_value,
                entry.description.replace("\n", " "),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setData(Qt.UserRole, (entry.index, entry.sub_index))
                self.object_table.setItem(row, column, item)

    def _dictionary_row_selected(self) -> None:
        row = self.object_table.currentRow()
        item = self.object_table.item(row, 0)
        if item is None:
            return
        index, sub_index = item.data(Qt.UserRole)
        self.sdo_index_spin.setValue(index)
        self.sdo_sub_spin.setValue(sub_index)

    def _protocol(self) -> str:
        return self.protocol_combo.currentData()

    def _protocol_changed(self) -> None:
        protocol = self._protocol()
        is_gb = protocol == self.GB_CANOPEN
        is_serial = protocol == self.DM_SERIAL
        is_virtual = protocol == self.DM_VIRTUAL
        if is_serial and self.channel_edit.text().strip() == "can0":
            self.channel_edit.setText("/dev/ttyACM0")
        elif not is_serial and self.channel_edit.text().strip().startswith("/dev/tty"):
            self.channel_edit.setText("can0")
        self.protocol_tabs.setCurrentWidget(self.gb_tab if is_gb else self.dm_tab)
        self.channel_edit.setEnabled(not is_virtual and not self.connected)
        self.bus_hint.setText(
            "FD+BRS" if is_gb else ("921600 / CAN 1M" if is_serial else ("无需硬件" if is_virtual else "经典帧 1 Mbps"))
        )
        self.raw_fd.setChecked(is_gb)
        self.raw_brs.setChecked(is_gb)
        channel = self.channel_edit.text().strip() or "can0"
        if is_gb:
            command = (
                f"sudo ip link set {channel} down\n"
                f"sudo ip link set {channel} type can bitrate 1000000 dbitrate 5000000 fd on\n"
                f"sudo ip link set {channel} up"
            )
        elif is_serial:
            command = (
                f"串口桥：{channel}\n"
                "主机串口：921600 baud\n"
                "CAN 总线：经典 CAN 1 Mbps（由适配器固件设置）"
            )
        elif is_virtual:
            command = "离线模拟不访问 CAN 硬件，可安全验证报文打包和界面流程。"
        else:
            command = (
                f"sudo ip link set {channel} down\n"
                f"sudo ip link set {channel} type can bitrate 1000000\n"
                f"sudo ip link set {channel} up"
            )
        self.setup_command.setText(command)
        self._update_actions()

    def _toggle_connection(self) -> None:
        if self.connected:
            self._safe_disconnect()
            return
        try:
            protocol = self._protocol()
            if protocol == self.GB_CANOPEN:
                transport: Transport = SocketCanTransport(self.channel_edit.text(), allow_fd=True)
            elif protocol == self.DM_SERIAL:
                transport = DmSerialTransport(self.channel_edit.text())
            elif protocol == self.DM_CLASSIC:
                transport = SocketCanTransport(self.channel_edit.text(), allow_fd=False)
            else:
                transport = VirtualTransport(self.dm_motor_id.value(), self.dm_feedback_id.value(), self._limits())
            self.session.connect_transport(transport)
        except Exception as exc:
            QMessageBox.critical(self, "连接失败", str(exc))

    def _safe_disconnect(self) -> None:
        self.periodic_check.setChecked(False)
        if self.dm_enabled and self._protocol() != self.GB_CANOPEN:
            try:
                frame = pack_special(self.dm_motor_id.value(), "disable")
                self._send_frame(frame, "关闭连接前失能")
            except Exception:
                pass
        self.dm_enabled = False
        self.session.disconnect_transport()

    def _on_connection_changed(self, connected: bool) -> None:
        self.connected = connected
        self.last_motion_target = None
        self.connection_label.setText("● 已连接" if connected else "● 未连接")
        self.connection_label.setStyleSheet(
            "color:#157347; font-weight:bold;" if connected else "color:#a22; font-weight:bold;"
        )
        self.connect_button.setText("断开" if connected else "连接")
        self.protocol_combo.setEnabled(not connected)
        self.channel_edit.setEnabled(not connected and self._protocol() != self.DM_VIRTUAL)
        self.statusBar().showMessage("CAN 接口已连接" if connected else "CAN 接口已断开")
        self._update_actions()

    def _on_bus_error(self, message: str) -> None:
        self.periodic_check.setChecked(False)
        self.statusBar().showMessage(f"CAN 接收错误：{message}")
        QMessageBox.critical(self, "CAN 总线错误", message)
        self.session.disconnect_transport()

    def _limits(self) -> MotorLimits:
        return MotorLimits(self.p_max.value(), self.v_max.value(), self.t_max.value())

    def _limits_changed(self) -> None:
        limits = self._limits()
        self.mit_position.setRange(-limits.p_max, limits.p_max)
        self.mit_velocity.setRange(-limits.v_max, limits.v_max)
        self.mit_torque.setRange(-limits.t_max, limits.t_max)

    def _selected_mode(self) -> ControlMode:
        return self.dm_mode.currentData()

    def _control_tab_changed(self, index: int) -> None:
        self.dm_mode.setCurrentIndex(index)

    def _update_actions(self) -> None:
        dm_protocol = hasattr(self, "protocol_combo") and self._protocol() != self.GB_CANOPEN
        armed = dm_protocol and self.connected and self.safety_check.isChecked()
        mode_locked = dm_protocol and self.connected and self.dm_enabled
        self.enable_button.setEnabled(armed)
        self.send_once_button.setEnabled(armed)
        self.periodic_check.setEnabled(armed)
        self.clear_button.setEnabled(armed)
        self.zero_button.setEnabled(armed and not self.dm_enabled)
        self.disable_button.setEnabled(dm_protocol and self.connected)
        self.sdo_read_button.setEnabled(self.connected and self._protocol() == self.GB_CANOPEN)
        self.raw_send_button.setEnabled(self.connected)
        self.dm_mode.setEnabled(not mode_locked)
        current_tab = self.control_tabs.currentIndex()
        for index in range(self.control_tabs.count()):
            self.control_tabs.setTabEnabled(index, not mode_locked or index == current_tab)
        self.mode_lock_notice.setVisible(mode_locked)

    def _special(self, command: str) -> None:
        if command == "enable":
            answer = QMessageBox.warning(
                self,
                "确认使能",
                "即将给电机发送使能帧。请再次确认电机架空、量程正确且人员远离。\n\n"
                "使能后将锁定当前运动帧类型；切换模式前必须先失能。",
                QMessageBox.Ok | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Ok:
                return
        if command == "set_zero":
            answer = QMessageBox.warning(
                self,
                "确认保存零点",
                "此命令可能把当前位置保存到驱动器。仅应在失能状态执行。是否继续？",
                QMessageBox.Ok | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Ok:
                return
        try:
            frame = pack_special(self.dm_motor_id.value(), command)
            labels = {"enable": "使能", "disable": "失能", "set_zero": "保存零点", "clear_error": "清错"}
            previous_target = self.last_motion_target
            self.last_motion_target = None
            try:
                self._send_frame(frame, labels[command])
            except Exception:
                self.last_motion_target = previous_target
                raise
            self.motion_mode_label.setText("尚未发送运动指令")
            if command == "enable":
                self.dm_enabled = True
            elif command == "disable":
                self.dm_enabled = False
                self.periodic_check.setChecked(False)
            self._update_actions()
        except Exception as exc:
            self._show_send_error(exc)

    def _current_control_frame(self) -> CanFrame:
        motor_id = self.dm_motor_id.value()
        index = self.control_tabs.currentIndex()
        if index == 0:
            return pack_mit(
                motor_id,
                self.mit_position.value(),
                self.mit_velocity.value(),
                self.mit_kp.value(),
                self.mit_kd.value(),
                self.mit_torque.value(),
                self._limits(),
            )
        if index == 1:
            return pack_position_velocity(motor_id, self.pv_position.value(), self.pv_velocity.value())
        return pack_velocity(motor_id, self.velocity_value.value())

    def _current_motion_target(self) -> MotionTarget:
        index = self.control_tabs.currentIndex()
        if index == 0:
            return MotionTarget(
                "MIT",
                float(self.mit_position.value()),
                float(self.mit_velocity.value()),
                float(self.mit_torque.value()),
            )
        if index == 1:
            return MotionTarget(
                "位置-速度（速度为轨迹上限）",
                float(self.pv_position.value()),
                float(self.pv_velocity.value()),
                None,
            )
        return MotionTarget("速度", None, float(self.velocity_value.value()), None)

    def _send_current_control(self) -> None:
        if not self.connected or not self.safety_check.isChecked():
            self.periodic_check.setChecked(False)
            return
        try:
            frame = self._current_control_frame()
            target = self._current_motion_target()
            previous_target = self.last_motion_target
            self.last_motion_target = target
            try:
                self._send_frame(frame, "控制指令")
            except Exception:
                self.last_motion_target = previous_target
                raise
            self.motion_mode_label.setText(target.mode)
        except Exception as exc:
            self.periodic_check.setChecked(False)
            self._show_send_error(exc)

    def _periodic_changed(self) -> None:
        if self.periodic_check.isChecked():
            if not self.connected or not self.safety_check.isChecked():
                self.periodic_check.setChecked(False)
                return
            self.periodic_timer.start(self.period_spin.value())
        else:
            self.periodic_timer.stop()

    def _read_sdo(self) -> None:
        try:
            frame = sdo_upload_request(
                self.node_id_spin.value(), self.sdo_index_spin.value(), self.sdo_sub_spin.value()
            )
            self.sdo_result.setText("等待回包…")
            self._send_frame(frame, "SDO 上传请求")
        except Exception as exc:
            self._show_send_error(exc)

    def _send_raw(self) -> None:
        try:
            data = parse_hex_data(self.raw_data.text())
            frame = CanFrame(
                self.raw_id.value(),
                data,
                is_fd=self.raw_fd.isChecked(),
                bitrate_switch=self.raw_brs.isChecked(),
            )
            answer = QMessageBox.warning(
                self,
                "确认原始发送",
                "原始 CAN 帧不经过协议安全检查，错误报文可能使电机运动或修改参数。是否发送？",
                QMessageBox.Ok | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer == QMessageBox.Ok:
                self._send_frame(frame, "原始帧")
        except Exception as exc:
            self._show_send_error(exc)

    def _send_frame(self, frame: CanFrame, note: str = "") -> None:
        self.session.send(frame)
        self._append_log("TX", frame, note)

    def _on_frame(self, frame: CanFrame) -> None:
        note = ""
        if self._protocol() == self.GB_CANOPEN:
            sdo = parse_sdo_response(frame)
            if sdo is not None:
                if sdo.abort_code is not None:
                    note = f"SDO 中止 0x{sdo.abort_code:08X}: {sdo.message}"
                    self.sdo_result.setText(note)
                else:
                    entry = self._object_lookup.get((sdo.index, sdo.sub_index))
                    value = decode_value(sdo.data, entry.data_type if entry else "")
                    name = entry.name if entry else "未知对象"
                    note = f"SDO 0x{sdo.index:04X}:{sdo.sub_index:02X} {name} = {value}"
                    self.sdo_result.setText(str(value))
            snapshot = parse_default_tpdo(frame, self.node_id_spin.value())
            if snapshot is not None:
                note = f"{snapshot.source}: " + ", ".join(f"{key}={value}" for key, value in snapshot.values.items())
                self.status_values["来源"].setText(snapshot.source)
                self.status_values["节点/电机"].setText(str(self.node_id_spin.value()))
                mapping = {
                    "状态字": "状态",
                    "位置(Rev)": "位置",
                    "速度(Rev/s)": "速度",
                    "力矩(峰值‰)": "力矩",
                    "驱动温度(0.1℃)": "驱动温度",
                    "电机温度(0.1℃)": "电机温度",
                }
                for key, value in snapshot.values.items():
                    target = mapping.get(key)
                    if target:
                        if "温度" in target:
                            self.status_values[target].setText(f"{float(value) / 10:.1f} ℃")
                        elif target == "状态":
                            self.status_values[target].setText(f"0x{int(value):04X}")
                        else:
                            self.status_values[target].setText(f"{value:g}")
                self.status_values["最后反馈"].setText(datetime.now().strftime("%H:%M:%S.%f")[:-3])
            if frame.arbitration_id == 0x700 + self.node_id_spin.value() and frame.data:
                note = f"Heartbeat / NMT state 0x{frame.data[0]:02X}"
            if frame.arbitration_id == 0x80 + self.node_id_spin.value() and len(frame.data) >= 2:
                note = f"EMCY error 0x{int.from_bytes(frame.data[:2], 'little'):04X}"
        else:
            if frame.arbitration_id == self.dm_feedback_id.value() and len(frame.data) == 8:
                try:
                    feedback = parse_feedback(frame, self._limits())
                    note = feedback.state_text
                    self.status_values["来源"].setText("达妙反馈帧")
                    self.status_values["节点/电机"].setText(f"0x{feedback.motor_id:X}")
                    self.status_values["状态"].setText(feedback.state_text)
                    self.status_values["位置"].setText(f"{feedback.position:.5f} rad")
                    self.status_values["速度"].setText(f"{feedback.velocity:.5f} rad/s")
                    self.status_values["力矩"].setText(f"{feedback.torque:.4f} Nm")
                    self.status_values["驱动温度"].setText(f"{feedback.mos_temperature} ℃")
                    self.status_values["电机温度"].setText(f"{feedback.rotor_temperature} ℃")
                    self.status_values["最后反馈"].setText(datetime.now().strftime("%H:%M:%S.%f")[:-3])
                    self._record_temperature(feedback.mos_temperature, feedback.rotor_temperature)
                    self._record_motion(feedback.position, feedback.velocity, feedback.torque)
                    self.dm_enabled = feedback.state_code == 1
                    self._update_actions()
                except ProtocolError as exc:
                    note = str(exc)
        self._append_log("RX", frame, note)

    def _record_temperature(self, drive_temperature: float, motor_temperature: float) -> None:
        self.temperature_samples.append((datetime.now(), float(drive_temperature), float(motor_temperature)))
        if len(self.temperature_samples) > 10000:
            del self.temperature_samples[: len(self.temperature_samples) - 10000]

        extrema_updates = {
            "drive_min": (min, float(drive_temperature)),
            "drive_max": (max, float(drive_temperature)),
            "motor_min": (min, float(motor_temperature)),
            "motor_max": (max, float(motor_temperature)),
        }
        for key, (operation, value) in extrema_updates.items():
            current = self.temperature_extrema[key]
            self.temperature_extrema[key] = value if current is None else operation(current, value)

        self.temperature_labels["驱动当前"].setText(f"{drive_temperature:.0f} ℃")
        self.temperature_labels["电机当前"].setText(f"{motor_temperature:.0f} ℃")
        self.temperature_labels["驱动最低/最高"].setText(
            f"{self.temperature_extrema['drive_min']:.0f} / {self.temperature_extrema['drive_max']:.0f} ℃"
        )
        self.temperature_labels["电机最低/最高"].setText(
            f"{self.temperature_extrema['motor_min']:.0f} / {self.temperature_extrema['motor_max']:.0f} ℃"
        )
        self.temperature_sample_count.setText(f"{len(self.temperature_samples)} 个采样点（最多保留 10000 个）")
        self.temperature_chart.update()
        self._update_temperature_alarm(drive_temperature, motor_temperature)

    def _record_motion(self, position: float, velocity: float, torque: float) -> None:
        target = self.last_motion_target
        sample = MotionSample(
            timestamp=datetime.now(),
            mode=target.mode if target is not None else "无运动目标",
            target_position=target.position if target is not None else None,
            actual_position=float(position),
            target_velocity=target.velocity if target is not None else None,
            actual_velocity=float(velocity),
            target_torque=target.torque if target is not None else None,
            actual_torque=float(torque),
        )
        self.motion_samples.append(sample)
        if len(self.motion_samples) > 10000:
            del self.motion_samples[: len(self.motion_samples) - 10000]

        actual_values = {
            "position": sample.actual_position,
            "velocity": sample.actual_velocity,
            "torque": sample.actual_torque,
        }
        for key, value in actual_values.items():
            minimum_key = f"{key}_min"
            maximum_key = f"{key}_max"
            current_min = self.motion_extrema[minimum_key]
            current_max = self.motion_extrema[maximum_key]
            self.motion_extrema[minimum_key] = value if current_min is None else min(current_min, value)
            self.motion_extrema[maximum_key] = value if current_max is None else max(current_max, value)

        units = {"position": "rad", "velocity": "rad/s", "torque": "Nm"}
        targets = {
            "position": sample.target_position,
            "velocity": sample.target_velocity,
            "torque": sample.target_torque,
        }
        for key, actual in actual_values.items():
            unit = units[key]
            target_value = targets[key]
            self.motion_labels[f"{key}_target"].setText(
                "—" if target_value is None else f"{target_value:.5f} {unit}"
            )
            self.motion_labels[f"{key}_actual"].setText(f"{actual:.5f} {unit}")
            self.motion_labels[f"{key}_error"].setText(
                "—" if target_value is None else f"{target_value - actual:+.5f} {unit}"
            )
            self.motion_labels[f"{key}_range"].setText(
                f"{self.motion_extrema[f'{key}_min']:.5f} / "
                f"{self.motion_extrema[f'{key}_max']:.5f} {unit}"
            )

        self.motion_mode_label.setText(sample.mode)
        self.motion_sample_count.setText(
            f"{len(self.motion_samples)} 个反馈采样点（最多保留 10000 个）"
        )
        self.motion_chart.update()

    def _temperature_thresholds_changed(self, _value: int = 0) -> None:
        warning = float(self.temperature_warning_spin.value())
        alarm = float(self.temperature_alarm_spin.value())
        self.temperature_chart.set_thresholds(warning, alarm)
        if self.temperature_samples:
            _stamp, drive, motor = self.temperature_samples[-1]
            self._update_temperature_alarm(drive, motor)
        elif warning >= alarm:
            self._set_temperature_alarm_text("阈值设置无效：预警阈值必须低于报警阈值", "alarm")
        else:
            self._set_temperature_alarm_text("等待温度反馈", "waiting")

    def _update_temperature_alarm(self, drive: float, motor: float) -> None:
        warning = float(self.temperature_warning_spin.value())
        alarm = float(self.temperature_alarm_spin.value())
        if warning >= alarm:
            self._set_temperature_alarm_text("阈值设置无效：预警阈值必须低于报警阈值", "alarm")
            return
        hottest = max(drive, motor)
        hottest_name = "驱动" if drive >= motor else "电机"
        if hottest >= alarm:
            self._set_temperature_alarm_text(f"高温报警：{hottest_name} {hottest:.0f} ℃", "alarm")
        elif hottest >= warning:
            self._set_temperature_alarm_text(f"温度预警：{hottest_name} {hottest:.0f} ℃", "warning")
        else:
            self._set_temperature_alarm_text(
                f"温度正常：驱动 {drive:.0f} ℃ / 电机 {motor:.0f} ℃", "normal"
            )

    def _set_temperature_alarm_text(self, text: str, level: str) -> None:
        styles = {
            "waiting": "background:#e9ecef; color:#495057;",
            "normal": "background:#d1e7dd; color:#0f5132;",
            "warning": "background:#fff3cd; color:#664d03;",
            "alarm": "background:#f8d7da; color:#842029;",
        }
        self.temperature_alarm_label.setText(text)
        self.temperature_alarm_label.setStyleSheet(
            f"{styles[level]} padding:7px; border-radius:3px; font-weight:bold;"
        )

    def _clear_temperature_history(self) -> None:
        self.temperature_samples.clear()
        for key in self.temperature_extrema:
            self.temperature_extrema[key] = None
        for label in self.temperature_labels.values():
            label.setText("—")
        self.temperature_sample_count.setText("0 个采样点（最多保留 10000 个）")
        self._temperature_thresholds_changed()
        self.temperature_chart.update()

    def _export_temperature_csv(self) -> None:
        if not self.temperature_samples:
            QMessageBox.information(self, "没有温度数据", "收到电机反馈后才能导出温度记录。")
            return
        suggested = f"GB5753H_temperature_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _selected_filter = QFileDialog.getSaveFileName(
            self, "导出温度记录", str(ROOT / suggested), "CSV 文件 (*.csv)"
        )
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                writer = csv.writer(stream)
                writer.writerow(("timestamp", "drive_temperature_c", "motor_temperature_c"))
                for stamp, drive, motor in self.temperature_samples:
                    writer.writerow((stamp.isoformat(timespec="milliseconds"), f"{drive:.1f}", f"{motor:.1f}"))
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))
            return
        QMessageBox.information(self, "导出完成", f"已导出 {len(self.temperature_samples)} 个采样点：\n{path}")

    def _clear_motion_history(self) -> None:
        self.motion_samples.clear()
        for key in self.motion_extrema:
            self.motion_extrema[key] = None
        for label in self.motion_labels.values():
            label.setText("—")
        self.motion_mode_label.setText(
            self.last_motion_target.mode if self.last_motion_target is not None else "尚未发送运动指令"
        )
        self.motion_sample_count.setText("0 个反馈采样点（最多保留 10000 个）")
        self.motion_chart.update()

    def _export_motion_csv(self) -> None:
        if not self.motion_samples:
            QMessageBox.information(self, "没有运动数据", "收到电机反馈后才能导出运动记录。")
            return
        suggested = f"GB5753H_motion_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path, _selected_filter = QFileDialog.getSaveFileName(
            self, "导出运动记录", str(ROOT / suggested), "CSV 文件 (*.csv)"
        )
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"

        def csv_value(value: float | None) -> str:
            return "" if value is None else f"{value:.7f}"

        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    (
                        "timestamp",
                        "mode",
                        "target_position_rad",
                        "actual_position_rad",
                        "position_error_rad",
                        "target_velocity_rad_s",
                        "actual_velocity_rad_s",
                        "velocity_error_rad_s",
                        "target_torque_nm",
                        "actual_torque_nm",
                        "torque_error_nm",
                    )
                )
                for sample in self.motion_samples:
                    writer.writerow(
                        (
                            sample.timestamp.isoformat(timespec="milliseconds"),
                            sample.mode,
                            csv_value(sample.target_position),
                            csv_value(sample.actual_position),
                            csv_value(
                                None
                                if sample.target_position is None
                                else sample.target_position - sample.actual_position
                            ),
                            csv_value(sample.target_velocity),
                            csv_value(sample.actual_velocity),
                            csv_value(
                                None
                                if sample.target_velocity is None
                                else sample.target_velocity - sample.actual_velocity
                            ),
                            csv_value(sample.target_torque),
                            csv_value(sample.actual_torque),
                            csv_value(
                                None
                                if sample.target_torque is None
                                else sample.target_torque - sample.actual_torque
                            ),
                        )
                    )
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))
            return
        QMessageBox.information(self, "导出完成", f"已导出 {len(self.motion_samples)} 个采样点：\n{path}")

    def _append_log(self, direction: str, frame: CanFrame, note: str) -> None:
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        frame_type = "FD+BRS" if frame.is_fd and frame.bitrate_switch else ("FD" if frame.is_fd else "CAN")
        values = (
            datetime.now().strftime("%H:%M:%S.%f")[:-3],
            direction,
            frame_type,
            f"0x{frame.arbitration_id:03X}",
            str(len(frame.data)),
            frame.data.hex(" ").upper(),
            note,
        )
        color = QColor("#0b6bcb") if direction == "RX" else QColor("#7a4e00")
        for column, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setForeground(color)
            self.log_table.setItem(row, column, item)
        if self.log_table.rowCount() > 1000:
            self.log_table.removeRow(0)
        self.log_table.scrollToBottom()

    def _show_send_error(self, exc: Exception) -> None:
        self.statusBar().showMessage(f"发送失败：{exc}")
        QMessageBox.critical(self, "发送失败", str(exc))

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._safe_disconnect()
        event.accept()


def main() -> int:
    app = QApplication([])
    app.setApplicationName("GB5753 CAN Test Tool")
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return app.exec_()
