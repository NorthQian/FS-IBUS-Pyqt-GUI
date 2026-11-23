#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FlySky iBUS 接收机监视器（PyQt5）。

协议帧：32 字节，0x20 0x40 开头，14 / 18 通道数据，
校验 = (Byte1 + ... + Byte30) ^ 0xFFFF，低字节在前。
"""

import colorsys
import math
import os
import sys
import time

# VSCode(snap 版) 的集成终端会注入指向 snap 内 GTK 的变量，
# 系统 Qt 加载这些库后会与系统 glibc 冲突：
#   symbol lookup error: /snap/core20/.../libpthread.so.0: __libc_pthread_init
# 只清理指向 /snap/ 的变量，普通终端不受影响。
for _key in (
    "GTK_PATH",
    "GTK_EXE_PREFIX",
    "GTK_IM_MODULE_FILE",
    "GDK_PIXBUF_MODULE_FILE",
    "GDK_PIXBUF_MODULEDIR",
    "GSETTINGS_SCHEMA_DIR",
    "GIO_MODULE_DIR",
    "LOCPATH",
):
    if "/snap/" in os.environ.get(_key, ""):
        os.environ.pop(_key, None)

import serial
from serial.tools import list_ports

from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

BAUD = 115200
HEADER = b"\x20\x40"
FRAME_LEN = 32
CHECKSUM_SUM_LEN = 30

MODE_18 = "18 通道 · 12bit 紧凑格式（本机协议）"
MODE_14 = "14 通道 · 标准 iBUS"

# 扩展通道 CH15~CH18 的 3 个 4bit 分段所在的字节下标（0 起）
EXT_NIBBLE_BYTES = ((3, 5, 7), (9, 11, 13), (15, 17, 19), (21, 23, 25))


# --------------------------------------------------------------------------
# 协议解析
# --------------------------------------------------------------------------
def checksum_of(frame):
    return (0xFFFF - (sum(frame[:CHECKSUM_SUM_LEN]) & 0xFFFF)) & 0xFFFF


def decode_channels(frame, mode):
    """把 32 字节帧解析成通道值列表。"""
    b = frame
    if mode == MODE_14:
        # 标准 iBUS：每通道 16bit 小端，1000~2000
        return [b[2 + 2 * i] | (b[3 + 2 * i] << 8) for i in range(14)]

    # 本机协议：18 通道 × 12bit
    ch = [0] * 18
    # CH1~CH12：低 8 位在前一字节，bit8~11 在后一字节的高半字节
    for n in range(1, 13):
        ch[n - 1] = b[2 * n] | ((b[2 * n + 1] >> 4) << 8)
    # CH13、CH14
    ch[12] = b[26] | ((b[27] >> 4) << 8)
    ch[13] = b[28] | ((b[29] >> 4) << 8)
    # CH15~CH18：3 个低半字节拼成 12bit
    for k, (i0, i1, i2) in enumerate(EXT_NIBBLE_BYTES):
        ch[14 + k] = (b[i0] & 0x0F) | ((b[i1] & 0x0F) << 4) | ((b[i2] & 0x0F) << 8)
    return ch


class IBusParser:
    """按 0x20 0x40 同步、校验后输出完整帧。"""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0:
                # 保留最后 1 字节，防止 0x20 被切到下一批数据
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if idx:
                del self.buf[:idx]
            if len(self.buf) < FRAME_LEN:
                break
            frame = bytes(self.buf[:FRAME_LEN])
            if checksum_of(frame) != (frame[30] | (frame[31] << 8)):
                self.errors += 1
                del self.buf[:2]          # 头部不可信，重新同步
                continue
            frames.append(frame)
            del self.buf[:FRAME_LEN]
        return frames


# --------------------------------------------------------------------------
# 后台线程
# --------------------------------------------------------------------------
class SerialWorker(QThread):
    frameReceived = pyqtSignal(list, str)
    parseErrors = pyqtSignal(int)
    failed = pyqtSignal(str)

    def __init__(self, port, mode, parent=None):
        super().__init__(parent)
        self.port = port
        self.mode = mode
        self._running = True
        self._ser = None

    def run(self):
        parser = IBusParser()
        try:
            self._ser = serial.Serial(self.port, BAUD, timeout=0.05)
        except Exception as exc:
            denied = (
                getattr(exc, "errno", None) == 13
                or "permission" in str(exc).lower()
                or "权限" in str(exc)
            )
            if denied:
                self.failed.emit(
                    f"无权限打开 {self.port}，请执行："
                    f"sudo usermod -aG dialout $USER 后重新登录"
                )
            else:
                self.failed.emit(f"打开串口失败：{exc}")
            return
        try:
            while self._running:
                chunk = self._ser.read(64)
                if not chunk:
                    continue
                for frame in parser.feed(chunk):
                    self.frameReceived.emit(
                        decode_channels(frame, self.mode),
                        frame.hex(" ").upper(),
                    )
                if parser.errors:
                    self.parseErrors.emit(parser.errors)
                    parser.errors = 0
        except Exception as exc:
            if self._running:
                self.failed.emit(f"串口读取异常：{exc}")
        finally:
            try:
                self._ser.close()
            except Exception:
                pass

    def stop(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.cancel_read()
            except Exception:
                pass
        self.wait(1500)


def pack_frame(chans, mode):
    """反向打包，供模拟数据使用（也用于自测）。"""
    if mode == MODE_14:
        b = bytearray(HEADER)
        for v in chans:
            b += bytes((v & 0xFF, (v >> 8) & 0xFF))
    else:
        b = bytearray(FRAME_LEN - 2)
        b[0], b[1] = 0x20, 0x40
        for n in range(1, 13):
            v = chans[n - 1]
            b[2 * n] = v & 0xFF
            b[2 * n + 1] = ((v >> 8) & 0x0F) << 4
        b[26] = chans[12] & 0xFF
        b[27] = ((chans[12] >> 8) & 0x0F) << 4
        b[28] = chans[13] & 0xFF
        b[29] = ((chans[13] >> 8) & 0x0F) << 4
        for k, (i0, i1, i2) in enumerate(EXT_NIBBLE_BYTES):
            v = chans[14 + k]
            b[i0] |= v & 0x0F
            b[i1] |= (v >> 4) & 0x0F
            b[i2] |= (v >> 8) & 0x0F
    chk = checksum_of(b)
    b += bytes((chk & 0xFF, (chk >> 8) & 0xFF))
    return bytes(b)


class SimWorker(QThread):
    """无硬件时生成正弦变化的模拟帧。"""

    frameReceived = pyqtSignal(list, str)

    def __init__(self, mode, parent=None):
        super().__init__(parent)
        self.mode = mode
        self._running = True

    def run(self):
        count = 18 if self.mode == MODE_18 else 14
        center, swing = (2047, 1900) if self.mode == MODE_18 else (1500, 450)
        t0 = time.monotonic()
        while self._running:
            t = time.monotonic() - t0
            chans = [
                int(round(center + swing * math.sin(2 * math.pi * 0.35 * (1 + (k % 4) * 0.3) * t + k * 0.7)))
                for k in range(count)
            ]
            frame = pack_frame(chans, self.mode)
            self.frameReceived.emit(decode_channels(frame, self.mode), frame.hex(" ").upper())
            time.sleep(0.007)  # ~143Hz，与真实 iBUS 一致

    def stop(self):
        self._running = False
        self.wait(1500)


# --------------------------------------------------------------------------
# 界面组件
# --------------------------------------------------------------------------
class ChannelWidget(QWidget):
    def __init__(self, index, parent=None):
        super().__init__(parent)
        self.index = index
        self._lo, self._hi = 0, 4095
        self._color = None

        name = QLabel(f"CH{index}")
        name.setObjectName("chName")
        name.setFixedWidth(42)
        if index > 14:
            name.setToolTip("扩展通道（CH15~CH18 由低半字节拼接）")

        self.bar = QProgressBar()
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(16)
        self.bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.value = QLabel("—")
        self.value.setObjectName("chValue")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value.setFixedWidth(58)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 1, 2, 1)
        layout.setSpacing(8)
        layout.addWidget(name)
        layout.addWidget(self.bar, 1)
        layout.addWidget(self.value)

        self.set_range(0, 4095)

    def set_range(self, lo, hi):
        self._lo, self._hi = lo, hi
        self.bar.setRange(lo, hi)

    def set_value(self, v):
        if v is None:
            self.value.setText("—")
            self.bar.setValue(self._lo)
            self._apply_color("#2b3038")
            return
        v = max(self._lo, min(self._hi, int(v)))
        self.bar.setValue(v)
        self.value.setText(str(v))
        span = self._hi - self._lo or 1
        t = max(0.0, min(1.0, (v - self._lo) / span))
        r, g, b = colorsys.hsv_to_rgb(0.36 * t, 0.72, 0.88)  # 红 -> 绿
        self._apply_color("#%02x%02x%02x" % (int(r * 255), int(g * 255), int(b * 255)))

    def _apply_color(self, color):
        if color == self._color:
            return
        self._color = color
        self.bar.setStyleSheet(
            "QProgressBar { background: #232730; border: 1px solid #3a4150;"
            " border-radius: 3px; }"
            f"QProgressBar::chunk {{ background: {color}; border-radius: 2px; }}"
        )


STYLE = """
QMainWindow, QWidget { background: #1b1e24; color: #d8dee9; font-size: 13px; }
QGroupBox { border: 1px solid #333a45; border-radius: 6px;
            margin-top: 10px; padding: 8px 6px 6px 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px;
                   color: #8fa1b3; font-weight: 600; }
QPushButton { background: #2d333b; border: 1px solid #444c56; border-radius: 4px;
              padding: 5px 16px; }
QPushButton:hover { background: #373e47; }
QPushButton:pressed { background: #22272e; }
QPushButton:disabled { color: #5c6773; border-color: #333941; }
QComboBox { background: #22272e; border: 1px solid #444c56; border-radius: 4px;
            padding: 4px 8px; }
QComboBox QAbstractItemView { background: #22272e; selection-background-color: #316dca; }
QLineEdit { background: #161a1f; border: 1px solid #333a45; border-radius: 4px;
            padding: 5px 8px; color: #a5d6ff; selection-background-color: #316dca; }
QCheckBox { spacing: 6px; }
QLabel#chName { color: #8fa1b3; font-weight: 600; }
QLabel#chValue { font-family: "DejaVu Sans Mono", monospace; color: #e6edf3; }
QLabel#statTitle { color: #7d8998; }
QLabel#statValue { font-family: "DejaVu Sans Mono", monospace; color: #7ee787; }
QStatusBar { color: #8fa1b3; }
"""


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FlySky iBUS 接收机监视器")
        self.resize(1080, 640)

        self.worker = None
        self.frame_count = 0
        self._parse_errors = 0
        self._latest = [None] * 18
        self._latest_hex = ""
        self._last_count = 0
        self._last_time = time.monotonic()
        self._fps = 0.0

        self._build_ui()
        self.refresh_ports()
        self._apply_mode()

        self._display_timer = QTimer(self)
        self._display_timer.timeout.connect(self._update_display)
        self._display_timer.start(50)  # 20Hz 刷新界面

    # ---------------- UI ----------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 8)
        root.setSpacing(8)

        # 连接栏
        conn = QGroupBox("串口连接")
        h = QHBoxLayout(conn)
        h.setSpacing(8)
        h.addWidget(QLabel("端口"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(280)
        h.addWidget(self.port_combo)
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        h.addWidget(self.refresh_btn)
        self.sim_check = QCheckBox("模拟数据")
        self.sim_check.toggled.connect(self._on_sim_toggled)
        h.addWidget(self.sim_check)
        h.addStretch(1)
        h.addWidget(QLabel("解码模式"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([MODE_14, MODE_18])  # 默认 14 通道
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        h.addWidget(self.mode_combo)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setMinimumWidth(88)
        self.connect_btn.clicked.connect(self._toggle_connection)
        h.addWidget(self.connect_btn)
        root.addWidget(conn)

        # 统计栏
        stats = QHBoxLayout()
        stats.setSpacing(6)
        self.fps_label = self._stat(stats, "帧率")
        self.frames_label = self._stat(stats, "累计帧数")
        self.errors_label = self._stat(stats, "校验错误")
        stats.addStretch(1)
        root.addLayout(stats)

        # 通道
        ch_box = QGroupBox("通道数据")
        self._ch_grid = QGridLayout(ch_box)
        self._ch_grid.setHorizontalSpacing(30)
        self._ch_grid.setVerticalSpacing(6)
        self.channels = []
        for i in range(18):
            self.channels.append(ChannelWidget(i + 1, ch_box))
        root.addWidget(ch_box, 1)

        # 原始帧
        raw_box = QGroupBox("原始帧")
        raw_layout = QVBoxLayout(raw_box)
        raw_layout.setContentsMargins(6, 4, 6, 4)
        self.raw_label = QLineEdit()
        self.raw_label.setReadOnly(True)
        self.raw_label.setPlaceholderText("等待数据…")
        self.raw_label.setFont(QFont("DejaVu Sans Mono", 9))
        raw_layout.addWidget(self.raw_label)
        root.addWidget(raw_box)

        self.statusBar().showMessage("就绪")

    @staticmethod
    def _stat(layout, title):
        title_label = QLabel(title)
        title_label.setObjectName("statTitle")
        value = QLabel("—")
        value.setObjectName("statValue")
        layout.addWidget(title_label)
        layout.addWidget(value)
        layout.addSpacing(20)
        return value

    # ---------------- 模式 / 端口 ----------------
    def _apply_mode(self):
        mode = self.mode_combo.currentText()
        if mode == MODE_14:
            lo, hi, count = 1000, 2000, 14
        else:
            lo, hi, count = 0, 4095, 18
        self._layout_channels(count)
        for i, widget in enumerate(self.channels):
            widget.setVisible(i < count)
            widget.set_range(lo, hi)
            widget.set_value(None)
        self._latest = [None] * count

    def _layout_channels(self, count):
        """双列排布，行数随通道数自适应（14 通道 7 行，18 通道 9 行）。"""
        while self._ch_grid.count():
            item = self._ch_grid.takeAt(0)
            del item
        rows = (count + 1) // 2
        for i in range(count):
            self._ch_grid.addWidget(self.channels[i], i % rows, i // rows)

    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        ports = list(list_ports.comports())
        # 优先 USB 串口，过滤掉系统上大量无信息的 ttyS*
        usb_ports = [p for p in ports if p.hwid not in ("", "n/a") or p.description not in ("", "n/a")]
        if usb_ports:
            ports = usb_ports
        if not ports:
            self.port_combo.addItem("未检测到串口", None)
            self.statusBar().showMessage("未检测到串口设备", 4000)
            return
        for p in ports:
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def _on_sim_toggled(self, checked):
        self.port_combo.setEnabled(not checked)
        self.refresh_btn.setEnabled(not checked)

    def _on_mode_changed(self):
        if self.worker is None:
            self._apply_mode()

    # ---------------- 连接管理 ----------------
    def _toggle_connection(self):
        if self.worker is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        mode = self.mode_combo.currentText()
        if self.sim_check.isChecked():
            worker = SimWorker(mode)
        else:
            port = self.port_combo.currentData()
            if not port:
                self.statusBar().showMessage("请选择串口", 4000)
                return
            worker = SerialWorker(port, mode)
            worker.failed.connect(self._on_worker_failed)
            worker.parseErrors.connect(self._on_parse_errors)
        worker.frameReceived.connect(self._on_frame)

        self.worker = worker
        self._latest = [None] * (18 if mode == MODE_18 else 14)
        self._latest_hex = ""
        worker.start()

        self.connect_btn.setText("断开")
        self.port_combo.setEnabled(False)
        self.refresh_btn.setEnabled(False)
        self.sim_check.setEnabled(False)
        self.mode_combo.setEnabled(False)
        self.statusBar().showMessage(
            "已连接（模拟数据）" if self.sim_check.isChecked() else f"已连接 {worker.port}",
            4000,
        )

    def _disconnect(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.connect_btn.setText("连接")
        self.sim_check.setEnabled(True)
        self.mode_combo.setEnabled(True)
        self.port_combo.setEnabled(not self.sim_check.isChecked())
        self.refresh_btn.setEnabled(not self.sim_check.isChecked())
        self.raw_label.clear()
        self._latest_hex = ""
        self._latest = [None] * len(self._latest)
        for widget in self.channels:
            widget.set_value(None)
        self.statusBar().showMessage("已断开", 3000)

    def _on_worker_failed(self, message):
        self._disconnect()
        self.statusBar().showMessage(message, 10000)

    def _on_parse_errors(self, count):
        self._parse_errors += count

    # ---------------- 数据 ----------------
    def _on_frame(self, channels, raw_hex):
        self._latest = channels
        self._latest_hex = raw_hex
        self.frame_count += 1

    def _update_display(self):
        now = time.monotonic()
        dt = now - self._last_time
        if dt > 0.05:
            self._fps = (self.frame_count - self._last_count) / dt
            self._last_time = now
            self._last_count = self.frame_count
        self.fps_label.setText(f"{self._fps:.1f} Hz")
        self.frames_label.setText(str(self.frame_count))
        self.errors_label.setText(str(self._parse_errors))
        for i, widget in enumerate(self.channels):
            if widget.isVisible():
                widget.set_value(self._latest[i] if i < len(self._latest) else None)
        if self._latest_hex and self.raw_label.text() != self._latest_hex:
            self.raw_label.setText(self._latest_hex)

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        super().closeEvent(event)


def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
