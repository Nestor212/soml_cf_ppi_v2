#!/usr/bin/env python3
import argparse
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

try:
    import serial
except Exception:
    serial = None

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


HEATER_COUNT = 30
ROW_ORDER = ["temp", "fault", "relay", "voltage", "local_current", "remote_current"]
ROW_LABELS = {
    "temp": "Temp",
    "fault": "Fault",
    "relay": "Relay",
    "voltage": "Voltage",
    "local_current": "Local I",
    "remote_current": "Remote I",
}

LINE_RE = re.compile(
    r"^H(?P<idx>\d+)\s+"
    r"temp=(?P<temp>-?\d+(?:\.\d+)?)\s+"
    r"relay=(?P<relay>[01])\s+"
    r"V=(?P<V>[01])\s+"
    r"LI=(?P<LI>[01])\s+"
    r"RI=(?P<RI>[01])\s+"
    r"ioValid=(?P<ioValid>[01])\s+"
    r"faultPin=(?P<faultPin>[01])\s*$"
)


@dataclass
class HeaterSample:
    temp: float = float("nan")
    relay: int = 0
    voltage: int = 0
    local_current: int = 0
    remote_current: int = 0
    io_valid: int = 0
    fault_pin: int = 0
    updated_at: float = 0.0


class SerialReader(threading.Thread):
    def __init__(self, source, out_queue: queue.Queue, is_serial: bool = False):
        super().__init__(daemon=True)
        self.source = source
        self.out_queue = out_queue
        self.is_serial = is_serial
        self._stop_requested = False

    def stop(self):
        self._stop_requested = True

    def run(self):
        while not self._stop_requested:
            try:
                line = self.source.readline()
                if self.is_serial and isinstance(line, bytes):
                    line = line.decode("utf-8", errors="replace")
                if not line:
                    time.sleep(0.02)
                    continue
                self.out_queue.put(line.strip())
            except Exception as exc:
                self.out_queue.put(f"__ERROR__ {exc}")
                time.sleep(0.25)


class LedWidget(QFrame):
    def __init__(self, diameter: int = 24):
        super().__init__()
        self.diameter = diameter
        self.setFixedSize(20, 20)
        self.set_on(False)

    def set_color(self, color: str, on: bool = False):
        if on:
            border = "#555555"
            bg = color
        else:
            border = "#444444"
            bg = "#2d2d2d"
        radius = self.diameter // 2
        self.setStyleSheet(
            f"background-color: {bg}; border: 1px solid {border}; border-radius: {radius}px;"
        )

    def set_on(self, on: bool):
        self.set_color("#37d67a", on)


class TempCell(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(
            "background-color: #222831; border: 1px solid #3a3f47; border-radius: 8px;"
        )
        self.setMinimumSize(50, 36)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)

        self.label = QLabel("--.-")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("color: #eaeaea; font-weight: bold; font-size: 12px;")
        layout.addWidget(self.label)

    def set_text(self, text: str):
        self.label.setText(text)


class PowerPanelSimulator(QMainWindow):
    def __init__(self, line_queue: queue.Queue):
        super().__init__()
        self.queue = line_queue
        self.samples: Dict[int, HeaterSample] = {i: HeaterSample() for i in range(HEATER_COUNT)}
        self.temp_widgets: Dict[int, TempCell] = {}
        self.fault_leds: Dict[int, LedWidget] = {}
        self.relay_leds: Dict[int, LedWidget] = {}
        self.voltage_leds: Dict[int, LedWidget] = {}
        self.local_leds: Dict[int, LedWidget] = {}
        self.remote_leds: Dict[int, LedWidget] = {}
        self.line_count = 0
        self.last_frame_complete_at = 0.0
        self.last_error = ""

        self.setWindowTitle("Power Panel LED Simulator")
        self.resize(1900, 520)
        self._build_ui()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._drain_queue)
        self.poll_timer.start(10)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(500)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        central.setStyleSheet("background-color: #171717;")

        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        title = QLabel("30-Heater Power Panel Simulator")
        title.setStyleSheet("color: #f2f2f2; font-weight: bold; font-size: 14px;")
        root.addWidget(title)

        subtitle = QLabel(
            "Columns: label + H0-H29   |   Rows: temp, fault, relay, voltage, local current, remote current"
        )
        subtitle.setStyleSheet("color: #bdbdbd; font-size: 11px;")
        root.addWidget(subtitle)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        root.addWidget(scroll)

        grid_host = QWidget()
        grid_host.setStyleSheet("background-color: #171717;")
        scroll.setWidget(grid_host)

        grid = QGridLayout(grid_host)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(6)
        grid.setContentsMargins(0, 0, 0, 0)

        corner = self._make_header_label("Signal")
        grid.addWidget(corner, 0, 0)

        for i in range(HEATER_COUNT):
            grid.addWidget(self._make_header_label(f"H{i}"), 0, i + 1)

        for row_idx, row_key in enumerate(ROW_ORDER, start=1):
            grid.addWidget(self._make_row_label(ROW_LABELS[row_key]), row_idx, 0)

        for i in range(HEATER_COUNT):
            temp = TempCell()
            self.temp_widgets[i] = temp
            grid.addWidget(temp, 1, i + 1)

            fault = self._centered_led()
            relay = self._centered_led()
            voltage = self._centered_led()
            local_i = self._centered_led()
            remote_i = self._centered_led()

            self.fault_leds[i] = fault
            self.relay_leds[i] = relay
            self.voltage_leds[i] = voltage
            self.local_leds[i] = local_i
            self.remote_leds[i] = remote_i

            grid.addWidget(self._wrap_center(fault), 2, i + 1)
            grid.addWidget(self._wrap_center(relay), 3, i + 1)
            grid.addWidget(self._wrap_center(voltage), 4, i + 1)
            grid.addWidget(self._wrap_center(local_i), 5, i + 1)
            grid.addWidget(self._wrap_center(remote_i), 6, i + 1)

        self.status_label = QLabel("Waiting for data...")
        self.status_label.setStyleSheet("color: #d0d0d0; font-size: 12px;")
        root.addWidget(self.status_label)

    def _make_header_label(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumWidth(45)
        lbl.setStyleSheet("color: #f2f2f2; font-weight: bold; font-size: 13px;")
        return lbl

    def _make_row_label(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        lbl.setStyleSheet("color: #f2f2f2; font-weight: bold; font-size: 13px;")
        return lbl

    def _centered_led(self) -> LedWidget:
        led = LedWidget(24)
        led.set_color("#2d2d2d", on=False)
        return led

    def _wrap_center(self, widget: QWidget) -> QWidget:
        host = QWidget()
        layout = QHBoxLayout(host)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addStretch()
        layout.addWidget(widget)
        layout.addStretch()
        return host

    def _drain_queue(self):
        updated = False
        while True:
            try:
                line = self.queue.get_nowait()
            except queue.Empty:
                break

            if line.startswith("__ERROR__"):
                self.last_error = line
                continue

            parsed = self._parse_line(line)
            if parsed is None:
                continue

            idx, sample = parsed
            self.samples[idx] = sample
            self.line_count += 1
            self._apply_sample(idx, sample)
            updated = True

        if updated and all(self.samples[i].updated_at > 0 for i in range(HEATER_COUNT)):
            self.last_frame_complete_at = time.time()

    def _parse_line(self, line: str) -> Optional[Tuple[int, HeaterSample]]:
        m = LINE_RE.match(line)
        if not m:
            return None

        idx = int(m.group("idx"))
        if idx < 0 or idx >= HEATER_COUNT:
            return None

        sample = HeaterSample(
            temp=float(m.group("temp")),
            relay=int(m.group("relay")),
            voltage=int(m.group("V")),
            local_current=int(m.group("LI")),
            remote_current=int(m.group("RI")),
            io_valid=int(m.group("ioValid")),
            fault_pin=int(m.group("faultPin")),
            updated_at=time.time(),
        )
        return idx, sample

    def _apply_sample(self, idx: int, s: HeaterSample):
        self.temp_widgets[idx].set_text(f"{s.temp:.1f}")

        self.fault_leds[idx].set_color("#ff5b57", on=bool(s.fault_pin))
        self.relay_leds[idx].set_color("#ffc857", on=bool(s.relay))

        if s.io_valid:
            self.voltage_leds[idx].set_color("#57a6ff", on=bool(s.voltage))
            self.local_leds[idx].set_color("#37d67a", on=bool(s.local_current))
            self.remote_leds[idx].set_color("#37d67a", on=bool(s.remote_current))
        else:
            self.voltage_leds[idx].set_color("#57a6ff", on=False)
            self.local_leds[idx].set_color("#37d67a", on=False)
            self.remote_leds[idx].set_color("#37d67a", on=False)

    def _refresh_status(self):
        seen = sum(1 for s in self.samples.values() if s.updated_at > 0)
        if self.last_error:
            self.status_label.setText(self.last_error)
            return

        if self.last_frame_complete_at > 0:
            age = time.time() - self.last_frame_complete_at
            self.status_label.setText(
                f"Heaters seen: {seen}/30   Parsed lines: {self.line_count}   Last complete frame: {age:.1f}s ago"
            )
        else:
            self.status_label.setText(
                f"Heaters seen: {seen}/30   Parsed lines: {self.line_count}   Waiting for first complete frame..."
            )


def build_source(args):
    if args.stdin:
        return sys.stdin, False

    if args.port:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install with: pip install pyserial")
        ser = serial.Serial(args.port, args.baud, timeout=0.25)
        return ser, True

    raise RuntimeError("Provide either --port COM3 or --stdin")


def main():
    parser = argparse.ArgumentParser(description="PyQt power panel LED simulator for 30 heaters")
    parser.add_argument("--port", help="Serial port, for example COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=460800, help="Serial baud rate")
    parser.add_argument("--stdin", action="store_true", help="Read heater lines from stdin instead of a serial port")
    args = parser.parse_args()

    try:
        source, is_serial = build_source(args)
    except Exception as exc:
        print(f"Startup error: {exc}", file=sys.stderr)
        sys.exit(1)

    line_queue = queue.Queue()
    reader = SerialReader(source, line_queue, is_serial=is_serial)
    reader.start()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = PowerPanelSimulator(line_queue)
    win.show()

    rc = app.exec_()
    reader.stop()
    sys.exit(rc)


if __name__ == "__main__":
    main()
