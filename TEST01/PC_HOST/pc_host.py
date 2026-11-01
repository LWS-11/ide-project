"""Tkinter host for the STM32F746 CH1-pot/CH2-microphone acquisition demo."""

from __future__ import annotations

import argparse
import pathlib
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import messagebox, ttk

_VENDOR_DIR = pathlib.Path(__file__).resolve().parent / "vendor"
if _VENDOR_DIR.is_dir():
    sys.path.insert(0, str(_VENDOR_DIR))

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

import pc_protocol as protocol

BAUD_RATE = 115200
ADC_REFERENCE = 3.3
ADC_MAX = 4095


@dataclass
class UiFrame:
    received_at: float
    data: protocol.DataFrame


class SerialWorker:
    def __init__(
        self,
        port: str,
        frame_queue: queue.Queue[UiFrame],
        error_queue: queue.Queue[str],
    ) -> None:
        self.port = port
        self.frame_queue = frame_queue
        self.error_queue = error_queue
        self._stop_event = threading.Event()
        self._serial = None
        self._thread = threading.Thread(target=self._run, name="serial-reader", daemon=True)
        self.crc_errors = 0
        self.discarded_bytes = 0

    def start(self) -> None:
        self._serial = serial.Serial(
            self.port,
            BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.5,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def send(self, payload: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            raise serial.SerialException("Serial port is not open")
        self._serial.write(payload)

    def _run(self) -> None:
        parser = protocol.FrameParser()
        try:
            while not self._stop_event.is_set():
                chunk = self._serial.read(max(1, self._serial.in_waiting))
                if not chunk:
                    continue
                for frame in parser.feed(chunk):
                    if isinstance(frame, protocol.DataFrame):
                        self.frame_queue.put(UiFrame(time.monotonic(), frame))
                self.crc_errors = parser.crc_errors
                self.discarded_bytes = parser.discarded_bytes
        except Exception as exc:  # The GUI reports worker failures to the user.
            self.error_queue.put(str(exc))
        finally:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass


class WaveformCanvas(tk.Canvas):
    def __init__(self, master: tk.Misc, **kwargs) -> None:
        super().__init__(master, background="#101720", highlightthickness=0, **kwargs)
        self.ch1: tuple[int, ...] = ()
        self.ch2: tuple[int, ...] = ()
        self.scan_seconds = 0.010
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_waveforms(
        self,
        ch1: tuple[int, ...],
        ch2: tuple[int, ...],
        scan_seconds: float,
    ) -> None:
        self.ch1 = ch1
        self.ch2 = ch2
        self.scan_seconds = scan_seconds
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 300)
        height = max(self.winfo_height(), 200)
        left, right, top, bottom = 58, 18, 20, 42
        plot_width = width - left - right
        plot_height = height - top - bottom

        for index in range(9):
            y = top + (plot_height * index / 8)
            self.create_line(left, y, width - right, y, fill="#263442")
            voltage = ADC_REFERENCE * (1.0 - index / 8)
            self.create_text(
                left - 8,
                y,
                text=f"{voltage:.1f}",
                fill="#9FB2C3",
                anchor="e",
                font=("Consolas", 9),
            )

        for index in range(11):
            x = left + (plot_width * index / 10)
            self.create_line(x, top, x, height - bottom, fill="#263442")
            scan_ms = self.scan_seconds * 1000
            self.create_text(
                x,
                height - bottom + 10,
                text=f"{scan_ms * index / 10:.2f}",
                fill="#9FB2C3",
                anchor="n",
                font=("Consolas", 9),
            )

        self.create_text(
            width - right,
            height - 7,
            text="time / ms",
            fill="#9FB2C3",
            anchor="se",
            font=("Microsoft YaHei UI", 9),
        )
        self.create_text(
            left + 12,
            top + 8,
            text="CH1 Pot  IN7 / PF9",
            fill="#42D6FF",
            anchor="nw",
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        self.create_text(
            left + 190,
            top + 8,
            text="CH2 Mic  IN0 / PA0",
            fill="#FFB84D",
            anchor="nw",
            font=("Microsoft YaHei UI", 10, "bold"),
        )

        self._draw_channel(self.ch1, "#42D6FF", left, top, plot_width, plot_height)
        self._draw_channel(self.ch2, "#FFB84D", left, top, plot_width, plot_height)

    def _draw_channel(
        self,
        samples: tuple[int, ...],
        color: str,
        left: float,
        top: float,
        width: float,
        height: float,
    ) -> None:
        if len(samples) < 2:
            return

        points: list[float] = []
        denominator = max(1, len(samples) - 1)
        for index, sample_mv in enumerate(samples):
            x = left + (width * index / denominator)
            voltage = min(max(sample_mv / 1000.0, 0.0), ADC_REFERENCE)
            y = top + height * (1.0 - voltage / ADC_REFERENCE)
            points.extend((x, y))
        self.create_line(*points, fill=color, width=2, smooth=False)


class HostApplication:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("STM32F746 双通道数据采集（团队协议）")
        self.root.geometry("1040x680")
        self.root.minsize(760, 520)

        self.frame_queue: queue.Queue[UiFrame] = queue.Queue(maxsize=200)
        self.error_queue: queue.Queue[str] = queue.Queue()
        self.worker: SerialWorker | None = None
        self.valid_frames = 0
        self.last_sequence: int | None = None
        self.sequence_drops = 0
        self.last_command: int | None = None
        self.received_times: deque[float] = deque()
        self.latest_frame: protocol.DataFrame | None = None

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="请选择 ST-Link Virtual COM 端口")
        self.fps_var = tk.StringVar(value="0.0 fps")
        self.packet_var = tk.StringVar(value="有效帧 0 | CRC 0 | 丢帧 0")
        self.ch1_var = tk.StringVar(value="CH1  Pot: --")
        self.ch2_var = tk.StringVar(value="CH2  Mic: --")

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._poll)

    def _build_ui(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(12, 7))
        style.configure("Mode.TButton", font=("Microsoft YaHei UI", 10, "bold"))

        toolbar = ttk.Frame(self.root, padding=(12, 10))
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="串口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(
            toolbar,
            textvariable=self.port_var,
            width=24,
            state="readonly",
        )
        self.port_box.pack(side=tk.LEFT, padx=(6, 4))
        ttk.Button(toolbar, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT, padx=2)
        self.connect_button = ttk.Button(toolbar, text="连接", command=self._toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=(8, 2))

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=12)
        ttk.Label(toolbar, text="扫描时间").pack(side=tk.LEFT, padx=(0, 5))
        self.mode_buttons: list[ttk.Button] = []
        for mode, label in enumerate(protocol.SCAN_LABELS):
            button = ttk.Button(
                toolbar,
                text=label,
                style="Mode.TButton",
                command=lambda selected=mode: self._send_scan_mode(selected),
            )
            button.pack(side=tk.LEFT, padx=3)
            self.mode_buttons.append(button)

        self.waveform = WaveformCanvas(self.root)
        self.waveform.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 8))

        readout = ttk.Frame(self.root, padding=(14, 10))
        readout.pack(fill=tk.X)
        ttk.Label(
            readout,
            textvariable=self.ch1_var,
            foreground="#007EA7",
            font=("Consolas", 11, "bold"),
        ).pack(side=tk.LEFT)
        ttk.Label(
            readout,
            textvariable=self.ch2_var,
            foreground="#A45B00",
            font=("Consolas", 11, "bold"),
        ).pack(side=tk.LEFT, padx=28)
        ttk.Label(readout, textvariable=self.fps_var).pack(side=tk.RIGHT)
        ttk.Label(readout, textvariable=self.packet_var).pack(side=tk.RIGHT, padx=16)

        ttk.Label(
            self.root,
            textvariable=self.status_var,
            anchor="w",
            padding=(14, 7),
            relief=tk.SUNKEN,
        ).pack(fill=tk.X, side=tk.BOTTOM)

    def _refresh_ports(self) -> None:
        if list_ports is None:
            ports: list[str] = []
            preferred = None
        else:
            detected = sorted(list_ports.comports(), key=lambda item: item.device)
            ports = [port.device for port in detected]
            preferred = next(
                (
                    port.device
                    for port in detected
                    if "stlink" in (port.description or "").lower()
                    or "stmicroelectronics" in (port.manufacturer or "").lower()
                    or "stlink" in (port.hwid or "").lower()
                ),
                None,
            )
        if not ports:
            ports = [f"COM{number}" for number in range(1, 17)]
            preferred = None

        current = self.port_var.get()
        self.port_box["values"] = ports
        if current in ports:
            self.port_var.set(current)
        elif preferred is not None:
            self.port_var.set(preferred)
        elif ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self) -> None:
        if self.worker is None:
            self._connect()
        else:
            self._disconnect("已断开串口")

    def _connect(self) -> None:
        if serial is None:
            messagebox.showerror(
                "缺少 pyserial",
                "请先运行：python -m pip install -r requirements.txt",
            )
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("未选择串口", "请选择 ST-Link Virtual COM 端口。")
            return

        worker = SerialWorker(port, self.frame_queue, self.error_queue)
        try:
            worker.start()
        except Exception as exc:
            messagebox.showerror("无法打开串口", str(exc))
            self.status_var.set(f"无法打开 {port}：{exc}")
            return

        self.worker = worker
        self.connect_button.configure(text="断开")
        self.status_var.set(f"已连接 {port} @ {BAUD_RATE} 8N1")
        self.root.after(100, self._send_scan_mode, protocol.SCAN_10MS)

    def _disconnect(self, status: str) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
            self.last_sequence = None
        self.connect_button.configure(text="连接")
        self.status_var.set(status)

    def _send_scan_mode(self, mode: int) -> None:
        if self.worker is None:
            self.status_var.set("串口未连接")
            return

        try:
            self.worker.send(protocol.build_control(mode))
            self.last_command = mode
            self.status_var.set(
                f"已发送换档控制包：{protocol.SCAN_LABELS[mode]} / "
                f"{protocol.SCAN_INTERVALS_US[mode]} us"
            )
        except Exception as exc:
            self.status_var.set(f"发送失败：{exc}")

    def _poll(self) -> None:
        newest: UiFrame | None = None
        while True:
            try:
                candidate = self.frame_queue.get_nowait()
            except queue.Empty:
                break

            self.valid_frames += 1
            if self.last_sequence is not None:
                expected = (self.last_sequence + 1) & 0xFFFF
                if candidate.data.sequence != expected:
                    self.sequence_drops += (
                        candidate.data.sequence - expected
                    ) & 0xFFFF
            self.last_sequence = candidate.data.sequence
            self.received_times.append(candidate.received_at)
            newest = candidate

        while (
            self.received_times
            and newest is not None
            and newest.received_at - self.received_times[0] > 2.0
        ):
            self.received_times.popleft()

        if newest is not None:
            data = newest.data
            self.latest_frame = data
            if len(self.received_times) >= 2:
                span = self.received_times[-1] - self.received_times[0]
                fps = (len(self.received_times) - 1) / span if span > 0 else 0.0
            else:
                fps = 0.0

            self.fps_var.set(f"{fps:.1f} fps")
            self.packet_var.set(
                f"有效帧 {self.valid_frames} | "
                f"CRC {self.worker.crc_errors if self.worker else 0} | "
                f"丢帧 {self.sequence_drops}"
            )
            self.waveform.set_waveforms(
                data.ch1_mv,
                data.ch2_mv,
                data.scan_seconds,
            )
            self.ch1_var.set(self._channel_summary("CH1  Pot", data.ch1_mv))
            self.ch2_var.set(self._channel_summary("CH2  Mic", data.ch2_mv))
            self.status_var.set(
                f"接收正常 | 模式 {protocol.SCAN_LABELS[data.scan_mode]} | "
                f"间隔 {data.interval_us} us | 帧序号 {data.sequence}"
            )

        try:
            while True:
                error = self.error_queue.get_nowait()
                self._disconnect(f"串口错误：{error}")
        except queue.Empty:
            pass

        self.root.after(50, self._poll)

    @staticmethod
    def _channel_summary(name: str, samples_mv: tuple[int, ...]) -> str:
        if not samples_mv:
            return f"{name}: --"
        low = min(samples_mv)
        high = max(samples_mv)
        latest = samples_mv[-1]
        return (
            f"{name}: {latest:4d} mV / {latest / 1000.0:1.3f} V  "
            f"[{low:4d}..{high:4d}]"
        )

    def _on_close(self) -> None:
        if self.worker is not None:
            self.worker.stop()
        self.root.destroy()


def main() -> None:
    argument_parser = argparse.ArgumentParser(description="STM32F746 two-channel waveform host")
    argument_parser.add_argument(
        "--auto-connect",
        action="store_true",
        help="connect to the selected serial port when the window opens",
    )
    arguments = argument_parser.parse_args()

    root = tk.Tk()
    application = HostApplication(root)
    if arguments.auto_connect:
        root.after(200, application._connect)
    root.mainloop()


if __name__ == "__main__":
    main()

