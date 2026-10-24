#!/usr/bin/env python3
"""Eyeliner telemetry logger: ESP-NOW dongle (espnow_dongle_raw) -> CSV + live plot.

Reads raw frames from the dongle's USB serial port (0xAA 0x55 <len> <rssi> <frame>),
decodes TelemFast / TelemSlow (firmware/src/app/telemetry.h), appends every frame to
timestamped CSVs in match_logs/ and shows a rolling live plot.

    tools/telem/run.sh                 # auto-detect the dongle, plot + log
    tools/telem/run.sh --no-plot       # CSV only (headless)
    tools/telem/run.sh --port /dev/cu.usbmodemXXXX --window 20

Stop with Ctrl-C or by closing the plot window. The CSVs are flushed every second.
"""
import argparse
import collections
import csv
import datetime as dt
import os
import struct
import sys
import threading
import time

import serial
import serial.tools.list_ports

# --- Frame layout: keep in sync with firmware/src/app/telemetry.h -------------------
MAGIC = (ord("E"), ord("Y"))
VERSION = 1
TYPE_FAST, TYPE_SLOW = 1, 2
HDR_FMT = "<BBBBHH"  # magic0 magic1 type version len seq

FAST_FMT = HDR_FMT + "I" + "fff" + "fffff" + "ff" + "ff" + "HH" + "BBBB" + "4h" + "18f" + "II"
FAST_FIELDS = (
    ["magic0", "magic1", "type", "version", "len", "seq", "t_ms",
     "heading_deg", "rate_dps", "k",
     "spinup", "heading_cos", "translation", "tank_x", "tank_y",
     "M", "modulation", "throttle1", "throttle2", "dshot1", "dshot2",
     "mode", "flags", "lq", "rssi_rx",
     "ch_left_x", "ch_left_y", "ch_right_y", "ch_right_x"]
    + [f"imu{n}_{q}_{a}" for n in ("A", "B") for q in ("acc", "gyr", "hg") for a in "xyz"]
    + ["imu_seq", "rx_seq"]
)
SLOW_FMT = HDR_FMT + "I" + "f" + "H" + "f" + "ff" + "HHHHH" + "II" + "I" + "I" + "II" + "I" + "BBBB"
SLOW_FIELDS = [
    "magic0", "magic1", "type", "version", "len", "seq", "t_ms",
    "vbat_v", "vbat_sense_mv", "chip_temp_c", "imuA_temp_c", "imuB_temp_c",
    "comms_loops_s", "control_loops_s", "comms_busy_max_us", "control_busy_max_us", "uart_bytes_s",
    "imuA_errors", "imuB_errors", "led_frames", "motor_frames", "espnow_sent", "espnow_failed",
    "free_heap", "led_mode", "imu_present_mask", "pca_ok", "pad",
]
assert len(FAST_FIELDS) == len(struct.unpack(FAST_FMT, bytes(struct.calcsize(FAST_FMT))))
assert len(SLOW_FIELDS) == len(struct.unpack(SLOW_FMT, bytes(struct.calcsize(SLOW_FMT))))
FAST_LEN, SLOW_LEN = struct.calcsize(FAST_FMT), struct.calcsize(SLOW_FMT)

FLAG_RX_SAFE, FLAG_LINK_UP, FLAG_KILLED, FLAG_ARMED_1, FLAG_ARMED_2, FLAG_LED_ARC = (1 << i for i in range(6))
MODE_NAMES = {0: "melty", 1: "tank", 2: "flip"}

DONGLE_SERIAL = "E0:72:A1:F7:D0:E8"   # the telemetry XIAO
ROBOT_SERIAL = "E0:72:A1:F7:D8:7C"    # the robot's XIAO (never pick this one)
ESP32S3_VID_PID = (0x303A, 0x1001)


def find_port(explicit):
    if explicit:
        return explicit
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if (p.serial_number or "").upper() == DONGLE_SERIAL:
            return p.device
    for p in ports:
        if (p.vid, p.pid) == ESP32S3_VID_PID and (p.serial_number or "").upper() != ROBOT_SERIAL:
            return p.device
    sys.exit("no dongle found; ports: " + ", ".join(f"{p.device} ({p.serial_number})" for p in ports)
             + "\nflash it with: pio run -e espnow_dongle_raw -t upload, or pass --port")


class Decoder:
    """Parses the dongle byte stream and hands out decoded frames."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(b"\xaa\x55")
            if i < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                break
            if i:
                self.buf = self.buf[i:]
            if len(self.buf) < 4:
                break
            n, rssi = self.buf[2], struct.unpack("b", self.buf[3:4])[0]
            if len(self.buf) < 4 + n:
                break
            frame = bytes(self.buf[4:4 + n])
            del self.buf[:4 + n]
            f = self.decode(frame)
            if f is None:
                self.bad += 1
            else:
                f["dongle_rssi"] = rssi
                frames.append(f)
        return frames

    def decode(self, frame):
        if len(frame) < struct.calcsize(HDR_FMT):
            return None
        m0, m1, typ, ver, ln, _ = struct.unpack_from(HDR_FMT, frame)
        if (m0, m1) != MAGIC or ver != VERSION or ln != len(frame):
            return None
        if typ == TYPE_FAST and ln == FAST_LEN:
            return dict(zip(FAST_FIELDS, struct.unpack(FAST_FMT, frame)))
        if typ == TYPE_SLOW and ln == SLOW_LEN:
            return dict(zip(SLOW_FIELDS, struct.unpack(SLOW_FMT, frame)))
        return None


class Logger:
    def __init__(self, logdir):
        os.makedirs(logdir, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.paths = {TYPE_FAST: os.path.join(logdir, f"{stamp}_fast.csv"),
                      TYPE_SLOW: os.path.join(logdir, f"{stamp}_slow.csv")}
        self.files, self.writers = {}, {}
        extra = ["host_time", "host_s", "dongle_rssi"]
        for typ, fields in ((TYPE_FAST, FAST_FIELDS), (TYPE_SLOW, SLOW_FIELDS)):
            f = open(self.paths[typ], "w", newline="")
            w = csv.writer(f)
            w.writerow(extra + fields[2:])  # drop the magic bytes
            self.files[typ], self.writers[typ] = f, w
        self.t0 = time.time()
        self.last_flush = self.t0

    def write(self, fr):
        now = time.time()
        fields = FAST_FIELDS if fr["type"] == TYPE_FAST else SLOW_FIELDS
        row = [dt.datetime.fromtimestamp(now).isoformat(timespec="milliseconds"), f"{now - self.t0:.3f}",
               fr["dongle_rssi"]] + [fr[k] for k in fields[2:]]
        self.writers[fr["type"]].writerow(row)
        if now - self.last_flush > 1.0:
            for f in self.files.values():
                f.flush()
            self.last_flush = now

    def close(self):
        for f in self.files.values():
            f.close()


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.fast = self.slow = self.lost = 0
        self.last_seq = None
        self.window = collections.deque()  # (time, type)

    def add(self, fr, now):
        with self.lock:
            if fr["type"] == TYPE_FAST:
                self.fast += 1
                if self.last_seq is not None:
                    gap = (fr["seq"] - self.last_seq - 1) & 0xFFFF
                    self.lost += gap if gap < 30000 else 0
                self.last_seq = fr["seq"]
            else:
                self.slow += 1
            self.window.append((now, fr["type"]))
            while self.window and now - self.window[0][0] > 2.0:
                self.window.popleft()

    def rate(self):
        with self.lock:
            n = sum(1 for _, t in self.window if t == TYPE_FAST)
            return n / 2.0, self.fast, self.slow, self.lost


def reader_thread(port, decoder, logger, stats, history, slow_state, stop):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.05
    s.dtr = s.rts = True  # assert before open and never toggle: a DTR/RTS edge resets the XIAO
    s.open()
    try:
        while not stop.is_set():
            data = s.read(4096)
            if not data:
                continue
            now = time.time()
            for fr in decoder.feed(data):
                logger.write(fr)
                stats.add(fr, now)
                if fr["type"] == TYPE_FAST:
                    with history.lock:
                        history.add(fr, now)
                else:
                    slow_state.update(fr)
    finally:
        s.close()


class History:
    def __init__(self, seconds):
        self.lock = threading.Lock()
        self.seconds = seconds
        self.rows = collections.deque()

    def add(self, fr, now):
        self.rows.append((now, fr))
        while self.rows and now - self.rows[0][0] > self.seconds:
            self.rows.popleft()

    def columns(self, keys):
        with self.lock:
            if not self.rows:
                return None
            t0 = self.rows[-1][0]
            out = {"t": [r[0] - t0 for r in self.rows]}
            for k in keys:
                out[k] = [r[1][k] for r in self.rows]
            return out


def run_plot(history, slow_state, stats, window, stop):
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    plt.rcParams["toolbar"] = "None"
    fig, axes = plt.subplots(6, 1, sharex=True, figsize=(12, 10))
    fig.canvas.manager.set_window_title("Eyeliner telemetry")
    panels = [
        ("heading [deg]", ["heading_deg"], (0, 360)),
        ("spin rate [dps]", ["rate_dps"], None),
        ("throttle", ["spinup", "throttle1", "throttle2", "modulation"], (-1.05, 1.05)),
        ("stop logic", ["heading_cos", "M", "translation"], (-1.05, 1.05)),
        ("IMU radial (Y) high-g [m/s2]", ["imuA_hg_y", "imuB_hg_y"], None),
        ("link", ["lq", "rssi_rx"], (0, 110)),
    ]
    lines = {}
    for ax, (title, keys, ylim) in zip(axes, panels):
        for k in keys:
            (lines[k],) = ax.plot([], [], lw=1, label=k)
        ax.set_ylabel(title, fontsize=8)
        ax.legend(loc="upper left", fontsize=7, ncol=len(keys))
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-window, 0)
        if ylim:
            ax.set_ylim(*ylim)
    axes[-1].set_xlabel("seconds ago")
    status = fig.text(0.01, 0.995, "", va="top", fontsize=9, family="monospace")
    keys = [k for _, ks, _ in panels for k in ks]

    def update(_):
        cols = history.columns(keys)
        if cols:
            t = cols["t"]
            for ax, (_, ks, ylim) in zip(axes, panels):
                for k in ks:
                    lines[k].set_data(t, cols[k])
                if ylim is None:
                    ax.relim()
                    ax.autoscale_view(scalex=False)
            last = history.rows[-1][1]
            flags = last["flags"]
            state = ("RXFAIL " if not flags & FLAG_RX_SAFE else "") + ("KILL " if flags & FLAG_KILLED else "") \
                + ("" if flags & FLAG_ARMED_1 and flags & FLAG_ARMED_2 else "ARMING ")
            rate, nf, ns, lost = stats.rate()
            sl = slow_state.get()
            status.set_text(
                f"mode {MODE_NAMES.get(last['mode'], '?'):5s} {state:14s} hdg {last['heading_deg']:6.1f}  "
                f"rate {last['rate_dps']:7.1f} dps  K {last['k']:5.1f} | vbat {sl.get('vbat_v', float('nan')):5.2f} V  "
                f"chip {sl.get('chip_temp_c', float('nan')):4.1f} C | frames {rate:5.1f}/s  lost {lost}  "
                f"dongle RSSI {last['dongle_rssi']} dBm")
        return list(lines.values()) + [status]

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    fig.canvas.mpl_connect("close_event", lambda _e: stop.set())
    plt.tight_layout(rect=(0, 0, 1, 0.97))
    plt.show()
    stop.set()
    return ani


class SlowState:
    def __init__(self):
        self.lock = threading.Lock()
        self.d = {}

    def update(self, fr):
        with self.lock:
            self.d = dict(fr)

    def get(self):
        with self.lock:
            return dict(self.d)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_logdir = os.path.normpath(os.path.join(here, "..", "..", "..", "match_logs"))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="dongle serial port (default: auto-detect by USB serial)")
    ap.add_argument("--logdir", default=default_logdir, help=f"CSV directory (default {default_logdir})")
    ap.add_argument("--window", type=float, default=15.0, help="plot window in seconds (default 15)")
    ap.add_argument("--no-plot", action="store_true", help="CSV only, print a status line instead")
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until Ctrl-C)")
    args = ap.parse_args()

    port = find_port(args.port)
    decoder, logger, stats = Decoder(), Logger(args.logdir), Stats()
    history, slow_state, stop = History(args.window), SlowState(), threading.Event()
    print(f"dongle {port}; fast frame {FAST_LEN} B, slow frame {SLOW_LEN} B")
    print(f"logging to {logger.paths[TYPE_FAST]}\n           {logger.paths[TYPE_SLOW]}")

    th = threading.Thread(target=reader_thread, args=(port, decoder, logger, stats, history, slow_state, stop),
                          daemon=True)
    th.start()
    t_start = time.time()
    try:
        if args.no_plot:
            while not stop.is_set():
                time.sleep(1.0)
                rate, nf, ns, lost = stats.rate()
                sl = slow_state.get()
                with history.lock:
                    last = history.rows[-1][1] if history.rows else None
                hdg = f"hdg {last['heading_deg']:6.1f} mode {MODE_NAMES.get(last['mode'], '?')}" if last else "no fast frame yet"
                print(f"{rate:5.1f} fast/s  total fast {nf} slow {ns} lost {lost} bad {decoder.bad} | {hdg} | "
                      f"vbat {sl.get('vbat_v', float('nan')):5.2f} V", flush=True)
                if args.duration and time.time() - t_start > args.duration:
                    break
        else:
            if args.duration:
                threading.Timer(args.duration, stop.set).start()
            run_plot(history, slow_state, stats, args.window, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=1.0)
        logger.close()
        rate, nf, ns, lost = stats.rate()
        print(f"\nsaved {nf} fast + {ns} slow frames ({lost} lost, {decoder.bad} bad) to {args.logdir}")


if __name__ == "__main__":
    main()
