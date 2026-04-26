#!/usr/bin/env python3
"""
Live plot Beyondex USB audio diagnostics over HID.

The firmware (when built with BEYONDEX_HID_DEBUG=1) exposes a generic HID
input report carrying `beyondex_audio_diag_t` (see beyondex_diag.h). This
script reads those reports cross-platform via hidapi.

Payload (little-endian, 34 bytes total, packed):
  uint32 magic              = 0x44584542 ('BEXD')
  uint32 underrun
  uint32 overflow
  int32  buf_len
  int32  buf_us
  int32  feedback
  uint32 sof_calls
  uint32 sof_updates
  uint16 streaming_enabled

Requirements:
  pip install hidapi matplotlib
  - macOS: works out of the box (uses IOHIDManager)
  - Linux: may need udev rule for non-root access (e.g. plugdev group)
  - Windows: HID is driver-less, no Zadig needed
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional


MAGIC = 0x44584542  # 'BEXD'
DIAG_FMT = "<IIIiiiIIH"
DIAG_SIZE = struct.calcsize(DIAG_FMT)  # 34


@dataclass(frozen=True)
class Diag:
    t: float
    underrun: int
    overflow: int
    buf_len: int
    buf_us: int
    feedback: int
    sof_calls: int
    sof_updates: int
    streaming: bool


def parse_report(buf: bytes) -> Optional[Diag]:
    """Parse a HID report. Some platforms prepend a 0 report-id byte; tolerate
    both layouts by trying offset 0 first, then offset 1."""
    if len(buf) < DIAG_SIZE:
        return None
    for off in (0, 1):
        if len(buf) < off + DIAG_SIZE:
            continue
        magic = struct.unpack_from("<I", buf, off)[0]
        if magic == MAGIC:
            (m, u, o, blen, bus, fb, sc, su, se) = struct.unpack_from(
                DIAG_FMT, buf, off
            )
            return Diag(
                t=time.time(),
                underrun=u,
                overflow=o,
                buf_len=blen,
                buf_us=bus,
                feedback=fb,
                sof_calls=sc,
                sof_updates=su,
                streaming=bool(se),
            )
    return None


def open_device(vid: int, pid: Optional[int]):
    try:
        import hid
    except ImportError:
        print(
            "Missing dependency: install with `pip install hidapi`.\n"
            "(The package name is `hidapi`; it imports as `hid`.)",
            file=sys.stderr,
        )
        sys.exit(1)

    candidates = hid.enumerate(vid, pid or 0)
    if not candidates:
        return None

    # Prefer the interface whose usage_page suggests vendor-defined data
    # (TinyUSB's GENERIC_INOUT report uses usage_page 0xFFxx).
    def pick_priority(info):
        up = info.get("usage_page", 0)
        return (0 if up >= 0xFF00 else 1, info.get("interface_number", 0))

    candidates.sort(key=pick_priority)

    for info in candidates:
        dev = hid.device()
        try:
            dev.open_path(info["path"])
        except OSError as e:
            print(f"open {info['path']!r}: {e}", file=sys.stderr)
            continue
        dev.set_nonblocking(True)
        return dev
    return None


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Live plot Beyondex USB audio diagnostics (HID input reports)."
    )
    ap.add_argument("--vid", default="0xCAFE", help="USB VID (default: 0xCAFE)")
    ap.add_argument("--pid", default=None, help="USB PID (optional, e.g. 0x4014)")
    ap.add_argument(
        "--window", type=float, default=60.0, help="Plot window seconds (default: 60)"
    )
    ap.add_argument("--csv", default=None, help="Optional CSV log path")
    args = ap.parse_args()

    vid = int(args.vid, 0)
    pid = int(args.pid, 0) if args.pid is not None else None

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    maxlen = 4096
    ts: deque[float] = deque(maxlen=maxlen)
    buf_us: deque[int] = deque(maxlen=maxlen)
    buf_len: deque[int] = deque(maxlen=maxlen)
    underrun: deque[int] = deque(maxlen=maxlen)
    overflow: deque[int] = deque(maxlen=maxlen)
    feedback_q: deque[float] = deque(maxlen=maxlen)
    sof_miss: deque[int] = deque(maxlen=maxlen)

    csv_f = None
    if args.csv:
        csv_f = open(args.csv, "w", encoding="utf-8")
        csv_f.write(
            "t,buf_us,buf_len,underrun,overflow,feedback_frames_per_ms,sof_calls,sof_updates,streaming\n"
        )
        csv_f.flush()

    dev = None
    last_connect_attempt = 0.0
    t0: Optional[float] = None

    fig, (ax0, ax1, ax2) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.suptitle("Beyondex USB audio diagnostics (HID)")

    (l_buf_us,) = ax0.plot([], [], label="buf_us")
    ax0.set_ylabel("buffered µs")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="upper right")

    (l_buf_len,) = ax1.plot([], [], label="buf_len")
    (l_underrun,) = ax1.plot([], [], label="underrun")
    (l_overflow,) = ax1.plot([], [], label="overflow")
    (l_sofmiss,) = ax1.plot([], [], label="sof miss (calls−updates)")
    ax1.set_ylabel("counts")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    (l_feedback,) = ax2.plot([], [], label="feedback (frames/ms)")
    ax2.set_ylabel("frames/ms")
    ax2.set_xlabel("seconds (relative)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    def update(_frame: int):
        nonlocal dev, last_connect_attempt, t0

        now = time.time()
        if dev is None and (now - last_connect_attempt) > 1.0:
            last_connect_attempt = now
            dev = open_device(vid, pid)
            if dev is None:
                print(f"Waiting for device... (vid=0x{vid:04x})")
                return (
                    l_buf_us,
                    l_buf_len,
                    l_underrun,
                    l_overflow,
                    l_sofmiss,
                    l_feedback,
                )
            print("Connected.")

        if dev is None:
            return (l_buf_us, l_buf_len, l_underrun, l_overflow, l_sofmiss, l_feedback)

        try:
            # Drain any pending reports; HID reports are pushed every ~200ms.
            d: Optional[Diag] = None
            for _ in range(8):
                report = dev.read(64)
                if not report:
                    break
                parsed = parse_report(bytes(report))
                if parsed is not None:
                    d = parsed
        except OSError as e:
            print(f"HID read error: {e}; will reconnect")
            try:
                dev.close()
            except Exception:
                pass
            dev = None
            return (l_buf_us, l_buf_len, l_underrun, l_overflow, l_sofmiss, l_feedback)

        if d is None:
            return (l_buf_us, l_buf_len, l_underrun, l_overflow, l_sofmiss, l_feedback)

        if t0 is None:
            t0 = d.t
        tr = d.t - t0

        ts.append(tr)
        buf_us.append(d.buf_us)
        buf_len.append(d.buf_len)
        underrun.append(d.underrun)
        overflow.append(d.overflow)
        # feedback is Q16.16 frames-per-ms; convert to a plain float
        feedback_q.append(d.feedback / 65536.0)
        sof_miss.append(d.sof_calls - d.sof_updates)

        if csv_f:
            csv_f.write(
                f"{tr:.6f},{d.buf_us},{d.buf_len},{d.underrun},{d.overflow},"
                f"{d.feedback / 65536.0:.6f},{d.sof_calls},{d.sof_updates},{int(d.streaming)}\n"
            )
            csv_f.flush()

        l_buf_us.set_data(ts, buf_us)
        l_buf_len.set_data(ts, buf_len)
        l_underrun.set_data(ts, underrun)
        l_overflow.set_data(ts, overflow)
        l_sofmiss.set_data(ts, sof_miss)
        l_feedback.set_data(ts, feedback_q)

        if ts:
            x_lo = max(0.0, ts[-1] - args.window)
            x_hi = ts[-1] + 1e-6
            for ax in (ax0, ax1, ax2):
                ax.set_xlim(x_lo, x_hi)

        for ax in (ax0, ax1, ax2):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        fig.suptitle(
            f"Beyondex USB audio diagnostics (HID) — "
            f"streaming={'YES' if d.streaming else 'no'}  "
            f"feedback={d.feedback / 65536.0:.4f} frames/ms  "
            f"underrun={d.underrun}  overflow={d.overflow}"
        )

        return (l_buf_us, l_buf_len, l_underrun, l_overflow, l_sofmiss, l_feedback)

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    try:
        plt.show()
    finally:
        if csv_f:
            csv_f.close()
        if dev is not None:
            try:
                dev.close()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
