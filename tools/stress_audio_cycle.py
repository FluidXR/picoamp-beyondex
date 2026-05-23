"""
Stress-test: cycle audio start/stop while polling diag. Stop on first wedge.

The smoke test of reproduce_idle_bug.py wedged the device after one full
start-idle-start-stop cycle. This script reproduces that pattern in tight loop
with logging detailed enough to identify which event preceded the wedge.

No PnP queries between polls (eliminates that variable). No long idle
(eliminates selective-suspend as the cause for now). Just rapid audio session
churn.
"""
from __future__ import annotations

import argparse
import ctypes
import math
import os
import struct
import subprocess
import sys
import tempfile
import time
import wave
from pathlib import Path

import libusb as _libusb_pkg
_libusb_root = (Path(_libusb_pkg.__file__).parent / "_platform" / "_windows" /
                ("x64" if ctypes.sizeof(ctypes.c_void_p) == 8 else "x86"))
os.environ["PATH"] = str(_libusb_root) + os.pathsep + os.environ.get("PATH", "")
import usb.core, usb.backend.libusb1  # noqa: E402
_backend = usb.backend.libusb1.get_backend(find_library=lambda x: str(_libusb_root / "libusb-1.0.dll"))

VID, PID = 0xCAFE, 0x4030
DIAG_REQ = 0x43
DIAG_SIZE = 34


def write_sine(path: Path, *, freq=440, secs=1.0, rate=48000, amp=0.2):
    n = int(secs * rate)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2); w.setsampwidth(2); w.setframerate(rate)
        for i in range(n):
            s = int(amp * 32767 * math.sin(2*math.pi*freq*i/rate))
            w.writeframesraw(struct.pack("<hh", s, s))


def start_audio(wav: Path, *, method="playlooping") -> subprocess.Popen:
    if method == "playlooping":
        ps = (f"$p = New-Object System.Media.SoundPlayer '{wav}'; "
              f"$p.PlayLooping(); while ($true) {{ Start-Sleep -Seconds 1 }}")
    elif method == "playsync":
        # Plays once and exits; useful to compare "graceful" vs killed.
        ps = f"$p = New-Object System.Media.SoundPlayer '{wav}'; $p.PlaySync()"
    else:
        raise ValueError(method)
    return subprocess.Popen(["powershell.exe", "-NoProfile", "-Command", ps],
                            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_audio(p: subprocess.Popen, *, graceful=False):
    if p.poll() is not None:
        return
    if graceful:
        # Sends Ctrl+Break to the process group; SoundPlayer doesn't trap it,
        # so this is effectively still abrupt. Documented for completeness.
        try:
            p.send_signal(subprocess.signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        except Exception:
            p.terminate()
    else:
        p.terminate()
    try:
        p.wait(timeout=2)
    except subprocess.TimeoutExpired:
        p.kill()


def poll_diag(dev, *, timeout_ms=1000) -> tuple[bool, dict]:
    try:
        r = dev.ctrl_transfer(0xC0, DIAG_REQ, 0, 0, DIAG_SIZE, timeout=timeout_ms)
        magic, underrun, overflow, buf_len, buf_us = struct.unpack_from("<IIIii", bytes(r))
        return True, dict(magic=magic, underrun=underrun, overflow=overflow,
                          buf_len=buf_len, buf_us=buf_us)
    except usb.core.USBError as e:
        return False, dict(error=str(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=20)
    ap.add_argument("--play-secs", type=float, default=2.0)
    ap.add_argument("--idle-secs", type=float, default=2.0)
    ap.add_argument("--polls-per-phase", type=int, default=3)
    args = ap.parse_args()

    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=_backend)
    if dev is None:
        sys.exit("device not found")

    wav = Path(tempfile.gettempdir()) / "beyondex_sine_440.wav"
    if not wav.exists():
        write_sine(wav)

    print(f"cycles={args.cycles}  play={args.play_secs}s  idle={args.idle_secs}s  polls/phase={args.polls_per_phase}")
    print(f"  format: event,cycle,phase,t_in_phase,result,buf_len,buf_us,underrun,overflow")

    t_global = time.time()
    def log(event, cycle, phase, t_in_phase, ok, d):
        t = time.time() - t_global
        if ok:
            print(f"  {event},{cycle},{phase},{t_in_phase:5.2f},ok,{d['buf_len']},{d['buf_us']},{d['underrun']},{d['overflow']}  // t={t:.2f}s")
        else:
            print(f"  {event},{cycle},{phase},{t_in_phase:5.2f},FAIL,{d.get('error')}  // t={t:.2f}s")
        return ok

    # Initial baseline diag (no audio).
    ok, d = poll_diag(dev)
    log("init", 0, "idle", 0, ok, d)
    if not ok:
        print("INITIAL DIAG FAILED — device already wedged at start; abort.")
        return 2

    for cyc in range(1, args.cycles + 1):
        # --- play phase ---
        p = start_audio(wav)
        t0 = time.time()
        for i in range(args.polls_per_phase):
            time.sleep(args.play_secs / args.polls_per_phase)
            ok, d = poll_diag(dev)
            log("play", cyc, "stream", time.time() - t0, ok, d)
            if not ok:
                print(f"\n*** WEDGE DETECTED during PLAY phase, cycle {cyc} ***")
                stop_audio(p)
                return 1
        stop_audio(p)

        # --- idle phase ---
        t0 = time.time()
        for i in range(args.polls_per_phase):
            time.sleep(args.idle_secs / args.polls_per_phase)
            ok, d = poll_diag(dev)
            log("idle", cyc, "silent", time.time() - t0, ok, d)
            if not ok:
                print(f"\n*** WEDGE DETECTED during IDLE phase, cycle {cyc} ***")
                return 1

    print("\n*** completed all cycles without wedge ***")
    return 0


if __name__ == "__main__":
    sys.exit(main())
