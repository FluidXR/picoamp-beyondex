"""
Poll Beyondex audio diag over WinUSB vendor control transfer.

Reads BEYONDEX_USB_REQ_AUDIO_DIAG (0x43) from MI_02 every second. Firmware
populates magic/underrun/overflow/buf_len/buf_us; other fields are zero on this
control-xfer path (see main.c:741-754).

Usage:
    python tools/diag_poll.py            # poll indefinitely
    python tools/diag_poll.py 30         # poll for 30 seconds
"""
import ctypes
import os
import struct
import sys
import time
from pathlib import Path

# Point pyusb at the libusb-1.0.dll shipped by the `libusb` PyPI package.
_libusb_root = Path(__file__).resolve().parent / "_libusb_bundled"
if not _libusb_root.exists():
    try:
        import libusb as _libusb_pkg
        _libusb_root = (
            Path(_libusb_pkg.__file__).parent / "_platform" / "_windows" /
            ("x64" if ctypes.sizeof(ctypes.c_void_p) == 8 else "x86")
        )
    except ImportError:
        sys.exit("install libusb pkg: pip install --user libusb")

os.environ["PATH"] = str(_libusb_root) + os.pathsep + os.environ.get("PATH", "")

import usb.core  # noqa: E402
import usb.backend.libusb1  # noqa: E402

_dll_path = _libusb_root / "libusb-1.0.dll"
backend = usb.backend.libusb1.get_backend(find_library=lambda x: str(_dll_path))
if backend is None:
    sys.exit(f"failed to load libusb backend from {_dll_path}")

VID, PID = 0xCAFE, 0x4030
BEYONDEX_USB_REQ_AUDIO_DIAG = 0x43
DIAG_SIZE = 34  # sizeof(beyondex_audio_diag_t) packed; firmware verifies wLength == this

dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
if dev is None:
    sys.exit(f"device {VID:04x}:{PID:04x} not found")

# bmRequestType = 0xC0 (IN | Vendor | Device)
duration = float(sys.argv[1]) if len(sys.argv) > 1 else float("inf")
t0 = time.time()
print("t_s,magic,underrun,overflow,buf_len,buf_us,power_state")
try:
    while time.time() - t0 < duration:
        try:
            r = dev.ctrl_transfer(0xC0, BEYONDEX_USB_REQ_AUDIO_DIAG, 0, 0, DIAG_SIZE, timeout=1000)
            magic, underrun, overflow, buf_len, buf_us = struct.unpack_from("<IIIii", bytes(r))
            print(f"{time.time() - t0:.3f},0x{magic:08x},{underrun},{overflow},{buf_len},{buf_us}")
        except usb.core.USBError as e:
            print(f"{time.time() - t0:.3f},USBError,{e}")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
