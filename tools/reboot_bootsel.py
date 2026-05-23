"""Trigger BOOTSEL reboot via vendor control request."""
import ctypes, os, sys, time
from pathlib import Path
import libusb as _libusb_pkg
_libusb_root = (Path(_libusb_pkg.__file__).parent / "_platform" / "_windows" /
                ("x64" if ctypes.sizeof(ctypes.c_void_p) == 8 else "x86"))
os.environ["PATH"] = str(_libusb_root) + os.pathsep + os.environ.get("PATH", "")
import usb.core, usb.backend.libusb1
_backend = usb.backend.libusb1.get_backend(find_library=lambda x: str(_libusb_root / "libusb-1.0.dll"))

dev = usb.core.find(idVendor=0xCAFE, idProduct=0x4030, backend=_backend)
if dev is None:
    sys.exit("device not found")

# BEYONDEX_USB_REQ_BOOTSEL = 0x42, BEYONDEX_USB_BOOTSEL_MAGIC = 0xB007
# bmRequestType = 0x40 (host-to-device | vendor | device)
try:
    dev.ctrl_transfer(0x40, 0x42, 0xB007, 0, b"", timeout=1000)
    print("BOOTSEL reboot requested; device should re-enumerate as RPI-RP2 mass storage")
except usb.core.USBError as e:
    print(f"control xfer error (may be expected if device rebooted before status stage): {e}")
