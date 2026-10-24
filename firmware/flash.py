#!/usr/bin/env python3
"""Flash the pendulum timer and open the serial log.

    python flash.py                  newest app.bin (here, next to this script, or Downloads)
    python flash.py path\\to\\app.bin  a specific image
    python flash.py --wifi wifi.txt   also write Wi-Fi credentials (0x9000)
    python flash.py --no-monitor      flash only

Writes the app only (0x10000), so the bootloader, partition table, saved Wi-Fi
and bob calibration are kept. Needs:  pip install esptool pyserial
"""
import argparse
import hashlib
import subprocess
import sys
import time
from pathlib import Path

APP_ADDR = "0x10000"
WIFI_ADDR = "0x9000"
BAUD = 115200
CP210X = (0x10C4, 0xEA60)  # the board's USB-serial chip


def die(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(1)


def find_image(arg):
    if arg:
        p = Path(arg).expanduser()
        if not p.is_file():
            die(f"{p} not found")
        return p
    here = Path(__file__).resolve().parent
    places = [Path.cwd(), here, here / "pendulum-timer", Path.home() / "Downloads"]
    found = [d / "app.bin" for d in places if (d / "app.bin").is_file()]
    if not found:
        die("no app.bin found in " + ", ".join(str(d) for d in places))
    return max(found, key=lambda p: p.stat().st_mtime)


def check_image(p):
    data = p.read_bytes()
    if not data or data[0] != 0xE9:
        die(f"{p} isn't an ESP32 app image (first byte should be 0xE9)")
    if len(data) > 0x3F0000:
        die(f"{p} is {len(data):,} bytes, too big for the app partition")
    age = time.time() - p.stat().st_mtime
    when = time.strftime("%Y-%m-%d %H:%M", time.localtime(p.stat().st_mtime))
    print(f"image : {p}")
    print(f"        {len(data):,} bytes, saved {when} ({age / 60:.0f} min ago), "
          f"sha256 {hashlib.sha256(data).hexdigest()[:12]}")


def find_port(arg):
    if arg:
        return arg
    try:
        from serial.tools import list_ports
    except ImportError:
        die("pyserial is missing:  pip install esptool pyserial")
    ports = list(list_ports.comports())
    cp = [p for p in ports if (p.vid, p.pid) == CP210X]
    if len(cp) == 1:
        return cp[0].device
    if not ports:
        die("no serial ports found. Is the board plugged in?")
    listing = "\n".join(f"  {p.device}  {p.description}" for p in ports)
    die(f"can't tell which port is the board. Pass --port, e.g. --port COM3\n{listing}")


def run(args):
    print(">", " ".join(args[1:] if args[0] == sys.executable else args))
    r = subprocess.run(args)
    if r.returncode != 0:
        die(f"command failed (exit {r.returncode})")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image", nargs="?", help="app.bin to flash (default: newest found)")
    ap.add_argument("--port", help="serial port (default: auto-detect the CP210x)")
    ap.add_argument("--wifi", metavar="FILE", help="also flash a wifi.txt at 0x9000")
    ap.add_argument("--no-monitor", action="store_true", help="don't open the serial log after")
    a = ap.parse_args()

    try:
        import esptool
    except ImportError:
        die("esptool is missing:  pip install esptool pyserial")
    # esptool 5 renamed write_flash to write-flash (the old name still works, with a warning)
    major = int(str(getattr(esptool, "__version__", "4")).split(".")[0] or 4)
    write = "write-flash" if major >= 5 else "write_flash"

    img = find_image(a.image)
    check_image(img)
    port = find_port(a.port)
    print(f"port  : {port}")

    cmd = [sys.executable, "-m", "esptool", "--chip", "esp32", "--port", port, write]
    if a.wifi:
        w = Path(a.wifi).expanduser()
        if not w.is_file():
            die(f"{w} not found")
        cmd += [WIFI_ADDR, str(w)]
    cmd += [APP_ADDR, str(img)]
    run(cmd)

    if not a.no_monitor:
        print("\nSerial log (Ctrl+] to quit)")
        subprocess.run([sys.executable, "-m", "serial.tools.miniterm", port, str(BAUD)])


if __name__ == "__main__":
    main()
