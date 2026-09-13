#!/usr/bin/env python3
"""Drive the firmware's serial console non-interactively.

    scripts/console.py x            run the particle-sweep benchmark
    scripts/console.py g            run the golden determinism sequence (~30s)
    scripts/console.py r i          several commands in order

Exists because the interesting console commands block for tens of seconds and then stop producing
output without printing a terminator, so `pio device monitor` cannot be scripted against them.
This reads until the port goes quiet instead.

The port defaults to the first /dev/cu.usbmodem*; override with PARTSIM_PORT.
"""
import glob
import os
import sys
import time

try:
    import serial  # pyserial; ships inside the PlatformIO venv
except ImportError:
    sys.exit("pyserial not found -- try ~/.platformio/penv/bin/python scripts/console.py")

BAUD = 115200
# The board reboots when the port opens (DTR), so nothing said before this is heard.
BOOT_SETTLE_S = 2.0
# `g` and `x` each run for tens of seconds with long silent stretches mid-command -- `g` blocks the
# console for ~30 s before its FIRST line, so a short window returns nothing at all and looks like
# the board is dead.
QUIET_READS_TO_STOP = 45
COMMAND_TIMEOUT_S = 300


def find_port():
    p = os.environ.get("PARTSIM_PORT")
    if p:
        return p
    # cu.usbserial-* and cu.SLAB_USBtoUART are the CP2102/CH340 bridges on a plain ESP32 devkit,
    # which has no native USB at all -- see MINI.md.
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.wchusbserial*") +
                   glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB_USBtoUART*"))
    if not ports:
        sys.exit("no board found (looked for /dev/cu.usbmodem*); set PARTSIM_PORT")
    return ports[0]


def main(commands):
    port = find_port()
    print(f"# {port} @ {BAUD}", file=sys.stderr)
    with serial.Serial(port, BAUD, timeout=1) as s:
        time.sleep(BOOT_SETTLE_S)
        s.reset_input_buffer()
        for cmd in commands:
            s.write((cmd + "\n").encode())
            s.flush()
            deadline = time.time() + COMMAND_TIMEOUT_S
            quiet = 0
            while time.time() < deadline:
                line = s.readline()
                if not line:
                    quiet += 1
                    if quiet > QUIET_READS_TO_STOP:
                        break
                    continue
                quiet = 0
                sys.stdout.write(line.decode("utf-8", "replace"))
                sys.stdout.flush()


if __name__ == "__main__":
    main(sys.argv[1:] or ["?"])
