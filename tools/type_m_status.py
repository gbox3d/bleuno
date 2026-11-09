# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial==3.5"]
# ///
"""Read type_m diagnostic state without issuing actuator commands or a reset."""
import argparse
import time

import serial

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--port", required=True)
args = parser.parse_args()
connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
connection.dtr = False
connection.rts = False
connection.port = args.port
connection.open()
try:
    connection.reset_input_buffer()
    deadline = time.monotonic() + 3
    next_request = 0.0
    found = False
    while time.monotonic() < deadline:
        # Reopening USB CDC can lose its first write on Windows.
        if time.monotonic() >= next_request:
            connection.write(b"about\nstatus\n")
            next_request = time.monotonic() + 0.5
        line = connection.readline().decode("ascii", errors="replace").strip()
        if line:
            print(line, flush=True)
        if line.startswith("status "):
            found = True
            break
    if not found:
        raise SystemExit("No type_m status response within 3 seconds")
finally:
    connection.close()
