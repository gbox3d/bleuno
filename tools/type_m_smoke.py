# /// script
# requires-python = ">=3.11"
# dependencies = ["bleak==3.0.1", "pyserial==3.5"]
# ///
"""BLE integration checks that never request motor movement or servo pulses.

Run with an already installed Python:
  uv run --no-python-downloads --python 3.12 tools/type_m_smoke.py --address XX:XX:XX:XX:XX:XX
"""
import argparse
import asyncio
import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from bleak import BleakClient, BleakScanner
import serial

SERVICE = "c6f8b088-2af8-4388-8364-ca2a907bdeb8"
CHARACTERISTIC = "f6aa83ca-de53-46b4-bdea-28a7cb57942e"


def serial_status(port):
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    try:
        connection.reset_input_buffer()
        deadline = time.monotonic() + 5
        next_request = 0.0
        while time.monotonic() < deadline:
            # Windows USB CDC may drop the first write while reopening a port.
            # Repeat this read-only request instead of resetting the board.
            if time.monotonic() >= next_request:
                connection.write(b"status\n")
                next_request = time.monotonic() + 0.5
            line = connection.readline().decode("ascii", errors="replace").strip()
            if line.startswith("status "):
                return dict(field.split("=", 1) for field in line.split()[1:])
        raise RuntimeError("No serial status response")
    finally:
        connection.close()


async def run(args):
    report = {"time": datetime.now(timezone.utc).isoformat(), "checks": [],
              "movement_requested": False, "address": args.address}
    notifications = asyncio.Queue()

    def record(name, actual, expected):
        passed = actual == expected
        report["checks"].append({"name": name, "actual": actual,
                                  "expected": expected, "passed": passed})
        print(f"{'PASS' if passed else 'FAIL'} {name}: {actual}", flush=True)
        if not passed:
            raise AssertionError(f"{name}: expected {expected!r}, got {actual!r}")

    def notified(_sender, data):
        notifications.put_nowait(bytes(data).decode("ascii", errors="replace"))

    async def exchange(client, command, expected=None, subscribed=True):
        while not notifications.empty():
            notifications.get_nowait()
        await client.write_gatt_char(CHARACTERISTIC, command.encode("ascii"), response=True)
        if subscribed:
            reply = await asyncio.wait_for(notifications.get(), timeout=3)
        else:
            # The firmware dispatches received commands on its control task.
            await asyncio.sleep(0.1)
            reply = bytes(await client.read_gatt_char(CHARACTERISTIC)).decode("ascii")
        if expected is not None:
            record(command, reply, expected)
        return reply

    async def sensor(client, model, require_value=False):
        reply = await exchange(client, "dht")
        valid = False
        match = re.fullmatch(r"dht (11|22) (-?\d+) (\d+)", reply)
        if match:
            actual_model, temperature, humidity = map(int, match.groups())
            valid = actual_model == model and -400 <= temperature <= 800 and 0 <= humidity <= 1000
        elif not require_value:
            valid = reply in (f"dht {model} wait", f"dht {model} error")
        report.setdefault("sensor_responses", []).append(reply)
        record(f"DHT{model} response {reply}", valid, True)
        return reply

    async def wait_with_heartbeat(client, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            await exchange(client, "ping", "ok ping")
            await asyncio.sleep(0.4)

    try:
        device = await BleakScanner.find_device_by_address(args.address, timeout=15)
        if device is None:
            raise RuntimeError("Requested board was not discovered; check power/advertising")
        report["name"] = device.name
        async with BleakClient(device, timeout=15) as client:
            if client.services.get_service(SERVICE) is None:
                raise AssertionError("type_m service missing")
            await exchange(client, "about", "type_m/1 fw=1.1.0", False)
            await exchange(client, "ready", "err subscribe", False)
            await client.start_notify(CHARACTERISTIC, notified)
            await exchange(client, "motor 1 0", "err not_ready")
            await exchange(client, "dht", "err not_ready")
            await exchange(client, "dht 22", "err not_ready")
            await exchange(client, "ready", "ok ready")
            await exchange(client, "ping", "ok ping")
            await exchange(client, "motor 1 101", "err range")
            await exchange(client, "servo 1 181", "err range")
            await exchange(client, "motor 0 0", "err range")
            await exchange(client, "motor 1 x", "err syntax")
            await exchange(client, "dht 12", "err range")
            await exchange(client, "dht x", "err syntax")
            await exchange(client, "dht 11 22", "err syntax")
            # Switching model invalidates the previous cached measurement.
            for model in (22, 11):
                await exchange(client, f"dht {model}", f"ok dht {model}")
                await exchange(client, "dht", f"dht {model} wait")
                await wait_with_heartbeat(client, 3.0)
                await sensor(client, model)
                await exchange(client, "ping", "ok ping")
            if args.dht_type != 11:
                await exchange(client, f"dht {args.dht_type}", f"ok dht {args.dht_type}")
                await wait_with_heartbeat(client, 3.0)
            if args.expect_sensor:
                await sensor(client, args.dht_type, require_value=True)
            await exchange(client, "motor 1 0", "ok motor 1 0")
            await exchange(client, "motor 2 0", "ok motor 2 0")
            await exchange(client, "stop", "ok stop")
            await exchange(client, "ping", "ok ping")
            event = await asyncio.wait_for(notifications.get(), timeout=3)
            record("watchdog", event, "event timeout")
            await exchange(client, "motor 1 0", "err not_ready")
            await exchange(client, "ready", "ok ready")
            await client.stop_notify(CHARACTERISTIC)
            await asyncio.sleep(0.1)
            await client.start_notify(CHARACTERISTIC, notified)
            await exchange(client, "ping", "err not_ready")
            await exchange(client, "ready", "ok ready")
            await exchange(client, "stop", "ok stop")
        # Windows can retain the physical link after BleakClient.disconnect().
        # Verify the peripheral's disconnect, not merely a disposed client object.
        if args.serial_port:
            deadline = time.monotonic() + 15
            while True:
                state = await asyncio.to_thread(serial_status, args.serial_port)
                if state["link"] == "0":
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Physical BLE link did not disconnect within 15s")
                await asyncio.sleep(0.3)
            for field, expected in {"ready": "0", "motor": "0,0", "pwm": "0,0,0,0",
                                    "servo": "-1,-1", "servoDuty": "0,0", "sleep": "0"}.items():
                record("disconnected " + field, state[field], expected)
        else:
            # Allow the OS to release its GATT references before a fresh scan.
            await asyncio.sleep(5)
        device = await BleakScanner.find_device_by_address(args.address, timeout=15)
        if device is None:
            raise RuntimeError("Board did not advertise after disconnect")
        async with BleakClient(device, timeout=15) as client:
            await client.start_notify(CHARACTERISTIC, notified)
            await exchange(client, "motor 1 0", "err not_ready")
            await exchange(client, "about", "type_m/1 fw=1.1.0")
            await exchange(client, "ready", "ok ready")
            await exchange(client, "stop", "ok stop")
        report["passed"] = True
    except Exception as error:
        report["passed"] = False
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", required=True, help="Explicit board BLE address")
    parser.add_argument("--output", default=".local/type_m/ble-smoke.json")
    parser.add_argument("--serial-port", help="Verify the board's physical disconnect and safe output state")
    parser.add_argument("--dht-type", type=int, choices=(11, 22), default=11, help="Leave this DHT model selected")
    parser.add_argument("--expect-sensor", action="store_true", help="Require a real, valid measurement for the selected sensor")
    asyncio.run(run(parser.parse_args()))
