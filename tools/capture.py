#!/usr/bin/env python3
"""Record telemetry records (TEST_PLAN.md section 4.0) from the serial port into a CSV file."""

import argparse
import csv
import sys
import time

import serial

from telemetry_records import COLUMNS, MODE_OF_TYPE, parse_line


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--type", choices=sorted(COLUMNS), default="T", help="record type to keep")
    parser.add_argument("--duration", type=float, required=True, help="seconds to record")
    parser.add_argument("--rate", type=int, help="send `telemetry rate` first (1..50 Hz)")
    parser.add_argument("--keep-mode", action="store_true",
                        help="do not switch the firmware to the matching telemetry mode")
    parser.add_argument("--out", required=True, help="CSV file to write")
    return parser.parse_args()


def send_command(port, command):
    port.write(f"\r{command}\r".encode())
    port.flush()
    # The shell echoes at 115200 baud; give it time before the next command.
    time.sleep(0.2)


def record(port, record_type, duration_s, writer):
    deadline = time.monotonic() + duration_s
    kept = 0

    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace")
        parsed = parse_line(line)
        if parsed is not None and parsed[0] == record_type:
            writer.writerow(parsed[1])
            kept += 1

    return kept


def main():
    args = parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.5) as port, \
         open(args.out, "w", newline="") as out_file:
        if args.rate is not None:
            send_command(port, f"telemetry rate {args.rate}")
        if not args.keep_mode:
            send_command(port, f"telemetry mode {MODE_OF_TYPE[args.type]}")
        port.reset_input_buffer()

        writer = csv.writer(out_file)
        writer.writerow(COLUMNS[args.type])
        kept = record(port, args.type, args.duration, writer)

    print(f"{kept} type {args.type} records written to {args.out}")
    return 0 if kept > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
