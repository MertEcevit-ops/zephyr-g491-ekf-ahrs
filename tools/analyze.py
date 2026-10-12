#!/usr/bin/env python3
"""Evaluate telemetry captures against the thresholds of TEST_PLAN.md.

Every sub-command prints the measured values with PASS/FAIL per criterion and
exits non-zero when a criterion fails. `--report FILE.md` also writes the
result as a Markdown table for docs/evidence/.
"""

import argparse
import sys

import numpy as np

from telemetry_records import load_csv

G0 = 9.80665
CYCLES_PER_S = 170.0e6
# 104 Hz setting of the ISM330DHCX: 6667 Hz / 64. The real rate follows the
# chip's own oscillator and differs per board (INTERNAL_FREQ_FINE).
NOMINAL_DT_US = 64.0e6 / 6667.0

# Sensor frame to body frame, MATH_SPEC.md section 2.6.
ISM_TO_BODY = np.array([1.0, -1.0, -1.0])
MMC_TO_BODY = np.array([1.0, -1.0, 1.0])


class Report:
    """Collects criteria; prints them and optionally writes a Markdown file."""

    def __init__(self, title):
        self.title = title
        self.rows = []

    def check(self, name, value, passed, limit):
        self.rows.append((name, value, limit, "PASS" if passed else "FAIL"))

    def info(self, name, value):
        self.rows.append((name, value, "-", "INFO"))

    @property
    def passed(self):
        return all(row[3] != "FAIL" for row in self.rows)

    def finish(self, path):
        verdict = "PASS" if self.passed else "FAIL"
        print(f"{self.title}: {verdict}")
        for name, value, limit, result in self.rows:
            print(f"  {result:4}  {name}: {value} (limit {limit})")

        if path is not None:
            with open(path, "w") as out:
                out.write(f"# {self.title}: {verdict}\n\n")
                out.write("| Criterion | Measured | Limit | Result |\n|---|---|---|---|\n")
                for row in self.rows:
                    out.write("| " + " | ".join(str(field) for field in row) + " |\n")

        return 0 if self.passed else 1


def unwrap_deg(angle_mdeg):
    return np.degrees(np.unwrap(np.radians(angle_mdeg / 1000.0)))


def attitude_deg(capture):
    return (capture["roll_mdeg"] / 1000.0, capture["pitch_mdeg"] / 1000.0,
            unwrap_deg(capture["yaw_mdeg"]))


def analyze_static(args):
    """T-DYN-01 (noise) and, for captures of 2 minutes or more, T-DYN-02/03 (drift)."""
    capture = load_csv(args.capture, "T")
    seconds = (capture["t_ms"] - capture["t_ms"][0]) / 1000.0
    roll, pitch, yaw = attitude_deg(capture)
    report = Report(f"Static attitude, {seconds[-1]:.0f} s")

    for name, series, limit in (("roll", roll, 0.05), ("pitch", pitch, 0.05), ("yaw", yaw, 0.2)):
        std = float(np.std(series))
        report.check(f"std {name} [deg]", f"{std:.4f}", std < limit, f"< {limit}")

    if seconds[-1] >= 120.0:
        first, last = seconds < 60.0, seconds >= seconds[-1] - 60.0
        yaw_limit = 5.0 if args.mag_off else 1.0
        for name, series, limit in (("roll", roll, 0.2), ("pitch", pitch, 0.2),
                                    ("yaw", yaw, yaw_limit)):
            drift = abs(float(np.mean(series[last]) - np.mean(series[first])))
            report.check(f"drift {name}, first vs last minute [deg]", f"{drift:.3f}",
                         drift < limit, f"< {limit}")

    return report.finish(args.report)


def split_at_rest(vectors, min_samples):
    """Mean vector of every stretch where the readings hold still."""
    step = np.linalg.norm(np.diff(vectors, axis=0), axis=1)
    moving = np.concatenate([[False], step > 5.0 * np.median(step) + 1.0e-9])
    means, start = [], 0
    for index in range(1, len(vectors) + 1):
        if index == len(vectors) or moving[index]:
            if index - start >= min_samples:
                means.append(vectors[start:index].mean(axis=0))
            start = index + 1
    return np.array(means)


def analyze_six_face(args):
    """T-RAW-12: accelerometer offset and scale from six faces (MATH_SPEC.md 3.3)."""
    capture = load_csv(args.capture, "S")
    accel = np.column_stack([capture["fax_mm_s2"], capture["fay_mm_s2"],
                             capture["faz_mm_s2"]]) / 1000.0
    faces = split_at_rest(accel, min_samples=args.min_samples)
    report = Report("Accelerometer six-face calibration")

    offset, scale = np.zeros(3), np.ones(3)
    for axis, name in enumerate("xyz"):
        up = [face for face in faces if face[axis] > 0.9 * G0]
        down = [face for face in faces if face[axis] < -0.9 * G0]
        report.check(f"faces found for +{name}/-{name}", f"{len(up)}/{len(down)}",
                     len(up) >= 1 and len(down) >= 1, ">= 1 each")
        if up and down:
            plus, minus = np.mean(up, axis=0)[axis], np.mean(down, axis=0)[axis]
            offset[axis] = (plus + minus) / 2.0
            scale[axis] = 2.0 * G0 / (plus - minus)

    for face in faces:
        residual = abs(np.linalg.norm(scale * (face - offset)) - G0) / G0
        report.check(f"calibrated norm error, face {np.round(face, 2)}", f"{residual:.4f}",
                     residual < 0.02, "< 0.02")

    report.info("ACCEL_CAL_OFFSET [m/s^2]", ", ".join(f"{value:.5f}f" for value in offset))
    report.info("ACCEL_CAL_SCALE", ", ".join(f"{value:.6f}f" for value in scale))
    return report.finish(args.report)


def analyze_gyro_turn(args):
    """T-RAW-09: integral of one body rate over a +90 degree turn and back."""
    capture = load_csv(args.capture, "R")
    axis_column = {"x": "gx_urad_s", "y": "gy_urad_s", "z": "gz_urad_s"}[args.axis]
    rate = capture[axis_column] * 1.0e-6
    # Telemetry is decimated: records are seq samples apart, and dt_us is the
    # measured interval of one sample. No output data rate is assumed.
    sample_period_s = float(np.mean(capture["dt_us"][capture["dt_us"] > 0])) * 1.0e-6
    seconds = (capture["seq"] - capture["seq"][0]) * sample_period_s
    # The bias would integrate into the angle; the first second of the capture is at rest.
    rate = rate - np.mean(rate[seconds < 1.0])
    spacing_s = np.diff(seconds)
    angle = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * spacing_s)])
    peak = float(angle[np.argmax(np.abs(angle))])

    report = Report(f"Gyro turn about body {args.axis}")
    report.info("measured sample period [us]", f"{sample_period_s * 1e6:.1f}")
    report.check("peak angle [rad]", f"{peak:+.4f}", abs(peak - np.pi / 2) <= 0.05,
                 "+1.571 +- 0.05")
    return report.finish(args.report)


def mean_mag_sensor(path):
    capture = load_csv(path, "S")
    valid = capture["mag_valid"] != 0
    return np.array([capture[name][valid].mean() for name in ("mmx_ug", "mmy_ug", "mmz_ug")])


def mean_accel_sensor(path):
    capture = load_csv(path, "S")
    return np.array([capture[name].mean() for name in ("fax_mm_s2", "fay_mm_s2", "faz_mm_s2")])


def analyze_mag_sign(args):
    """T-RAW-10, magnetometer: differences cancel the hard-iron offset."""
    # The north capture doubles as the flat one: the inverted pose also points X north.
    sensor = {name: mean_mag_sensor(path) for name, path in
              (("north", args.north), ("east", args.east), ("south", args.south),
               ("west", args.west), ("flat", args.north), ("inverted", args.inverted))}
    body = {name: MMC_TO_BODY * value for name, value in sensor.items()}
    report = Report("Magnetometer axis signs")

    checks = (
        ("m_b,x(N) - m_b,x(S) [uG]", body["north"][0] - body["south"][0], 1),
        ("m_b,y(E) - m_b,y(W) [uG]", body["east"][1] - body["west"][1], -1),
        ("m_M,y(E) - m_M,y(W) [uG]", sensor["east"][1] - sensor["west"][1], 1),
        ("m_b,z(flat) - m_b,z(inverted) [uG]", body["flat"][2] - body["inverted"][2], 1),
    )
    for name, difference, expected_sign in checks:
        report.check(name, f"{difference:+.0f}", difference * expected_sign > 0.0,
                     "> 0" if expected_sign > 0 else "< 0")
    return report.finish(args.report)


def analyze_accel_sign(args):
    """T-RAW-10, accelerometer: the axis pointing up reads +g (MATH_SPEC.md 2.8)."""
    expected_body = {"flat": (0, 0, -1), "inverted": (0, 0, 1), "nose_up": (1, 0, 0),
                     "right_up": (0, 1, 0)}
    report = Report("Accelerometer axis signs")

    for pose, path in (("flat", args.flat), ("inverted", args.inverted),
                       ("nose_up", args.nose_up), ("right_up", args.right_up)):
        body_g = ISM_TO_BODY * mean_accel_sensor(path) / 1000.0 / G0
        expected = np.array(expected_body[pose], dtype=float)
        dominant = float(body_g @ expected)
        others = float(np.max(np.abs(body_g - dominant * expected)))
        report.check(f"{pose}: f_b [g]", np.round(body_g, 3), dominant > 0.9 and others < 0.15,
                     f"{expected_body[pose]}: > 0.9 g, others < 0.15 g")
    return report.finish(args.report)


def analyze_step(args):
    """T-DYN-06: 90 degree yaw step, final value, overshoot and settling time."""
    capture = load_csv(args.capture, "T")
    seconds = (capture["t_ms"] - capture["t_ms"][0]) / 1000.0
    yaw = unwrap_deg(capture["yaw_mdeg"])
    yaw = yaw - np.mean(yaw[seconds < 1.0])
    final = float(np.mean(yaw[seconds >= seconds[-1] - 1.0]))
    direction = 1.0 if final >= 0.0 else -1.0

    rate = np.abs(np.gradient(np.radians(yaw), seconds))
    moving = np.nonzero(rate > 0.05)[0]
    motion_end = seconds[moving[-1]] if len(moving) else seconds[0]
    outside = np.nonzero(np.abs(yaw - final) > 1.0)[0]
    settled_at = seconds[min(outside[-1] + 1, len(seconds) - 1)] if len(outside) else seconds[0]
    settle_time = max(0.0, float(settled_at - motion_end))
    overshoot = max(0.0, float(np.max(direction * (yaw - final))))

    report = Report("Yaw step response")
    report.check("final value [deg]", f"{final:+.2f}", abs(abs(final) - 90.0) <= 2.0, "90 +- 2")
    report.check("settling to +-1 deg after motion end [s]", f"{settle_time:.2f}",
                 settle_time < 0.5, "< 0.5")
    report.check("overshoot [deg]", f"{overshoot:.2f}", overshoot < 1.0, "< 1.0")
    return report.finish(args.report)


def timing_from_raw(capture, report, expected_dt_us):
    """T-RAW-07 on a type R capture; dt_us is per sample even when records are decimated."""
    dt_us = capture["dt_us"][capture["dt_us"] > 0]
    mean, std = float(np.mean(dt_us)), float(np.std(dt_us))
    report.check("mean dt [us]", f"{mean:.1f}", abs(mean - expected_dt_us) <= 0.02 * expected_dt_us,
                 f"{expected_dt_us:.0f} +- 2 %")
    report.check("std dt [us]", f"{std:.1f}", std < 50.0, "< 50")


def timing_from_fusion(capture, report):
    """T-DYN-07 latency and the timing and fault part of T-DYN-12 on a type T capture."""
    running = capture["state"] == 1
    ekf_p99 = float(np.percentile(capture["ekf_cyc"][running], 99))
    latency_p99 = float(np.percentile(capture["lat_cyc"][running], 99))
    report.check("ekf_cyc p99 [us]", f"{ekf_p99 / CYCLES_PER_S * 1e6:.0f}", ekf_p99 < 170000,
                 "< 1000")
    report.check("lat_cyc p99 [us]", f"{latency_p99 / CYCLES_PER_S * 1e6:.0f}",
                 latency_p99 < 340000, "< 2000")
    for counter in ("msgq_drop", "dt_fault", "numeric_faults"):
        increase = int(capture[counter][-1] - capture[counter][0])
        report.check(f"{counter} increase", increase, increase == 0, "0")


def analyze_timing(args):
    report = Report(f"Timing, type {args.type} capture")
    capture = load_csv(args.capture, args.type)
    if args.type == "R":
        timing_from_raw(capture, report, args.expected_dt_us)
    else:
        timing_from_fusion(capture, report)
    return report.finish(args.report)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)

    def add_command(name, handler, summary):
        command = commands.add_parser(name, help=summary)
        command.add_argument("--report", help="Markdown file to write")
        command.set_defaults(handler=handler)
        return command

    static = add_command("static", analyze_static, "T-DYN-01/02/03 on a type T capture")
    static.add_argument("capture")
    static.add_argument("--mag-off", action="store_true", help="T-DYN-03 yaw drift limit")

    six_face = add_command("six_face", analyze_six_face, "T-RAW-12 on a type S capture")
    six_face.add_argument("capture")
    six_face.add_argument("--min-samples", type=int, default=40,
                          help="shortest stretch at rest that counts as a face")

    gyro_turn = add_command("gyro_turn", analyze_gyro_turn, "T-RAW-09 on a type R capture")
    gyro_turn.add_argument("capture")
    gyro_turn.add_argument("--axis", choices="xyz", required=True)

    mag_sign = add_command("mag_sign", analyze_mag_sign, "T-RAW-10 magnetometer, type S captures")
    for pose in ("north", "east", "south", "west", "inverted"):
        mag_sign.add_argument(f"--{pose}", required=True, help=f"capture with the board {pose}")

    accel_sign = add_command("accel_sign", analyze_accel_sign,
                             "T-RAW-10 accelerometer, type S captures")
    for pose in ("flat", "inverted", "nose-up", "right-up"):
        accel_sign.add_argument(f"--{pose}", required=True, help=f"capture with the board {pose}")

    step = add_command("step", analyze_step, "T-DYN-06 on a type T capture")
    step.add_argument("capture")

    timing = add_command("timing", analyze_timing, "T-RAW-07 (type R) or T-DYN-07/12 (type T)")
    timing.add_argument("capture")
    timing.add_argument("--type", choices="RT", default="T")
    timing.add_argument("--expected-dt-us", type=float, default=NOMINAL_DT_US,
                        help="sample period this chip should have (type R); the bus_id "
                             "drivers test prints it from INTERNAL_FREQ_FINE")

    return parser


def main():
    args = build_parser().parse_args()
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
