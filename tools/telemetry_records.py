"""Telemetry record layouts of TEST_PLAN.md section 4.0, shared by the tools."""

import csv
import re

import numpy as np

COLUMNS = {
    "R": ["seq", "dt_us", "ax_mm_s2", "ay_mm_s2", "az_mm_s2",
          "gx_urad_s", "gy_urad_s", "gz_urad_s", "mx_ug", "my_ug", "mz_ug", "mag_valid"],
    "S": ["seq", "fax_mm_s2", "fay_mm_s2", "faz_mm_s2",
          "wax_urad_s", "way_urad_s", "waz_urad_s", "mmx_ug", "mmy_ug", "mmz_ug", "mag_valid"],
    "T": ["seq", "t_ms", "state", "roll_mdeg", "pitch_mdeg", "yaw_mdeg",
          "q0_e6", "q1_e6", "q2_e6", "q3_e6", "bgx_urad_s", "bgy_urad_s", "bgz_urad_s",
          "acc_soft", "acc_gated", "acc_rejected", "mag_gated", "mag_rejected",
          "yaw_resets", "tilt_resets", "gyro_sat", "numeric_faults", "dt_fault", "msgq_drop",
          "ekf_cyc", "lat_cyc", "flags"],
}

MODE_OF_TYPE = {"R": "raw", "S": "sensor", "T": "fusion"}

# The shell redraws its prompt around every record, so a record may be preceded
# by escape sequences and prompt text on the same line.
_ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
_RECORD = re.compile(r"(?:^|[^A-Za-z0-9,])([RST])((?:,-?\d+)+)\s*$")


def parse_line(line):
    """Return (record type, list of ints) for a telemetry line, or None."""
    match = _RECORD.search(_ANSI_ESCAPE.sub("", line))
    if match is None:
        return None

    record_type = match.group(1)
    values = [int(field) for field in match.group(2).split(",")[1:]]
    if len(values) != len(COLUMNS[record_type]):
        return None

    return record_type, values


def load_csv(path, record_type):
    """Load a capture.py CSV into a dict of float64 arrays keyed by column name."""
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        if header != COLUMNS[record_type]:
            raise ValueError(f"{path} is not a type {record_type} capture")
        rows = [[int(field) for field in row] for row in reader if row]

    if not rows:
        raise ValueError(f"{path} holds no records")

    table = np.array(rows, dtype=np.float64)
    return {name: table[:, index] for index, name in enumerate(header)}
