#!/usr/bin/env python3
"""Float64 reference of the attitude EKF in MATH_SPEC.md.

`vectors` reproduces the reference vectors V1..V9 of section 15 and compares
them with the values printed in the specification. `scenarios` runs the
simulated cases of TEST_PLAN.md section 3.3 (T-EKF-06..12) with the same
deterministic noise generator as app/tests/ekf/src/sim.c and checks them
against the same thresholds.
"""

import argparse
import math
import sys

import numpy as np

# MATH_SPEC.md section 14
G0 = 9.80665
DT = 1.0 / 104.0
GYRO_ARW = 1.0e-3
GYRO_BIAS_RW = 2.0e-5
ACC_SIGMA = 0.03
HEADING_SIGMA = 0.05
P0_QUAT_SIGMA = 0.05
P0_BIAS_SIGMA_STATIC = 1.745e-3
P0_BIAS_SIGMA_MOVING = 5.236e-2
ACC_GATE_SOFT = 0.05
ACC_GATE_HARD = 0.20
ACC_R_INFLATE_MAX = 100.0
ACC_NIS_MAX = 11.34
MAG_NIS_MAX = 6.63
MAG_NORM_GATE = 0.15
MAG_DIP_GATE = 0.17453
MAG_MIN_COS2_PITCH = 0.0302
YAW_RESET_REJECTS = 312
MAG_CONVERGE_ACCEPTS = 104
MAG_DISTRUST_REJECTS = 3120
TILT_RESET_REJECTS = 520
GYRO_SAT = 8.29
ALIGN_SAMPLES = 100
QEXP_SMALL_ANGLE = 1.0e-3

# TEST_PLAN.md section 3.1
FIELD_N = np.array([0.25, 0.0, 0.40])
FIELD_NORM = 0.4717
FIELD_DIP = math.radians(57.99)
TRUE_BIAS = np.array([0.01, -0.02, 0.015])
GYRO_NOISE = 6.3e-4
ACCEL_NOISE = 4.2e-3
MAG_NOISE = 2.0e-3
NOISE_SEED = 0x12345678


def quat_multiply(p, r):
    return np.array([
        p[0] * r[0] - p[1] * r[1] - p[2] * r[2] - p[3] * r[3],
        p[0] * r[1] + p[1] * r[0] + p[2] * r[3] - p[3] * r[2],
        p[0] * r[2] - p[1] * r[3] + p[2] * r[0] + p[3] * r[1],
        p[0] * r[3] + p[1] * r[2] - p[2] * r[1] + p[3] * r[0],
    ])


def quat_from_rotation_vector(rotvec):
    angle = np.linalg.norm(rotvec)
    if angle < QEXP_SMALL_ANGLE:
        return np.concatenate([[1.0 - angle**2 / 8.0], (0.5 - angle**2 / 48.0) * rotvec])
    return np.concatenate([[math.cos(angle / 2.0)], math.sin(angle / 2.0) / angle * rotvec])


def rotation_matrix(q):
    q0, q1, q2, q3 = q
    return np.array([
        [q0*q0 + q1*q1 - q2*q2 - q3*q3, 2 * (q1*q2 - q0*q3), 2 * (q1*q3 + q0*q2)],
        [2 * (q1*q2 + q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2 * (q2*q3 - q0*q1)],
        [2 * (q1*q3 - q0*q2), 2 * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3],
    ])


def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q):
    q0, q1, q2, q3 = q
    sin_pitch = min(1.0, max(-1.0, 2 * (q0*q2 - q1*q3)))
    return np.array([
        math.atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)),
        math.asin(sin_pitch),
        math.atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3)),
    ])


def omega_matrix(w):
    return np.array([
        [0.0, -w[0], -w[1], -w[2]],
        [w[0], 0.0, w[2], -w[1]],
        [w[1], -w[2], 0.0, w[0]],
        [w[2], w[1], -w[0], 0.0],
    ])


def xi_matrix(q):
    return np.array([
        [-q[1], -q[2], -q[3]],
        [q[0], -q[3], q[2]],
        [q[3], q[0], -q[1]],
        [-q[2], q[1], q[0]],
    ])


def gravity_direction(q):
    """Predicted specific-force direction at rest and its Jacobian (section 8)."""
    q0, q1, q2, q3 = q
    predicted = -np.array([2 * (q1*q3 - q0*q2), 2 * (q2*q3 + q0*q1),
                           q0*q0 - q1*q1 - q2*q2 + q3*q3])
    jacobian = -2.0 * np.array([
        [-q2, q3, -q0, q1],
        [q1, q0, q3, q2],
        [q0, -q1, -q2, q3],
    ])
    return predicted, jacobian


def heading_jacobian(q):
    """d(yaw)/dq and cos^2(pitch) (section 9)."""
    q0, q1, q2, q3 = q
    a = 2 * (q0*q3 + q1*q2)
    b = 1 - 2 * (q2*q2 + q3*q3)
    cos2_pitch = a*a + b*b
    jacobian = np.array([2*b*q3, 2*b*q2, 2*b*q1 + 4*a*q2, 2*b*q0 + 4*a*q3]) / cos2_pitch
    return jacobian, cos2_pitch


def wrap_pi(angle):
    wrapped = math.fmod(angle, 2 * math.pi)
    if wrapped > math.pi:
        wrapped -= 2 * math.pi
    elif wrapped <= -math.pi:
        wrapped += 2 * math.pi
    return wrapped


def tilt_from_accel(accel):
    return (math.atan2(-accel[1], -accel[2]),
            math.atan2(accel[0], math.hypot(accel[1], accel[2])))


def align(accel_mean, mag_mean, mag_ok, declination):
    """Initial attitude from averaged readings (section 6)."""
    roll, pitch = tilt_from_accel(accel_mean)
    yaw = 0.0
    if mag_ok:
        sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
        level_x = cp * mag_mean[0] + sp * (sr * mag_mean[1] + cr * mag_mean[2])
        level_y = cr * mag_mean[1] - sr * mag_mean[2]
        yaw = wrap_pi(declination + math.atan2(-level_y, level_x))
    return quat_from_euler(roll, pitch, yaw)


class Ekf:
    """Sections 7 to 10, statement for statement like app/src/ekf/ekf_core.c."""

    def __init__(self, q0, bias0, stationary, declination=0.0, inclination=FIELD_DIP,
                 field_norm=FIELD_NORM, use_mag=True):
        bias_sigma = P0_BIAS_SIGMA_STATIC if stationary else P0_BIAS_SIGMA_MOVING
        self.q = np.array(q0, dtype=np.float64)
        self.bias = np.array(bias0, dtype=np.float64)
        self.P = np.diag([P0_QUAT_SIGMA**2] * 4 + [bias_sigma**2] * 3)
        self.declination = declination
        self.inclination = inclination
        self.field_norm = field_norm
        self.use_mag = use_mag
        self.stats = dict.fromkeys(["acc_soft", "acc_gated", "acc_rejected", "mag_gated",
                                    "mag_rejected", "yaw_resets", "tilt_resets", "gyro_sat"], 0)
        self.acc_reject_run = 0
        self.mag_reject_run = 0
        self.mag_accept_run = 0
        self.heading_converged = False
        self._normalize_and_project()

    def _normalize_and_project(self):
        norm = np.linalg.norm(self.q)
        self.q = self.q / norm
        transform = np.eye(7)
        transform[:4, :4] = (np.eye(4) - np.outer(self.q, self.q)) / norm
        self.P = transform @ self.P @ transform.T
        self.P = 0.5 * (self.P + self.P.T)

    def _inflate_attitude(self):
        self.P[:4, :] = 0.0
        self.P[:, :4] = 0.0
        self.P[:4, :4] = P0_QUAT_SIGMA**2 * np.eye(4)
        self._normalize_and_project()

    def predict(self, gyro, dt):
        rate = gyro - self.bias
        xi = xi_matrix(self.q)

        transition = np.eye(7)
        transition[:4, :4] += 0.5 * dt * omega_matrix(rate)
        transition[:4, 4:] = -0.5 * dt * xi
        process_noise = np.zeros((7, 7))
        process_noise[:4, :4] = GYRO_ARW**2 * dt / 4.0 * xi @ xi.T
        process_noise[4:, 4:] = GYRO_BIAS_RW**2 * dt * np.eye(3)

        self.q = quat_multiply(self.q, quat_from_rotation_vector(rate * dt))
        self.P = transition @ self.P @ transition.T + process_noise
        self._normalize_and_project()

        if np.any(np.abs(gyro) >= GYRO_SAT):
            self.stats["gyro_sat"] += 1
            self._inflate_attitude()

    def _apply_correction(self, jacobian, gain, noise, innovation):
        state = np.concatenate([self.q, self.bias]) + gain @ innovation
        self.q, self.bias = state[:4], state[4:]
        joseph = np.eye(7) - gain @ jacobian
        self.P = joseph @ self.P @ joseph.T + gain @ noise @ gain.T
        self._normalize_and_project()

    def update_accel(self, accel):
        accel_norm = np.linalg.norm(accel)
        norm_error = abs(accel_norm - G0) / G0
        if norm_error >= ACC_GATE_HARD:
            self.stats["acc_gated"] += 1
            return False

        variance = ACC_SIGMA**2
        if norm_error > ACC_GATE_SOFT:
            self.stats["acc_soft"] += 1
            fraction = (norm_error - ACC_GATE_SOFT) / (ACC_GATE_HARD - ACC_GATE_SOFT)
            variance *= 1.0 + (ACC_R_INFLATE_MAX - 1.0) * fraction

        predicted, quat_jacobian = gravity_direction(self.q)
        jacobian = np.hstack([quat_jacobian, np.zeros((3, 3))])
        noise = variance * np.eye(3)
        innovation = accel / accel_norm - predicted
        innovation_cov_inv = np.linalg.inv(jacobian @ self.P @ jacobian.T + noise)

        if innovation @ innovation_cov_inv @ innovation > ACC_NIS_MAX:
            self.stats["acc_rejected"] += 1
            self.acc_reject_run += 1
            if self.acc_reject_run >= TILT_RESET_REJECTS:
                roll, pitch = tilt_from_accel(accel)
                self.q = quat_from_euler(roll, pitch, quat_to_euler(self.q)[2])
                self.stats["tilt_resets"] += 1
                self.acc_reject_run = 0
                self._inflate_attitude()
            return False

        self.acc_reject_run = 0
        self._apply_correction(jacobian, self.P @ jacobian.T @ innovation_cov_inv, noise,
                               innovation)
        return True

    def _mag_passes_gates(self, mag, mag_n, cos2_pitch):
        dip = math.atan2(mag_n[2], math.hypot(mag_n[0], mag_n[1]))
        norm_error = abs(np.linalg.norm(mag) - self.field_norm) / self.field_norm
        return (self.use_mag and norm_error <= MAG_NORM_GATE
                and abs(dip - self.inclination) <= MAG_DIP_GATE
                and cos2_pitch >= MAG_MIN_COS2_PITCH)

    def _reject_mag(self, heading_error):
        self.stats["mag_rejected"] += 1
        self.mag_reject_run += 1
        self.mag_accept_run = 0

        if not self.heading_converged:
            if self.mag_reject_run >= YAW_RESET_REJECTS:
                roll, pitch, yaw = quat_to_euler(self.q)
                self.q = quat_from_euler(roll, pitch, wrap_pi(yaw + heading_error))
                self.stats["yaw_resets"] += 1
                self.mag_reject_run = 0
                self._inflate_attitude()
        elif self.mag_reject_run == YAW_RESET_REJECTS:
            self._inflate_attitude()
        elif self.mag_reject_run >= MAG_DISTRUST_REJECTS:
            self.heading_converged = False

    def update_mag(self, mag):
        mag_n = rotation_matrix(self.q) @ mag
        quat_jacobian, cos2_pitch = heading_jacobian(self.q)
        if not self._mag_passes_gates(mag, mag_n, cos2_pitch):
            self.stats["mag_gated"] += 1
            return False

        heading_error = wrap_pi(self.declination - math.atan2(mag_n[1], mag_n[0]))
        jacobian = np.concatenate([quat_jacobian, np.zeros(3)]).reshape(1, 7)
        noise = np.array([[HEADING_SIGMA**2]])
        innovation_var = (jacobian @ self.P @ jacobian.T + noise)[0, 0]

        if heading_error**2 / innovation_var > MAG_NIS_MAX:
            self._reject_mag(heading_error)
            return False

        self.mag_reject_run = 0
        self.mag_accept_run += 1
        if self.mag_accept_run >= MAG_CONVERGE_ACCEPTS:
            self.heading_converged = True
        self._apply_correction(jacobian, self.P @ jacobian.T / innovation_var, noise,
                               np.array([heading_error]))
        return True

    def step(self, sample):
        self.predict(sample["gyro"], DT)
        self.update_accel(sample["accel"])
        self.update_mag(sample["mag"])


def euler_rotation(euler):
    """R = Rz(yaw) Ry(pitch) Rx(roll); independent of the quaternion code above."""
    sr, cr = math.sin(euler[0]), math.cos(euler[0])
    sp, cp = math.sin(euler[1]), math.cos(euler[1])
    sy, cy = math.sin(euler[2]), math.cos(euler[2])
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class Sim:
    """Synthetic IMU, the float64 twin of app/tests/ekf/src/sim.c."""

    def __init__(self, motion, disturbance=None, gyro_limit=0.0):
        self.motion = motion
        self.disturbance = disturbance
        self.gyro_limit = gyro_limit
        self.seed = NOISE_SEED
        self.step = 0

    def _uniform(self):
        self.seed ^= (self.seed << 13) & 0xFFFFFFFF
        self.seed ^= self.seed >> 17
        self.seed ^= (self.seed << 5) & 0xFFFFFFFF
        return (self.seed >> 8) / 16777216.0

    def _randn(self):
        return sum(self._uniform() for _ in range(12)) - 6.0

    def next(self):
        self.step += 1
        t = self.step * DT

        mid_euler, mid_rate = self.motion(t - 0.5 * DT)
        sr, cr = math.sin(mid_euler[0]), math.cos(mid_euler[0])
        sp, cp = math.sin(mid_euler[1]), math.cos(mid_euler[1])
        rate_b = np.array([mid_rate[0] - mid_rate[2] * sp,
                           mid_rate[1] * cr + mid_rate[2] * sr * cp,
                           -mid_rate[1] * sr + mid_rate[2] * cr * cp])

        euler, _ = self.motion(t)
        rot = euler_rotation(euler)
        accel = -G0 * rot[2, :]
        mag = rot.T @ FIELD_N
        if self.disturbance is not None:
            self.disturbance(t, accel, mag)

        gyro = rate_b + TRUE_BIAS
        for axis in range(3):
            gyro[axis] += GYRO_NOISE * self._randn()
            accel[axis] += ACCEL_NOISE * self._randn()
            mag[axis] += MAG_NOISE * self._randn()
        if self.gyro_limit > 0.0:
            gyro = np.clip(gyro, -self.gyro_limit, self.gyro_limit)

        return {"t": t, "euler": np.array(euler), "gyro": gyro, "accel": accel, "mag": mag}


def static_motion(roll_deg, pitch_deg, yaw_deg):
    euler = np.radians([roll_deg, pitch_deg, yaw_deg])
    return lambda t: (euler.copy(), np.zeros(3))


def dynamic_motion(t):
    roll_w, pitch_w = 2 * math.pi * 0.2, 2 * math.pi * 0.13
    roll_amp, pitch_amp, yaw_rate = math.radians(20), math.radians(15), math.radians(30)
    euler = np.array([roll_amp * math.sin(roll_w * t), pitch_amp * math.sin(pitch_w * t),
                      yaw_rate * t])
    rate = np.array([roll_amp * roll_w * math.cos(roll_w * t),
                     pitch_amp * pitch_w * math.cos(pitch_w * t), yaw_rate])
    return euler, rate


def fast_roll_motion(t, start=10.0, duration=0.35):
    euler, rate = np.radians([0.0, 0.0, 30.0]), np.zeros(3)
    if start <= t < start + duration:
        euler[0] += 2 * math.pi / duration * (t - start)
        rate[0] = 2 * math.pi / duration
    return euler, rate


def error_deg(ekf, sample):
    difference = np.degrees(quat_to_euler(ekf.q) - sample["euler"])
    return (difference + 180.0) % 360.0 - 180.0


def start_from_stationary_alignment(sim):
    samples = [sim.next() for _ in range(ALIGN_SAMPLES)]
    accel_mean = np.mean([s["accel"] for s in samples], axis=0)
    mag_mean = np.mean([s["mag"] for s in samples], axis=0)
    gyro_mean = np.mean([s["gyro"] for s in samples], axis=0)
    return Ekf(align(accel_mean, mag_mean, True, 0.0), gyro_mean, stationary=True), gyro_mean


def run_and_track(sim, ekf, track_from, end):
    """Largest absolute attitude error per axis for t >= track_from, in degrees."""
    worst = np.zeros(3)
    while True:
        sample = sim.next()
        ekf.step(sample)
        if sample["t"] >= track_from:
            worst = np.maximum(worst, np.abs(error_deg(ekf, sample)))
        if sample["t"] >= end:
            return worst


def scenario_static():
    sim = Sim(static_motion(10, -5, 120))
    ekf = Ekf(quat_from_euler(*np.radians([10, -5, 120])), np.zeros(3), stationary=False)
    worst = run_and_track(sim, ekf, 30.0, 30.0)
    bias_error = np.max(np.abs(ekf.bias - TRUE_BIAS))
    bias_var = np.max(np.diag(ekf.P)[4:])
    ok = worst[0] < 0.1 and worst[1] < 0.1 and worst[2] < 0.3 and bias_error < 5e-4 \
        and bias_var < 1e-6
    return ok, f"err {worst} deg, bias err {bias_error:.2e} rad/s, max P_bb {bias_var:.2e}"


def scenario_dynamic():
    sim = Sim(dynamic_motion)
    first = sim.next()
    ekf = Ekf(align(first["accel"], first["mag"], True, 0.0), np.zeros(3), stationary=False)
    errors, total = [], 0
    while True:
        sample = sim.next()
        ekf.step(sample)
        total += 1
        if sample["t"] >= 5.0:
            errors.append(error_deg(ekf, sample))
        if sample["t"] >= 30.0:
            break
    errors = np.array(errors)
    rms, worst = np.sqrt(np.mean(errors**2, axis=0)), np.max(np.abs(errors), axis=0)
    rejected = ekf.stats["acc_rejected"] + ekf.stats["mag_rejected"]
    ok = np.all(rms < 0.5) and np.all(worst < 1.5) and rejected * 100 < total
    return ok, f"rms {rms} deg, max {worst} deg, rejected {rejected}/{total}"


def scenario_accel(disturbance, limit_deg):
    sim = Sim(static_motion(10, -5, 120), disturbance)
    ekf, _ = start_from_stationary_alignment(sim)
    worst = run_and_track(sim, ekf, 10.0, 20.0)
    ok = worst[0] < limit_deg and worst[1] < limit_deg and ekf.stats["tilt_resets"] == 0
    return ok, (f"roll/pitch dev {worst[:2]} deg, acc_rejected {ekf.stats['acc_rejected']}, "
                f"tilt_resets {ekf.stats['tilt_resets']}")


def accel_step(t, accel, mag):
    if 10.0 <= t < 12.0:
        accel[0] += 0.5 * G0


def accel_shake(t, accel, mag):
    if 10.0 <= t < 15.0:
        accel[0] += 0.3 * G0 * math.sin(2 * math.pi * 1.0 * t)
        accel[1] += 0.2 * G0 * math.cos(2 * math.pi * 0.7 * t)


def mag_pulse(t, accel, mag):
    if 10.0 <= t < 15.0:
        mag[0] += 0.3


def mag_permanent(t, accel, mag):
    if t >= 10.0:
        mag[0] += 0.3


def scenario_mag_pulse():
    sim = Sim(static_motion(10, -5, 120), mag_pulse)
    ekf, _ = start_from_stationary_alignment(sim)
    worst = run_and_track(sim, ekf, 10.0, 20.0)
    ok = worst[2] < 1.0 and ekf.stats["yaw_resets"] == 0 and ekf.stats["mag_rejected"] > 0
    return ok, (f"yaw dev {worst[2]:.3f} deg, mag_rejected {ekf.stats['mag_rejected']}, "
                f"mag_gated {ekf.stats['mag_gated']}")


def scenario_mag_permanent():
    sim = Sim(static_motion(10, -5, 120), mag_permanent)
    ekf, _ = start_from_stationary_alignment(sim)
    reset_time = None
    while True:
        resets_before = ekf.stats["yaw_resets"]
        sample = sim.next()
        ekf.step(sample)
        if ekf.stats["yaw_resets"] != resets_before:
            reset_time = sample["t"]
        if sample["t"] >= 60.0:
            break
    ok = ekf.stats["yaw_resets"] == 1 and reset_time is not None and 39.0 <= reset_time <= 42.0
    return ok, f"yaw_resets {ekf.stats['yaw_resets']}, reset at {reset_time} s"


def scenario_wrong_heading():
    sim = Sim(static_motion(10, -5, 120))
    gyro_mean = np.mean([sim.next()["gyro"] for _ in range(ALIGN_SAMPLES)], axis=0)
    ekf = Ekf(quat_from_euler(*np.radians([10, -5, 0])), gyro_mean, stationary=True)
    sample = sim.next()
    start = sample["t"]
    ekf.step(sample)
    while sample["t"] < start + 5.0:
        sample = sim.next()
        ekf.step(sample)
    yaw_error = abs(error_deg(ekf, sample)[2])
    resets_at_5s = ekf.stats["yaw_resets"]
    while sample["t"] < start + 10.0:
        sample = sim.next()
        ekf.step(sample)
    ok = resets_at_5s == 1 and yaw_error < 1.0 and ekf.heading_converged
    return ok, (f"yaw err at 5 s {yaw_error:.3f} deg, yaw_resets {resets_at_5s}, "
                f"converged at 10 s {ekf.heading_converged}")


def scenario_heading_leak():
    truth = np.radians([20, -15, 50])
    mag = euler_rotation(truth).T @ FIELD_N
    ekf = Ekf(quat_from_euler(truth[0], truth[1], math.radians(49)), np.zeros(3),
              stationary=True)
    before = quat_to_euler(ekf.q)
    accepted = ekf.update_mag(mag)
    moved = np.degrees(quat_to_euler(ekf.q) - before)
    ok = accepted and abs(moved[0]) < 0.25 and abs(moved[1]) < 0.25 and moved[2] > 0.0
    return ok, f"moved roll/pitch/yaw {moved} deg"


def scenario_gyro_saturation():
    sim = Sim(fast_roll_motion, gyro_limit=8.727)
    ekf, _ = start_from_stationary_alignment(sim)
    worst = run_and_track(sim, ekf, 10.85, 12.85)
    ok = ekf.stats["gyro_sat"] > 0 and np.all(worst < 2.0)
    return ok, f"err {worst} deg, gyro_sat {ekf.stats['gyro_sat']}"


SCENARIOS = [
    ("T-EKF-06 static convergence", scenario_static),
    ("T-EKF-07 dynamic tracking", scenario_dynamic),
    ("T-EKF-08a 0.5 g step", lambda: scenario_accel(accel_step, 0.5)),
    ("T-EKF-08b shaking", lambda: scenario_accel(accel_shake, 3.0)),
    ("T-EKF-09a magnetic pulse", scenario_mag_pulse),
    ("T-EKF-09b permanent disturbance", scenario_mag_permanent),
    ("T-EKF-10 wrong initial heading", scenario_wrong_heading),
    ("T-EKF-11 heading leak into tilt", scenario_heading_leak),
    ("T-EKF-12 gyro saturation", scenario_gyro_saturation),
]


def reference_vectors():
    """(name, computed value, value printed in MATH_SPEC.md section 15)."""
    euler = np.radians([30, -20, 45])
    q = quat_from_euler(*euler)
    rot = rotation_matrix(q)
    accel_b = -G0 * rot[2, :]
    mag_b = rot.T @ FIELD_N
    predicted, gravity_jacobian = gravity_direction(q)
    dq = quat_from_rotation_vector(np.array([0.1, -0.2, 0.3]) * 0.01)

    yaw_filter = Ekf([1, 0, 0, 0], np.zeros(3), stationary=True)
    for _ in range(208):
        yaw_filter.predict(np.array([0.0, 0.0, 0.5]), DT)

    return [
        ("V1 q", q, [0.8616424, 0.2996729, -0.0574224, 0.4055504]),
        ("V2 R", rot, [[0.6644630, -0.7332948, 0.1441097],
                       [0.6644630, 0.4914501, -0.5629971],
                       [0.3420201, 0.4698463, 0.8137977]]),
        ("V3 f_b", accel_b, [-3.3540718, -4.6076183, -7.9806290]),
        ("V3 h_a", predicted, [-0.3420201, -0.4698463, -0.8137977]),
        ("V4 m_b", mag_b, [0.3029238, 0.0046148, 0.3615465]),
        ("V4 y_psi", wrap_pi(0.0 - math.atan2(*(rot @ mag_b)[1::-1])), 0.0),
        ("V5 euler [deg]", np.degrees(quat_to_euler(align(accel_b, mag_b, True, 0.0))),
         [30.0, -20.0, 45.0]),
        ("V6 H_a", gravity_jacobian, [[-0.1148449, -0.8111009, 1.7232849, -0.5993457],
                                      [-0.5993457, -1.7232849, -0.8111009, 0.1148449],
                                      [-1.7232849, 0.5993457, -0.1148449, -0.8111009]]),
        ("V6 H_psi", heading_jacobian(q)[0], [0.6103431, -0.0864193, 0.2781614, 2.5174362]),
        ("V7 dq", dq, [0.9999983, 0.0005000, -0.0010000, 0.0015000]),
        ("V7 q*dq", quat_multiply(q, dq), [0.8608253, 0.3004226, -0.0585307, 0.4065712]),
        ("V8 yaw [rad]", quat_to_euler(yaw_filter.q)[2], 1.0),
        ("V9 f_b", np.array([1, -1, -1]) * np.array([0.0, 0.0, 9.80665]), [0.0, 0.0, -9.80665]),
        ("V9 m_b", np.array([1, -1, 1]) * np.array([1.0, 2.0, 3.0]), [1.0, -2.0, 3.0]),
        ("V9 det ism_to_body", np.linalg.det(np.diag([1.0, -1.0, -1.0])), 1.0),
        ("V9 det mmc_to_body", np.linalg.det(np.diag([1.0, -1.0, 1.0])), -1.0),
    ]


def run_vectors():
    # The specification prints seven decimals.
    tolerance = 1.0e-6
    all_ok = True
    np.set_printoptions(precision=7, suppress=True)
    for name, computed, printed in reference_vectors():
        deviation = float(np.max(np.abs(np.asarray(computed) - np.asarray(printed))))
        ok = deviation <= tolerance
        all_ok &= ok
        print(f"{'PASS' if ok else 'FAIL'}  {name}: max deviation {deviation:.1e}")
        print(f"      {np.asarray(computed)}".replace("\n", "\n      "))
    return all_ok


def run_scenarios():
    all_ok = True
    np.set_printoptions(precision=4, suppress=True)
    for name, scenario in SCENARIOS:
        ok, detail = scenario()
        all_ok &= bool(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    return all_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=["vectors", "scenarios", "all"])
    command = parser.parse_args().command

    ok = True
    if command in ("vectors", "all"):
        ok &= run_vectors()
    if command in ("scenarios", "all"):
        ok &= run_scenarios()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
