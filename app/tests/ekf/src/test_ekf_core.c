/* TEST_PLAN.md section 3.3, T-EKF-01..13, plus the gate paths of MATH_SPEC.md 8 and 9. */
#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "ekf_config.h"
#include "ekf_core.h"
#include "ekf_math.h"
#include "sim.h"

#define N            EKF_STATE_DIM
#define RAD_TO_DEG   (1.0f / SIM_DEG)
#define STEPS_PER_S  104
#define TWO_PI       (2.0f * EKF_PI)
#define ROLL_START_S 10.0f
#define ROLL_TIME_S  0.35f

static const struct ekf_params params = {
	.declination_rad = 0.0f,
	.inclination_rad = SIM_FIELD_DIP,
	.field_norm_g = SIM_FIELD_NORM,
	.use_mag = true,
};

static const float v1_q[4] = { 0.8616424f, 0.2996729f, -0.0574224f, 0.4055504f };
static const float zero_bias[3] = { 0.0f, 0.0f, 0.0f };

static struct ekf ekf;
static struct sim sim;

/* Attitude held by motion_static(), rad. */
static float pose[3];

static void set_pose_deg(float roll_deg, float pitch_deg, float yaw_deg)
{
	pose[0] = roll_deg * SIM_DEG;
	pose[1] = pitch_deg * SIM_DEG;
	pose[2] = yaw_deg * SIM_DEG;
}

static void motion_static(float t, float euler[3], float euler_rate[3])
{
	ARG_UNUSED(t);

	for (int i = 0; i < 3; i++) {
		euler[i] = pose[i];
		euler_rate[i] = 0.0f;
	}
}

static void motion_dynamic(float t, float euler[3], float euler_rate[3])
{
	const float roll_w = TWO_PI * 0.2f, pitch_w = TWO_PI * 0.13f;
	const float roll_amp = 20.0f * SIM_DEG, pitch_amp = 15.0f * SIM_DEG;
	const float yaw_rate = 30.0f * SIM_DEG;

	euler[0] = roll_amp * sinf(roll_w * t);
	euler[1] = pitch_amp * sinf(pitch_w * t);
	euler[2] = yaw_rate * t;
	euler_rate[0] = roll_amp * roll_w * cosf(roll_w * t);
	euler_rate[1] = pitch_amp * pitch_w * cosf(pitch_w * t);
	euler_rate[2] = yaw_rate;
}

/* One full roll at 1028 dps on top of the static pose. */
static void motion_fast_roll(float t, float euler[3], float euler_rate[3])
{
	const float rate = TWO_PI / ROLL_TIME_S;

	motion_static(t, euler, euler_rate);
	if (t >= ROLL_START_S && t < ROLL_START_S + ROLL_TIME_S) {
		euler[0] += rate * (t - ROLL_START_S);
		euler_rate[0] = rate;
	}
}

static void disturb_accel_step(float t, float accel_b[3], float mag_b[3])
{
	ARG_UNUSED(mag_b);

	if (t >= 10.0f && t < 12.0f) {
		accel_b[0] += 0.5f * EKF_G0;
	}
}

static void disturb_accel_shake(float t, float accel_b[3], float mag_b[3])
{
	ARG_UNUSED(mag_b);

	if (t >= 10.0f && t < 15.0f) {
		accel_b[0] += 0.3f * EKF_G0 * sinf(TWO_PI * 1.0f * t);
		accel_b[1] += 0.2f * EKF_G0 * cosf(TWO_PI * 0.7f * t);
	}
}

static void disturb_mag_pulse(float t, float accel_b[3], float mag_b[3])
{
	ARG_UNUSED(accel_b);

	if (t >= 10.0f && t < 15.0f) {
		mag_b[0] += 0.3f;
	}
}

static void disturb_mag_permanent(float t, float accel_b[3], float mag_b[3])
{
	ARG_UNUSED(accel_b);

	if (t >= 10.0f) {
		mag_b[0] += 0.3f;
	}
}

static bool applied_or_gated(int rc)
{
	return rc == 0 || rc == -EAGAIN;
}

static void filter_step(const struct sim_sample *sample)
{
	zassert_ok(ekf_predict(&ekf, sample->gyro, SIM_DT_S));
	zassert_true(applied_or_gated(ekf_update_accel(&ekf, sample->accel)));
	zassert_true(applied_or_gated(ekf_update_mag(&ekf, sample->mag)));
}

/* Estimate minus truth for roll, pitch and yaw, each wrapped, in degrees. */
static void attitude_error_deg(const struct sim_sample *sample, float error_deg[3])
{
	float estimate[3];

	quat_to_euler(ekf.x, &estimate[0], &estimate[1], &estimate[2]);
	for (int i = 0; i < 3; i++) {
		error_deg[i] = sim_wrap_deg((estimate[i] - sample->euler[i]) * RAD_TO_DEG);
	}
}

static int to_mdeg(float angle_deg)
{
	return (int)(angle_deg * 1000.0f);
}

/* Averages one alignment window the way the fusion thread does. */
static void average_window(float accel_mean[3], float mag_mean[3], float gyro_mean[3])
{
	struct sim_sample sample;

	memset(accel_mean, 0, 3 * sizeof(float));
	memset(mag_mean, 0, 3 * sizeof(float));
	memset(gyro_mean, 0, 3 * sizeof(float));

	for (uint32_t n = 0; n < EKF_ALIGN_SAMPLES; n++) {
		sim_next(&sim, &sample);
		for (int i = 0; i < 3; i++) {
			accel_mean[i] += sample.accel[i] / (float)EKF_ALIGN_SAMPLES;
			mag_mean[i] += sample.mag[i] / (float)EKF_ALIGN_SAMPLES;
			gyro_mean[i] += sample.gyro[i] / (float)EKF_ALIGN_SAMPLES;
		}
	}
}

static void start_from_stationary_alignment(void)
{
	float accel_mean[3], mag_mean[3], gyro_mean[3], q[4];

	average_window(accel_mean, mag_mean, gyro_mean);
	zassert_ok(ekf_align(accel_mean, mag_mean, true, params.declination_rad, q));
	zassert_ok(ekf_init(&ekf, &params, q, gyro_mean, true));
}

/* Runs the simulation up to end_s and tracks the largest error seen from track_from_s on. */
static void run_and_track(float track_from_s, float end_s, float max_error_deg[3])
{
	struct sim_sample sample;
	float error_deg[3];

	memset(max_error_deg, 0, 3 * sizeof(float));

	do {
		sim_next(&sim, &sample);
		filter_step(&sample);
		if (sample.t < track_from_s) {
			continue;
		}

		attitude_error_deg(&sample, error_deg);
		for (int i = 0; i < 3; i++) {
			max_error_deg[i] = fmaxf(max_error_deg[i], fabsf(error_deg[i]));
		}
	} while (sample.t < end_s);
}

static void reset_fixture(void *fixture)
{
	ARG_UNUSED(fixture);

	memset(&ekf, 0, sizeof(ekf));
	set_pose_deg(0.0f, 0.0f, 0.0f);
	sim_init(&sim, motion_static, NULL);
}

ZTEST(ekf_core, test_init_projects_covariance_onto_tangent_space)
{
	const float quat_variance = EKF_P0_QUAT_SIGMA * EKF_P0_QUAT_SIGMA;
	const float bias_variance = EKF_P0_BIAS_SIGMA_STATIC * EKF_P0_BIAS_SIGMA_STATIC;
	float trace = 0.0f;

	zassert_ok(ekf_init(&ekf, &params, v1_q, zero_bias, true));
	zassert_equal(ekf.state, EKF_STATE_RUNNING);

	for (int row = 0; row < 4; row++) {
		float along_q = 0.0f;

		trace += ekf.P[row * N + row];
		for (int col = 0; col < 4; col++) {
			along_q += ekf.P[row * N + col] * ekf.x[col];
		}
		zassert_within(along_q, 0.0f, 1e-8f, "P_qq q row %d", row);
	}
	zassert_within(trace, 3.0f * quat_variance, 3.0f * quat_variance * 1e-5f);

	for (int i = 4; i < N; i++) {
		zassert_within(ekf.P[i * N + i], bias_variance, bias_variance * 1e-5f);
	}
}

ZTEST(ekf_core, test_align_recovers_reference_attitude)
{
	const float v3_accel[3] = { -3.3540718f, -4.6076183f, -7.9806290f };
	const float v4_mag[3] = { 0.3029238f, 0.0046148f, 0.3615465f };
	float q[4], roll, pitch, yaw;

	zassert_ok(ekf_align(v3_accel, v4_mag, true, 0.0f, q));
	quat_to_euler(q, &roll, &pitch, &yaw);
	zassert_within(roll, 30.0f * SIM_DEG, 1e-4f);
	zassert_within(pitch, -20.0f * SIM_DEG, 1e-4f);
	zassert_within(yaw, 45.0f * SIM_DEG, 1e-4f);

	zassert_ok(ekf_align(v3_accel, v4_mag, false, 0.0f, q));
	quat_to_euler(q, &roll, &pitch, &yaw);
	zassert_within(roll, 30.0f * SIM_DEG, 1e-4f);
	zassert_within(pitch, -20.0f * SIM_DEG, 1e-4f);
	zassert_within(yaw, 0.0f, 1e-4f);
}

ZTEST(ekf_core, test_predict_integrates_constant_yaw_rate)
{
	const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	const float gyro[3] = { 0.0f, 0.0f, 0.5f };
	float roll, pitch, yaw;

	zassert_ok(ekf_init(&ekf, &params, identity, zero_bias, true));
	for (int step = 0; step < 208; step++) {
		zassert_ok(ekf_predict(&ekf, gyro, SIM_DT_S));
	}

	quat_to_euler(ekf.x, &roll, &pitch, &yaw);
	zassert_within(yaw, 1.0f, 1e-4f);
}

static void assert_predict_invariants(int step)
{
	const float *q = ekf.x;
	float q_norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	float max_abs = 0.0f, max_asymmetry = 0.0f, pqq_norm_sq = 0.0f, along_q_sq = 0.0f;

	zassert_within(q_norm, 1.0f, 1e-6f, "step %d", step);

	for (int row = 0; row < N; row++) {
		zassert_true(ekf.P[row * N + row] >= 0.0f, "step %d", step);
		for (int col = 0; col < N; col++) {
			float element = ekf.P[row * N + col];

			max_abs = fmaxf(max_abs, fabsf(element));
			max_asymmetry = fmaxf(max_asymmetry, fabsf(element - ekf.P[col * N + row]));
		}
	}
	zassert_true(max_asymmetry <= 1e-6f * max_abs, "step %d", step);

	for (int row = 0; row < 4; row++) {
		float along_q = 0.0f;

		for (int col = 0; col < 4; col++) {
			along_q += ekf.P[row * N + col] * q[col];
			pqq_norm_sq += ekf.P[row * N + col] * ekf.P[row * N + col];
		}
		along_q_sq += along_q * along_q;
	}
	zassert_true(sqrtf(along_q_sq) < 1e-6f * sqrtf(pqq_norm_sq), "step %d", step);
}

ZTEST(ekf_core, test_predict_keeps_quaternion_and_covariance_invariants)
{
	/* Each component below 5/sqrt(3) keeps the rate norm below 5 rad/s. */
	const float component_limit = 5.0f / sqrtf(3.0f);
	uint32_t seed = 0x12345678U;

	zassert_ok(ekf_init(&ekf, &params, v1_q, zero_bias, false));

	for (int step = 0; step < 10000; step++) {
		float gyro[3];

		for (int i = 0; i < 3; i++) {
			gyro[i] = component_limit * (2.0f * sim_uniform(&seed) - 1.0f);
		}
		zassert_ok(ekf_predict(&ekf, gyro, SIM_DT_S));
		assert_predict_invariants(step);
	}
}

ZTEST(ekf_core, test_invalid_input_is_rejected_without_touching_state)
{
	const float gyro[3] = { 0.1f, 0.2f, 0.3f };
	const float nan_gyro[3] = { 0.1f, NAN, 0.3f };
	const float tiny_accel[3] = { 5e-4f, 0.0f, 0.0f };
	const float tiny_mag[3] = { 0.0f, 5e-7f, 0.0f };
	const float nan_vector[3] = { NAN, 0.0f, 1.0f };
	static struct ekf before;

	zassert_ok(ekf_init(&ekf, &params, v1_q, zero_bias, true));
	memcpy(&before, &ekf, sizeof(ekf));

	zassert_equal(ekf_predict(&ekf, nan_gyro, SIM_DT_S), -EINVAL);
	zassert_equal(ekf_predict(&ekf, gyro, 0.0f), -EINVAL);
	zassert_equal(ekf_predict(&ekf, gyro, 0.06f), -EINVAL);
	zassert_equal(ekf_predict(&ekf, gyro, NAN), -EINVAL);
	zassert_equal(ekf_update_accel(&ekf, tiny_accel), -EINVAL);
	zassert_equal(ekf_update_accel(&ekf, nan_vector), -EINVAL);
	zassert_equal(ekf_update_mag(&ekf, tiny_mag), -EINVAL);
	zassert_equal(ekf_update_mag(&ekf, nan_vector), -EINVAL);

	zassert_mem_equal(&before, &ekf, sizeof(ekf));
}

ZTEST(ekf_core, test_static_run_converges_attitude_and_bias)
{
	float q[4], max_error_deg[3];

	set_pose_deg(10.0f, -5.0f, 120.0f);
	quat_from_euler(pose[0], pose[1], pose[2], q);
	zassert_ok(ekf_init(&ekf, &params, q, zero_bias, false));

	run_and_track(30.0f, 30.0f, max_error_deg);

	zassert_true(max_error_deg[0] < 0.1f, "roll %d mdeg", to_mdeg(max_error_deg[0]));
	zassert_true(max_error_deg[1] < 0.1f, "pitch %d mdeg", to_mdeg(max_error_deg[1]));
	zassert_true(max_error_deg[2] < 0.3f, "yaw %d mdeg", to_mdeg(max_error_deg[2]));

	for (int i = 0; i < 3; i++) {
		float bias_error = fabsf(ekf.x[4 + i] - sim_true_bias[i]);

		zassert_true(bias_error < 5e-4f, "bias %d error %d urad/s", i,
			     (int)(bias_error * 1e6f));
		zassert_true(ekf.P[(4 + i) * N + 4 + i] < 1e-6f, "bias %d variance", i);
	}
}

ZTEST(ekf_core, test_dynamic_motion_is_tracked)
{
	const float settle_s = 5.0f, end_s = 30.0f;
	float sum_sq[3] = { 0.0f, 0.0f, 0.0f }, max_error_deg[3] = { 0.0f, 0.0f, 0.0f };
	uint32_t tracked = 0, total = 0;
	struct sim_sample sample;
	struct ekf_stats stats;
	float q[4];

	/* Alignment from the first sample: the board is already moving. */
	sim_init(&sim, motion_dynamic, NULL);
	sim_next(&sim, &sample);
	zassert_ok(ekf_align(sample.accel, sample.mag, true, params.declination_rad, q));
	zassert_ok(ekf_init(&ekf, &params, q, zero_bias, false));

	while (sample.t < end_s) {
		float error_deg[3];

		sim_next(&sim, &sample);
		filter_step(&sample);
		total++;
		if (sample.t < settle_s) {
			continue;
		}

		attitude_error_deg(&sample, error_deg);
		tracked++;
		for (int i = 0; i < 3; i++) {
			sum_sq[i] += error_deg[i] * error_deg[i];
			max_error_deg[i] = fmaxf(max_error_deg[i], fabsf(error_deg[i]));
		}
	}

	for (int i = 0; i < 3; i++) {
		float rms = sqrtf(sum_sq[i] / (float)tracked);

		zassert_true(rms < 0.5f, "axis %d rms %d mdeg", i, to_mdeg(rms));
		zassert_true(max_error_deg[i] < 1.5f, "axis %d max %d mdeg", i,
			     to_mdeg(max_error_deg[i]));
	}

	ekf_get_stats(&ekf, &stats);
	zassert_true((stats.acc_rejected + stats.mag_rejected) * 100U < total, "rejected %u + %u",
		     stats.acc_rejected, stats.mag_rejected);
}

ZTEST(ekf_core, test_constant_linear_acceleration_is_rejected)
{
	float max_error_deg[3];
	struct ekf_stats stats;

	set_pose_deg(10.0f, -5.0f, 120.0f);
	sim_init(&sim, motion_static, disturb_accel_step);
	start_from_stationary_alignment();

	run_and_track(10.0f, 20.0f, max_error_deg);

	ekf_get_stats(&ekf, &stats);
	zassert_true(max_error_deg[0] < 0.5f, "roll %d mdeg", to_mdeg(max_error_deg[0]));
	zassert_true(max_error_deg[1] < 0.5f, "pitch %d mdeg", to_mdeg(max_error_deg[1]));
	zassert_equal(stats.tilt_resets, 0);
	zassert_true(stats.acc_rejected + stats.acc_gated > 0);
}

ZTEST(ekf_core, test_shaking_acceleration_stays_bounded)
{
	float max_error_deg[3];

	set_pose_deg(10.0f, -5.0f, 120.0f);
	sim_init(&sim, motion_static, disturb_accel_shake);
	start_from_stationary_alignment();

	run_and_track(10.0f, 20.0f, max_error_deg);

	zassert_true(max_error_deg[0] < 3.0f, "roll %d mdeg", to_mdeg(max_error_deg[0]));
	zassert_true(max_error_deg[1] < 3.0f, "pitch %d mdeg", to_mdeg(max_error_deg[1]));
}

ZTEST(ekf_core, test_short_magnetic_disturbance_is_ignored)
{
	float max_error_deg[3];
	struct ekf_stats stats;

	/* At this heading the disturbed field keeps its norm and dip, so only the NIS gate sees it. */
	set_pose_deg(10.0f, -5.0f, 120.0f);
	sim_init(&sim, motion_static, disturb_mag_pulse);
	start_from_stationary_alignment();

	run_and_track(10.0f, 20.0f, max_error_deg);

	ekf_get_stats(&ekf, &stats);
	zassert_true(max_error_deg[2] < 1.0f, "yaw %d mdeg", to_mdeg(max_error_deg[2]));
	zassert_equal(stats.yaw_resets, 0);
	zassert_true(stats.mag_rejected > 0, "the disturbance never reached the NIS gate");
}

ZTEST(ekf_core, test_permanent_magnetic_disturbance_is_adopted_after_distrust_period)
{
	struct sim_sample sample;
	float reset_time_s = 0.0f;

	set_pose_deg(10.0f, -5.0f, 120.0f);
	sim_init(&sim, motion_static, disturb_mag_permanent);
	start_from_stationary_alignment();

	do {
		uint32_t resets_before = ekf.stats.yaw_resets;

		sim_next(&sim, &sample);
		filter_step(&sample);
		if (ekf.stats.yaw_resets != resets_before) {
			reset_time_s = sample.t;
		}
	} while (sample.t < 60.0f);

	zassert_equal(ekf.stats.yaw_resets, 1, "yaw resets %u", ekf.stats.yaw_resets);
	zassert_true(reset_time_s >= 39.0f && reset_time_s <= 42.0f, "reset at %d ms",
		     (int)(reset_time_s * 1000.0f));
}

ZTEST(ekf_core, test_wrong_initial_heading_is_recovered_by_yaw_reset)
{
	float accel_mean[3], mag_mean[3], gyro_mean[3], q[4], error_deg[3];
	struct sim_sample sample;
	float start_s;

	set_pose_deg(10.0f, -5.0f, 120.0f);
	average_window(accel_mean, mag_mean, gyro_mean);
	quat_from_euler(pose[0], pose[1], 0.0f, q);
	zassert_ok(ekf_init(&ekf, &params, q, gyro_mean, true));
	zassert_false(ekf.heading_converged);

	sim_next(&sim, &sample);
	start_s = sample.t;
	filter_step(&sample);

	while (sample.t < start_s + 5.0f) {
		sim_next(&sim, &sample);
		filter_step(&sample);
	}
	attitude_error_deg(&sample, error_deg);
	zassert_equal(ekf.stats.yaw_resets, 1, "yaw resets %u", ekf.stats.yaw_resets);
	zassert_true(fabsf(error_deg[2]) < 1.0f, "yaw %d mdeg", to_mdeg(error_deg[2]));

	while (sample.t < start_s + 10.0f) {
		sim_next(&sim, &sample);
		filter_step(&sample);
	}
	zassert_true(ekf.heading_converged);
	zassert_true(ekf.yaw_valid);
	zassert_equal(ekf.stats.yaw_resets, 1);
}

ZTEST(ekf_core, test_heading_update_leaks_little_into_tilt)
{
	const float truth[3] = { 20.0f * SIM_DEG, -15.0f * SIM_DEG, 50.0f * SIM_DEG };
	float rot[3][3], mag_b[3], q[4], before[3], after[3];

	sim_rotation_matrix(truth, rot);
	for (int i = 0; i < 3; i++) {
		mag_b[i] = rot[0][i] * sim_field_n[0] + rot[1][i] * sim_field_n[1] +
			   rot[2][i] * sim_field_n[2];
	}

	quat_from_euler(truth[0], truth[1], 49.0f * SIM_DEG, q);
	zassert_ok(ekf_init(&ekf, &params, q, zero_bias, true));
	quat_to_euler(ekf.x, &before[0], &before[1], &before[2]);

	zassert_ok(ekf_update_mag(&ekf, mag_b));
	quat_to_euler(ekf.x, &after[0], &after[1], &after[2]);

	zassert_true(fabsf(after[0] - before[0]) * RAD_TO_DEG < 0.25f, "roll moved %d mdeg",
		     to_mdeg((after[0] - before[0]) * RAD_TO_DEG));
	zassert_true(fabsf(after[1] - before[1]) * RAD_TO_DEG < 0.25f, "pitch moved %d mdeg",
		     to_mdeg((after[1] - before[1]) * RAD_TO_DEG));
	zassert_true(after[2] > before[2], "yaw moved away from the measurement");
}

ZTEST(ekf_core, test_gyro_saturation_recovers_after_fast_roll)
{
	const float check_from_s = ROLL_START_S + ROLL_TIME_S + 0.5f;
	float max_error_deg[3];
	struct ekf_stats stats;

	set_pose_deg(0.0f, 0.0f, 30.0f);
	sim_init(&sim, motion_fast_roll, NULL);
	sim.gyro_limit = 8.727f;
	start_from_stationary_alignment();

	run_and_track(check_from_s, check_from_s + 2.0f, max_error_deg);

	ekf_get_stats(&ekf, &stats);
	zassert_true(stats.gyro_sat > 0);
	for (int i = 0; i < 3; i++) {
		zassert_true(max_error_deg[i] < 2.0f, "axis %d error %d mdeg", i,
			     to_mdeg(max_error_deg[i]));
	}
}

ZTEST(ekf_core, test_corrupted_covariance_drops_filter_to_aligning)
{
	const float gyro[3] = { 0.0f, 0.0f, 0.0f };
	const float accel[3] = { 0.0f, 0.0f, -EKF_G0 };
	struct ekf_stats stats;

	zassert_ok(ekf_init(&ekf, &params, v1_q, zero_bias, true));
	ekf.P[0] = NAN;

	zassert_equal(ekf_predict(&ekf, gyro, SIM_DT_S), -EDOM);
	ekf_get_stats(&ekf, &stats);
	zassert_equal(stats.numeric_faults, 1);
	zassert_equal(ekf.state, EKF_STATE_ALIGNING);

	/* A filter waiting for alignment accepts no samples. */
	zassert_equal(ekf_predict(&ekf, gyro, SIM_DT_S), -EINVAL);
	zassert_equal(ekf_update_accel(&ekf, accel), -EINVAL);
}

ZTEST(ekf_core, test_accel_update_skipped_above_hard_gate)
{
	const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	const float heavy[3] = { 0.0f, 0.0f, -1.25f * EKF_G0 };
	const float soft[3] = { 0.0f, 0.0f, -1.10f * EKF_G0 };
	static struct ekf before;

	zassert_ok(ekf_init(&ekf, &params, identity, zero_bias, true));
	memcpy(&before, &ekf, sizeof(ekf));

	zassert_equal(ekf_update_accel(&ekf, heavy), -EAGAIN);
	zassert_equal(ekf.stats.acc_gated, 1);
	zassert_mem_equal(ekf.x, before.x, sizeof(ekf.x));
	zassert_mem_equal(ekf.P, before.P, sizeof(ekf.P));

	zassert_ok(ekf_update_accel(&ekf, soft));
	zassert_equal(ekf.stats.acc_soft, 1);
	zassert_equal(ekf.stats.acc_gated, 1);
}

ZTEST(ekf_core, test_persistent_accel_rejection_resets_tilt)
{
	const float gyro[3] = { 0.0f, 0.0f, 0.0f };
	const float level[3] = { 0.0f, 0.0f, -EKF_G0 };
	float q[4], roll, pitch, yaw;

	/* 40 degrees of roll error is far outside the innovation gate. */
	quat_from_euler(40.0f * SIM_DEG, 0.0f, 25.0f * SIM_DEG, q);
	zassert_ok(ekf_init(&ekf, &params, q, zero_bias, true));

	for (uint32_t n = 1; n < EKF_TILT_RESET_REJECTS; n++) {
		zassert_ok(ekf_predict(&ekf, gyro, SIM_DT_S));
		zassert_equal(ekf_update_accel(&ekf, level), -EAGAIN);
	}
	zassert_equal(ekf.stats.tilt_resets, 0);
	zassert_equal(ekf.stats.acc_rejected, EKF_TILT_RESET_REJECTS - 1U);

	zassert_ok(ekf_predict(&ekf, gyro, SIM_DT_S));
	zassert_equal(ekf_update_accel(&ekf, level), -EAGAIN);
	zassert_equal(ekf.stats.tilt_resets, 1);

	quat_to_euler(ekf.x, &roll, &pitch, &yaw);
	zassert_within(roll, 0.0f, 1e-4f);
	zassert_within(pitch, 0.0f, 1e-4f);
	zassert_within(yaw, 25.0f * SIM_DEG, 1e-4f);
}

ZTEST(ekf_core, test_mag_update_gates)
{
	const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	const float good[3] = { sim_field_n[0], sim_field_n[1], sim_field_n[2] };
	const float strong[3] = { 1.3f * good[0], 1.3f * good[1], 1.3f * good[2] };
	const float flat[3] = { SIM_FIELD_NORM, 0.0f, 0.0f };
	float nose_up[4];

	zassert_ok(ekf_init(&ekf, &params, identity, zero_bias, true));

	zassert_equal(ekf_update_mag(&ekf, strong), -EAGAIN, "norm gate");
	zassert_equal(ekf_update_mag(&ekf, flat), -EAGAIN, "dip gate");
	zassert_equal(ekf.stats.mag_gated, 2);

	ekf_set_use_mag(&ekf, false);
	zassert_equal(ekf_update_mag(&ekf, good), -EAGAIN, "disabled");
	zassert_equal(ekf.stats.mag_gated, 3);
	zassert_false(ekf.yaw_valid);

	ekf_set_use_mag(&ekf, true);
	zassert_ok(ekf_update_mag(&ekf, good));
	zassert_true(ekf.yaw_valid);
	zassert_equal(ekf.stats.mag_rejected, 0);

	/* Above 80 degrees of pitch yaw is not defined well enough to correct. */
	quat_from_euler(0.0f, 85.0f * SIM_DEG, 0.0f, nose_up);
	zassert_ok(ekf_init(&ekf, &params, nose_up, zero_bias, true));
	zassert_equal(ekf_update_mag(&ekf, good), -EAGAIN, "singularity gate");
	zassert_equal(ekf.stats.mag_gated, 4);
}

ZTEST(ekf_core, test_init_and_align_reject_bad_arguments)
{
	const float zero_q[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	const float nan_q[4] = { NAN, 0.0f, 0.0f, 0.0f };
	const float accel[3] = { 0.0f, 0.0f, -EKF_G0 };
	const float zero_vector[3] = { 0.0f, 0.0f, 0.0f };
	struct ekf_params no_field = params;
	float q[4];

	no_field.field_norm_g = 0.0f;

	zassert_equal(ekf_init(&ekf, &params, nan_q, zero_bias, true), -EINVAL);
	zassert_equal(ekf_init(&ekf, &no_field, v1_q, zero_bias, true), -EINVAL);
	zassert_equal(ekf_init(&ekf, &params, zero_q, zero_bias, true), -EDOM);

	zassert_equal(ekf_align(zero_vector, zero_vector, false, 0.0f, q), -EINVAL);
	zassert_equal(ekf_align(accel, zero_vector, true, 0.0f, q), -EINVAL);
	zassert_ok(ekf_align(accel, zero_vector, false, 0.0f, q));
}

ZTEST_SUITE(ekf_core, NULL, NULL, reset_fixture, NULL, NULL);
