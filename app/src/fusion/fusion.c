#include "fusion.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "ekf/ekf_config.h"
#include "ekf/ekf_math.h"
#include "sensor_hub/mag_cal.h"
#include "sensor_hub/sensor_hub.h"

LOG_MODULE_REGISTER(fusion, CONFIG_APP_LOG_LEVEL);

#define MDEG_TO_RAD (EKF_PI / 180000.0f)

/* Sums over one alignment window (MATH_SPEC.md section 6). */
struct align_window {
	float accel_sum[3];
	float gyro_sum[3];
	float gyro_sq_sum[3];
	float mag_sum[3];
	uint32_t samples;
	uint32_t mag_samples;
	uint32_t moving_windows; /* consecutive windows that failed the stationarity test */
};

static atomic_t align_requested;
static atomic_t mag_enabled = ATOMIC_INIT(1);

/* Owned by the fusion thread. */
static struct ekf ekf;
static struct ekf_params params;
static struct align_window window;
static struct fusion_stats stats;
static enum fusion_state state;
static uint32_t sequence;

static struct k_spinlock snapshot_lock;
static struct fusion_snapshot published;

static void restart_window(void)
{
	uint32_t moving_windows = window.moving_windows;

	memset(&window, 0, sizeof(window));
	window.moving_windows = moving_windows;
}

static void enter_aligning(void)
{
	if (state == FUSION_RUNNING) {
		stats.align_restarts++;
	}

	state = FUSION_ALIGNING;
	memset(&window, 0, sizeof(window));
}

static void accumulate(const struct imu_sample *sample)
{
	for (int i = 0; i < 3; i++) {
		window.accel_sum[i] += sample->accel[i];
		window.gyro_sum[i] += sample->gyro[i];
		window.gyro_sq_sum[i] += sample->gyro[i] * sample->gyro[i];
	}
	window.samples++;

	if ((sample->flags & IMU_SAMPLE_MAG_VALID) != 0U) {
		for (int i = 0; i < 3; i++) {
			window.mag_sum[i] += sample->mag[i];
		}
		window.mag_samples++;
	}
}

static bool window_is_stationary(const float accel_mean[3], const float gyro_mean[3])
{
	const float count = (float)window.samples;
	float accel_norm_error = fabsf(vec3_norm(accel_mean) - EKF_G0);

	for (int i = 0; i < 3; i++) {
		float variance = window.gyro_sq_sum[i] / count - gyro_mean[i] * gyro_mean[i];

		if (variance >= EKF_ALIGN_GYRO_STD_MAX * EKF_ALIGN_GYRO_STD_MAX) {
			return false;
		}
	}

	return accel_norm_error < EKF_ALIGN_ACC_NORM_TOL * EKF_G0;
}

static bool mag_mean_is_usable(const float mag_mean[3])
{
	float norm_error = fabsf(vec3_norm(mag_mean) - params.field_norm_g) / params.field_norm_g;

	return params.use_mag && window.mag_samples >= EKF_ALIGN_MAG_MIN &&
	       norm_error <= EKF_MAG_NORM_GATE;
}

/* Turns a full window into an initial state, or restarts it if the board moved. */
static void finish_alignment(void)
{
	const float zero_bias[3] = { 0.0f, 0.0f, 0.0f };
	float accel_mean[3], gyro_mean[3], mag_mean[3] = { 0.0f, 0.0f, 0.0f };
	float q[4];
	bool stationary, mag_ok;

	for (int i = 0; i < 3; i++) {
		accel_mean[i] = window.accel_sum[i] / (float)window.samples;
		gyro_mean[i] = window.gyro_sum[i] / (float)window.samples;
		if (window.mag_samples > 0U) {
			mag_mean[i] = window.mag_sum[i] / (float)window.mag_samples;
		}
	}

	stationary = window_is_stationary(accel_mean, gyro_mean);
	if (!stationary) {
		window.moving_windows++;
		if (window.moving_windows < EKF_ALIGN_MAX_WINDOWS) {
			restart_window();
			return;
		}
		/* The board will not hold still; start from this window with wide priors. */
	}

	mag_ok = mag_mean_is_usable(mag_mean);
	if (ekf_align(accel_mean, mag_mean, mag_ok, params.declination_rad, q) != 0 ||
	    ekf_init(&ekf, &params, q, stationary ? gyro_mean : zero_bias, stationary) != 0) {
		restart_window();
		return;
	}

	state = FUSION_RUNNING;
	LOG_INF("aligned (%s, heading from %s)", stationary ? "at rest" : "moving",
		mag_ok ? "magnetometer" : "zero");
}

static void align_step(const struct imu_sample *sample)
{
	accumulate(sample);

	if (window.samples >= EKF_ALIGN_SAMPLES) {
		finish_alignment();
	}
}

/* MATH_SPEC 11 call order. -EAGAIN from an update only means the sample was gated. */
static int run_filter(const struct imu_sample *sample, bool *mag_accepted)
{
	int rc = ekf_predict(&ekf, sample->gyro, sample->dt_s);

	if (rc != 0) {
		return rc;
	}

	rc = ekf_update_accel(&ekf, sample->accel);
	if (rc != 0 && rc != -EAGAIN) {
		return rc;
	}

	if ((sample->flags & IMU_SAMPLE_MAG_VALID) == 0U) {
		return 0;
	}

	rc = ekf_update_mag(&ekf, sample->mag);
	if (rc != 0 && rc != -EAGAIN) {
		return rc;
	}

	*mag_accepted = (rc == 0);
	return 0;
}

static void filter_step(const struct imu_sample *sample, bool *mag_accepted)
{
	int rc = run_filter(sample, mag_accepted);

	if (rc == -EINVAL) {
		stats.ekf_input_faults++;
	} else if (rc == -EDOM) {
		stats.ekf_numeric_faults++;
		LOG_ERR("EKF numerical fault, re-aligning");
		enter_aligning();
	}
}

static void apply_requests(void)
{
	bool use_mag = IS_ENABLED(CONFIG_APP_EKF_USE_MAG) && (MAG_CAL_VALID != 0) &&
		       (atomic_get(&mag_enabled) != 0);

	if (use_mag != params.use_mag) {
		params.use_mag = use_mag;
		if (state == FUSION_RUNNING) {
			ekf_set_use_mag(&ekf, use_mag);
		}
	}

	if (atomic_cas(&align_requested, 1, 0)) {
		enter_aligning();
	}
}

static void publish_snapshot(uint32_t ekf_cyc, uint32_t latency_cyc, bool mag_accepted)
{
	struct fusion_snapshot next = {
		.seq = sequence,
		.timestamp_ms = k_uptime_get_32(),
		.state = state,
		.fusion = stats,
		.ekf_cyc = ekf_cyc,
		.latency_cyc = latency_cyc,
	};
	k_spinlock_key_t key;

	memcpy(next.q, &ekf.x[0], sizeof(next.q));
	memcpy(next.gyro_bias, &ekf.x[4], sizeof(next.gyro_bias));
	ekf_get_stats(&ekf, &next.ekf);

	if (state == FUSION_RUNNING) {
		next.flags = (ekf.heading_converged ? FUSION_FLAG_HEADING_CONVERGED : 0U) |
			     (ekf.yaw_valid ? FUSION_FLAG_YAW_VALID : 0U) |
			     (mag_accepted ? FUSION_FLAG_MAG_ACCEPTED : 0U);
	}

	key = k_spin_lock(&snapshot_lock);
	published = next;
	k_spin_unlock(&snapshot_lock, key);
}

void fusion_run(void *p1, void *p2, void *p3)
{
	struct imu_sample sample;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	params.declination_rad = (float)CONFIG_APP_MAG_DECLINATION_MDEG * MDEG_TO_RAD;
	params.inclination_rad = (float)CONFIG_APP_MAG_INCLINATION_MDEG * MDEG_TO_RAD;
	params.field_norm_g = MAG_CAL_FIELD_NORM_G;

	while (true) {
		bool mag_accepted = false;
		uint32_t start_cyc, end_cyc;

		if (sensor_hub_get_sample(&sample, K_FOREVER) != 0) {
			continue;
		}

		apply_requests();

		start_cyc = k_cycle_get_32();
		if (state == FUSION_ALIGNING) {
			align_step(&sample);
		} else {
			filter_step(&sample, &mag_accepted);
		}
		end_cyc = k_cycle_get_32();

		sequence++;
		publish_snapshot(end_cyc - start_cyc, end_cyc - sample.timestamp_cyc, mag_accepted);
	}
}

void fusion_get_snapshot(struct fusion_snapshot *out)
{
	k_spinlock_key_t key = k_spin_lock(&snapshot_lock);

	*out = published;
	k_spin_unlock(&snapshot_lock, key);
}

void fusion_request_align(void)
{
	atomic_set(&align_requested, 1);
}

void fusion_set_use_mag(bool enable)
{
	atomic_set(&mag_enabled, enable ? 1 : 0);
}
