/*
 * 7-state attitude EKF: orientation quaternion and gyroscope bias.
 *
 * Equations and constants: MATH_SPEC.md sections 5 to 11. All vectors are in
 * the body frame (FRD); units are rad/s, m/s^2, gauss and seconds.
 *
 * The filter shares one static workspace, so a single thread must own every
 * struct ekf in the system.
 */
#ifndef EKF_CORE_H_
#define EKF_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#include <arm_math.h>

#define EKF_STATE_DIM 7

enum ekf_state {
	EKF_STATE_ALIGNING = 0, /* needs ekf_init() before it accepts samples */
	EKF_STATE_RUNNING,
};

struct ekf_params {
	float declination_rad; /* D, east positive */
	float inclination_rad; /* I, down positive */
	float field_norm_g;    /* B_ref */
	bool use_mag;          /* false also when no magnetometer calibration exists */
};

/* Health counters; they wrap on overflow and survive ekf_init(). */
struct ekf_stats {
	uint32_t acc_soft;
	uint32_t acc_gated;
	uint32_t acc_rejected;
	uint32_t mag_gated;
	uint32_t mag_rejected;
	uint32_t yaw_resets;
	uint32_t tilt_resets;
	uint32_t gyro_sat;
	uint32_t numeric_faults;
};

struct ekf {
	float32_t x[EKF_STATE_DIM];                 /* q0..q3, gyro bias x/y/z */
	float32_t P[EKF_STATE_DIM * EKF_STATE_DIM]; /* row-major covariance */
	struct ekf_params params;
	struct ekf_stats stats;
	uint32_t acc_reject_run;
	uint32_t mag_reject_run;
	uint32_t mag_accept_run;
	enum ekf_state state;
	bool heading_converged;
	bool yaw_valid; /* heading has been tied to the magnetometer at least once */
};

/**
 * Start the filter from a known attitude and bias.
 *
 * @param stationary true when b0 was averaged at rest; selects the tight bias prior
 * @retval -EINVAL non-finite input or non-positive field norm
 * @retval -EDOM q0 cannot be normalised
 */
int ekf_init(struct ekf *ekf, const struct ekf_params *params, const float q0[4],
	     const float b0[3], bool stationary);

/**
 * Propagate attitude and covariance with one gyroscope sample.
 *
 * @retval -EINVAL filter not running, non-finite input or dt outside (EKF_DT_MIN, EKF_DT_MAX]
 * @retval -EDOM numerical fault; the filter drops back to EKF_STATE_ALIGNING
 */
int ekf_predict(struct ekf *ekf, const float gyro_b[3], float dt_s);

/**
 * Correct roll and pitch with the specific-force direction.
 *
 * @retval -EAGAIN sample gated or rejected (counted, not an error)
 * @retval -EINVAL filter not running, non-finite or near-zero input
 * @retval -EDOM numerical fault; the filter drops back to EKF_STATE_ALIGNING
 */
int ekf_update_accel(struct ekf *ekf, const float accel_b[3]);

/**
 * Correct heading with a calibrated magnetic field sample.
 *
 * Return values as for ekf_update_accel().
 */
int ekf_update_mag(struct ekf *ekf, const float mag_b[3]);

/**
 * Attitude from averaged accelerometer and magnetometer readings taken at rest.
 *
 * @param mag_ok false leaves yaw at zero and ignores mag_mean
 * @retval -EINVAL non-finite or near-zero input
 */
int ekf_align(const float accel_mean[3], const float mag_mean[3], bool mag_ok,
	      float declination_rad, float q_out[4]);

void ekf_set_use_mag(struct ekf *ekf, bool use_mag);
void ekf_get_stats(const struct ekf *ekf, struct ekf_stats *out);

#endif /* EKF_CORE_H_ */
