#include "ekf_core.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "ekf_config.h"
#include "ekf_math.h"

#define N        EKF_STATE_DIM
#define QUAT_DIM 4
#define BIAS_DIM 3
#define MEAS_MAX 3

/*
 * Scratch matrices, all row-major. Kept static because 7x7 float matrices do
 * not belong on a 2 kB thread stack.
 */
struct ekf_workspace {
	float32_t M[N * N];  /* transform applied to P: F, T or (I - K H) */
	float32_t Mt[N * N];
	float32_t A[N * N];
	float32_t B[N * N];
	float32_t Q[N * N];
	float32_t H[MEAS_MAX * N];
	float32_t Ht[N * MEAS_MAX];
	float32_t PHt[N * MEAS_MAX];
	float32_t K[N * MEAS_MAX];
	float32_t Kt[MEAS_MAX * N];
	float32_t KR[N * MEAS_MAX];
	float32_t R[MEAS_MAX * MEAS_MAX];
	float32_t S[MEAS_MAX * MEAS_MAX];
	float32_t S_scratch[MEAS_MAX * MEAS_MAX];
	float32_t S_inv[MEAS_MAX * MEAS_MAX];
};

static struct ekf_workspace ws;

static arm_matrix_instance_f32 matrix(uint16_t rows, uint16_t cols, float32_t *data)
{
	arm_matrix_instance_f32 instance;

	arm_mat_init_f32(&instance, rows, cols, data);
	return instance;
}

static bool mult_ok(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
		    arm_matrix_instance_f32 *out)
{
	return arm_mat_mult_f32(a, b, out) == ARM_MATH_SUCCESS;
}

static bool trans_ok(const arm_matrix_instance_f32 *in, arm_matrix_instance_f32 *out)
{
	return arm_mat_trans_f32(in, out) == ARM_MATH_SUCCESS;
}

static void set_identity(float32_t *m)
{
	memset(m, 0, N * N * sizeof(float32_t));
	for (int i = 0; i < N; i++) {
		m[i * N + i] = 1.0f;
	}
}

/* P <- M P M^T, with M taken from ws.M. */
static int transform_covariance(float32_t *P)
{
	arm_matrix_instance_f32 m = matrix(N, N, ws.M);
	arm_matrix_instance_f32 mt = matrix(N, N, ws.Mt);
	arm_matrix_instance_f32 a = matrix(N, N, ws.A);
	arm_matrix_instance_f32 p = matrix(N, N, P);

	if (!trans_ok(&m, &mt) || !mult_ok(&m, &p, &a) || !mult_ok(&a, &mt, &p)) {
		return -EDOM;
	}

	return 0;
}

static void symmetrize(float32_t *P)
{
	for (int row = 0; row < N; row++) {
		for (int col = row + 1; col < N; col++) {
			float32_t mean = 0.5f * (P[row * N + col] + P[col * N + row]);

			P[row * N + col] = mean;
			P[col * N + row] = mean;
		}
	}
}

/*
 * MATH_SPEC 10: renormalise q and project P onto the tangent space of the unit
 * sphere, so that no variance builds up along the unobservable norm direction.
 */
static int normalize_and_project(struct ekf *ekf)
{
	float32_t *q = ekf->x;
	float norm;

	if (quat_normalize(q, &norm) != 0) {
		return -EDOM;
	}

	set_identity(ws.M);
	for (int row = 0; row < QUAT_DIM; row++) {
		for (int col = 0; col < QUAT_DIM; col++) {
			float identity = (row == col) ? 1.0f : 0.0f;

			ws.M[row * N + col] = (identity - q[row] * q[col]) / norm;
		}
	}

	if (transform_covariance(ekf->P) != 0) {
		return -EDOM;
	}

	symmetrize(ekf->P);
	return 0;
}

/*
 * MATH_SPEC 10 "inflate": forget the attitude but keep the bias estimate, whose
 * information does not depend on how well the attitude is known.
 */
static int inflate_attitude(struct ekf *ekf)
{
	const float32_t variance = EKF_P0_QUAT_SIGMA * EKF_P0_QUAT_SIGMA;

	for (int row = 0; row < N; row++) {
		for (int col = 0; col < N; col++) {
			if (row >= QUAT_DIM && col >= QUAT_DIM) {
				continue;
			}
			ekf->P[row * N + col] = (row == col) ? variance : 0.0f;
		}
	}

	return normalize_and_project(ekf);
}

static bool state_is_sane(const struct ekf *ekf)
{
	if (!ekf_all_finite(ekf->x, N)) {
		return false;
	}

	for (int i = 0; i < N; i++) {
		float32_t variance = ekf->P[i * N + i];

		if (!isfinite(variance) || variance < 0.0f) {
			return false;
		}
	}

	return true;
}

/* Common exit of every step: verify the state and fall back to aligning on a fault. */
static int finish_step(struct ekf *ekf, int rc)
{
	if (rc == 0 && !state_is_sane(ekf)) {
		rc = -EDOM;
	}

	if (rc == -EDOM) {
		ekf->stats.numeric_faults++;
		ekf->state = EKF_STATE_ALIGNING;
	}

	return rc;
}

/* MATH_SPEC 7.4: F is built with q before the attitude update. */
static void build_transition(const float32_t *q, const float rate[3], float dt_s)
{
	float omega[4][4];
	float xi[4][3];

	quat_omega_matrix(rate, omega);
	quat_xi_matrix(q, xi);

	set_identity(ws.M);
	for (int row = 0; row < QUAT_DIM; row++) {
		for (int col = 0; col < QUAT_DIM; col++) {
			ws.M[row * N + col] += 0.5f * dt_s * omega[row][col];
		}
		for (int col = 0; col < BIAS_DIM; col++) {
			ws.M[row * N + QUAT_DIM + col] = -0.5f * dt_s * xi[row][col];
		}
	}
}

/* MATH_SPEC 7.5: angular random walk mapped through Xi, plus bias random walk. */
static void build_process_noise(const float32_t *q, float dt_s)
{
	const float attitude_scale = EKF_GYRO_ARW * EKF_GYRO_ARW * dt_s / 4.0f;
	const float bias_variance = EKF_GYRO_BIAS_RW * EKF_GYRO_BIAS_RW * dt_s;
	float xi[4][3];

	quat_xi_matrix(q, xi);
	memset(ws.Q, 0, sizeof(ws.Q));

	for (int row = 0; row < QUAT_DIM; row++) {
		for (int col = 0; col < QUAT_DIM; col++) {
			float dot = xi[row][0] * xi[col][0] + xi[row][1] * xi[col][1] +
				    xi[row][2] * xi[col][2];

			ws.Q[row * N + col] = attitude_scale * dot;
		}
	}

	for (int i = QUAT_DIM; i < N; i++) {
		ws.Q[i * N + i] = bias_variance;
	}
}

static bool gyro_saturated(const float gyro_b[3])
{
	return fabsf(gyro_b[0]) >= EKF_GYRO_SAT || fabsf(gyro_b[1]) >= EKF_GYRO_SAT ||
	       fabsf(gyro_b[2]) >= EKF_GYRO_SAT;
}

static int propagate(struct ekf *ekf, const float gyro_b[3], float dt_s)
{
	float32_t *q = ekf->x;
	float rate[3], rotvec[3], dq[4], q_next[4];

	for (int i = 0; i < 3; i++) {
		rate[i] = gyro_b[i] - ekf->x[QUAT_DIM + i];
		rotvec[i] = rate[i] * dt_s;
	}

	build_transition(q, rate, dt_s);
	build_process_noise(q, dt_s);

	quat_from_rotation_vector(rotvec, dq);
	quat_multiply(q, dq, q_next);
	memcpy(q, q_next, sizeof(q_next));

	if (transform_covariance(ekf->P) != 0) {
		return -EDOM;
	}
	for (int i = 0; i < N * N; i++) {
		ekf->P[i] += ws.Q[i];
	}

	if (normalize_and_project(ekf) != 0) {
		return -EDOM;
	}

	/*
	 * A clipped gyro has lost part of the real rotation. Opening up the
	 * attitude covariance lets the next corrections recover it at once.
	 */
	if (gyro_saturated(gyro_b)) {
		ekf->stats.gyro_sat++;
		return inflate_attitude(ekf);
	}

	return 0;
}

int ekf_predict(struct ekf *ekf, const float gyro_b[3], float dt_s)
{
	if (ekf->state != EKF_STATE_RUNNING || !ekf_all_finite(gyro_b, 3) || !isfinite(dt_s)) {
		return -EINVAL;
	}
	if (dt_s <= EKF_DT_MIN || dt_s > EKF_DT_MAX) {
		return -EINVAL;
	}

	return finish_step(ekf, propagate(ekf, gyro_b, dt_s));
}

/*
 * MATH_SPEC 10: x <- x + K y and the Joseph-form covariance update, followed by
 * the normalisation step. Expects ws.H (m x 7), ws.K (7 x m) and ws.R (m x m).
 */
static int apply_correction(struct ekf *ekf, uint16_t m, const float innovation[])
{
	arm_matrix_instance_f32 h = matrix(m, N, ws.H);
	arm_matrix_instance_f32 k = matrix(N, m, ws.K);
	arm_matrix_instance_f32 kt = matrix(m, N, ws.Kt);
	arm_matrix_instance_f32 r = matrix(m, m, ws.R);
	arm_matrix_instance_f32 kr = matrix(N, m, ws.KR);
	arm_matrix_instance_f32 kh = matrix(N, N, ws.B);
	arm_matrix_instance_f32 krkt = matrix(N, N, ws.B);

	for (int row = 0; row < N; row++) {
		for (int col = 0; col < m; col++) {
			ekf->x[row] += ws.K[row * m + col] * innovation[col];
		}
	}

	if (!mult_ok(&k, &h, &kh)) {
		return -EDOM;
	}
	set_identity(ws.M);
	for (int i = 0; i < N * N; i++) {
		ws.M[i] -= ws.B[i];
	}
	if (transform_covariance(ekf->P) != 0) {
		return -EDOM;
	}

	if (!mult_ok(&k, &r, &kr) || !trans_ok(&k, &kt) || !mult_ok(&kr, &kt, &krkt)) {
		return -EDOM;
	}
	for (int i = 0; i < N * N; i++) {
		ekf->P[i] += ws.B[i];
	}

	return normalize_and_project(ekf);
}

/* Fills ws.PHt and ws.S = H P H^T + R for the m x 7 Jacobian in ws.H. */
static int innovation_covariance(struct ekf *ekf, uint16_t m)
{
	arm_matrix_instance_f32 h = matrix(m, N, ws.H);
	arm_matrix_instance_f32 ht = matrix(N, m, ws.Ht);
	arm_matrix_instance_f32 p = matrix(N, N, ekf->P);
	arm_matrix_instance_f32 pht = matrix(N, m, ws.PHt);
	arm_matrix_instance_f32 s = matrix(m, m, ws.S);

	if (!trans_ok(&h, &ht) || !mult_ok(&p, &ht, &pht) || !mult_ok(&h, &pht, &s)) {
		return -EDOM;
	}

	for (int i = 0; i < m * m; i++) {
		ws.S[i] += ws.R[i];
	}

	return 0;
}

/* Measurement noise for the gravity direction, inflated in the soft gate zone. */
static float accel_noise_variance(struct ekf *ekf, float norm_error)
{
	float variance = EKF_ACC_SIGMA * EKF_ACC_SIGMA;
	float zone_fraction;

	if (norm_error <= EKF_ACC_GATE_SOFT) {
		return variance;
	}

	ekf->stats.acc_soft++;
	zone_fraction = (norm_error - EKF_ACC_GATE_SOFT) / (EKF_ACC_GATE_HARD - EKF_ACC_GATE_SOFT);

	return variance * (1.0f + (EKF_ACC_R_INFLATE_MAX - 1.0f) * zone_fraction);
}

static void tilt_from_accel(const float accel_b[3], float *roll, float *pitch)
{
	*roll = atan2f(-accel_b[1], -accel_b[2]);
	*pitch = atan2f(accel_b[0], sqrtf(accel_b[1] * accel_b[1] + accel_b[2] * accel_b[2]));
}

/*
 * MATH_SPEC 8 tilt reset: the gravity direction has disagreed with the filter
 * for seconds while its magnitude looked right, so trust it over the gyro.
 */
static int reset_tilt(struct ekf *ekf, const float accel_b[3])
{
	float roll, pitch, yaw, unused;

	quat_to_euler(ekf->x, &unused, &unused, &yaw);
	tilt_from_accel(accel_b, &roll, &pitch);
	quat_from_euler(roll, pitch, yaw, ekf->x);

	ekf->stats.tilt_resets++;
	ekf->acc_reject_run = 0;

	return inflate_attitude(ekf);
}

static int reject_accel(struct ekf *ekf, const float accel_b[3])
{
	ekf->stats.acc_rejected++;
	ekf->acc_reject_run++;

	if (ekf->acc_reject_run >= EKF_TILT_RESET_REJECTS) {
		int rc = reset_tilt(ekf, accel_b);

		return (rc == 0) ? -EAGAIN : rc;
	}

	return -EAGAIN;
}

static int correct_with_accel(struct ekf *ekf, const float accel_b[3], float accel_norm)
{
	arm_matrix_instance_f32 pht = matrix(N, 3, ws.PHt);
	arm_matrix_instance_f32 s_scratch = matrix(3, 3, ws.S_scratch);
	arm_matrix_instance_f32 s_inv = matrix(3, 3, ws.S_inv);
	arm_matrix_instance_f32 k = matrix(N, 3, ws.K);
	float norm_error = fabsf(accel_norm - EKF_G0) / EKF_G0;
	float predicted[3], jacobian[3][4], innovation[3];
	float noise_variance, nis = 0.0f;

	if (norm_error >= EKF_ACC_GATE_HARD) {
		ekf->stats.acc_gated++;
		return -EAGAIN;
	}
	noise_variance = accel_noise_variance(ekf, norm_error);

	ekf_gravity_direction(ekf->x, predicted, jacobian);
	memset(ws.H, 0, sizeof(ws.H));
	memset(ws.R, 0, sizeof(ws.R));
	for (int row = 0; row < 3; row++) {
		innovation[row] = accel_b[row] / accel_norm - predicted[row];
		ws.R[row * 3 + row] = noise_variance;
		memcpy(&ws.H[row * N], jacobian[row], sizeof(jacobian[row]));
	}

	if (innovation_covariance(ekf, 3) != 0) {
		return -EDOM;
	}

	/* arm_mat_inverse_f32() destroys its source, so invert a copy of S. */
	memcpy(ws.S_scratch, ws.S, sizeof(ws.S));
	if (arm_mat_inverse_f32(&s_scratch, &s_inv) != ARM_MATH_SUCCESS) {
		return -EDOM;
	}

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			nis += innovation[row] * ws.S_inv[row * 3 + col] * innovation[col];
		}
	}
	if (nis > EKF_ACC_NIS_MAX) {
		return reject_accel(ekf, accel_b);
	}

	if (!mult_ok(&pht, &s_inv, &k)) {
		return -EDOM;
	}

	ekf->acc_reject_run = 0;
	return apply_correction(ekf, 3, innovation);
}

int ekf_update_accel(struct ekf *ekf, const float accel_b[3])
{
	float accel_norm;

	if (ekf->state != EKF_STATE_RUNNING || !ekf_all_finite(accel_b, 3)) {
		return -EINVAL;
	}

	accel_norm = vec3_norm(accel_b);
	if (accel_norm < EKF_ACC_NORM_MIN) {
		return -EINVAL;
	}

	return finish_step(ekf, correct_with_accel(ekf, accel_b, accel_norm));
}

/* MATH_SPEC 9 gates; true when the sample may be compared against the heading. */
static bool mag_passes_gates(const struct ekf *ekf, float mag_norm, const float mag_n[3],
			     float cos2_pitch)
{
	const struct ekf_params *params = &ekf->params;
	float horizontal = sqrtf(mag_n[0] * mag_n[0] + mag_n[1] * mag_n[1]);
	float dip = atan2f(mag_n[2], horizontal);

	if (!params->use_mag) {
		return false;
	}
	if (fabsf(mag_norm - params->field_norm_g) / params->field_norm_g > EKF_MAG_NORM_GATE) {
		return false;
	}
	if (fabsf(dip - params->inclination_rad) > EKF_MAG_DIP_GATE) {
		return false;
	}

	return cos2_pitch >= EKF_MAG_MIN_COS2_PITCH;
}

static int reset_yaw(struct ekf *ekf, float heading_error)
{
	float roll, pitch, yaw;

	quat_to_euler(ekf->x, &roll, &pitch, &yaw);
	quat_from_euler(roll, pitch, ekf_wrap_pi(yaw + heading_error), ekf->x);

	ekf->stats.yaw_resets++;
	ekf->mag_reject_run = 0;
	ekf->yaw_valid = true;

	return inflate_attitude(ekf);
}

/*
 * MATH_SPEC 9 heading recovery. The Earth field does not move, so a converged
 * filter treats persistent rejection as a local disturbance, while a filter
 * that never converged treats it as a wrong heading and adopts the measurement.
 */
static int reject_mag(struct ekf *ekf, float heading_error)
{
	int rc = 0;

	ekf->stats.mag_rejected++;
	ekf->mag_reject_run++;
	ekf->mag_accept_run = 0;

	if (!ekf->heading_converged) {
		if (ekf->mag_reject_run >= EKF_YAW_RESET_REJECTS) {
			rc = reset_yaw(ekf, heading_error);
		}
	} else if (ekf->mag_reject_run == EKF_YAW_RESET_REJECTS) {
		rc = inflate_attitude(ekf);
	} else if (ekf->mag_reject_run >= EKF_MAG_DISTRUST_REJECTS) {
		ekf->heading_converged = false;
	}

	return (rc == 0) ? -EAGAIN : rc;
}

static void accept_mag(struct ekf *ekf)
{
	ekf->mag_reject_run = 0;
	ekf->mag_accept_run++;
	ekf->yaw_valid = true;

	if (ekf->mag_accept_run >= EKF_MAG_CONVERGE_ACCEPTS) {
		ekf->heading_converged = true;
	}
}

static int correct_with_mag(struct ekf *ekf, const float mag_b[3], float mag_norm)
{
	const float noise_variance = EKF_HEADING_SIGMA * EKF_HEADING_SIGMA;
	float rot[3][3], mag_n[3], jacobian[4];
	float cos2_pitch, heading_error, innovation_variance, nis;

	quat_to_rotation_matrix(ekf->x, rot);
	for (int row = 0; row < 3; row++) {
		mag_n[row] = rot[row][0] * mag_b[0] + rot[row][1] * mag_b[1] + rot[row][2] * mag_b[2];
	}
	ekf_heading_jacobian(ekf->x, jacobian, &cos2_pitch);

	if (!mag_passes_gates(ekf, mag_norm, mag_n, cos2_pitch)) {
		ekf->stats.mag_gated++;
		return -EAGAIN;
	}

	/* Only the horizontal projection of the field carries heading. */
	heading_error = ekf_wrap_pi(ekf->params.declination_rad - atan2f(mag_n[1], mag_n[0]));

	memset(ws.H, 0, sizeof(ws.H));
	memcpy(ws.H, jacobian, sizeof(jacobian));
	ws.R[0] = noise_variance;
	if (innovation_covariance(ekf, 1) != 0) {
		return -EDOM;
	}

	innovation_variance = ws.S[0];
	if (!(innovation_variance > EKF_HEADING_S_MIN)) {
		return -EDOM;
	}

	nis = heading_error * heading_error / innovation_variance;
	if (nis > EKF_MAG_NIS_MAX) {
		return reject_mag(ekf, heading_error);
	}

	for (int row = 0; row < N; row++) {
		ws.K[row] = ws.PHt[row] / innovation_variance;
	}

	accept_mag(ekf);
	return apply_correction(ekf, 1, &heading_error);
}

int ekf_update_mag(struct ekf *ekf, const float mag_b[3])
{
	float mag_norm;

	if (ekf->state != EKF_STATE_RUNNING || !ekf_all_finite(mag_b, 3)) {
		return -EINVAL;
	}

	mag_norm = vec3_norm(mag_b);
	if (mag_norm < EKF_MAG_NORM_MIN) {
		return -EINVAL;
	}

	return finish_step(ekf, correct_with_mag(ekf, mag_b, mag_norm));
}

int ekf_init(struct ekf *ekf, const struct ekf_params *params, const float q0[4],
	     const float b0[3], bool stationary)
{
	const float quat_variance = EKF_P0_QUAT_SIGMA * EKF_P0_QUAT_SIGMA;
	const float bias_sigma = stationary ? EKF_P0_BIAS_SIGMA_STATIC : EKF_P0_BIAS_SIGMA_MOVING;
	float q[4], norm;

	if (!ekf_all_finite(q0, 4) || !ekf_all_finite(b0, 3) ||
	    !isfinite(params->declination_rad) || !isfinite(params->inclination_rad) ||
	    !(params->field_norm_g > 0.0f)) {
		return -EINVAL;
	}

	memcpy(q, q0, sizeof(q));
	if (quat_normalize(q, &norm) != 0) {
		return -EDOM;
	}

	memcpy(&ekf->x[0], q, sizeof(q));
	memcpy(&ekf->x[QUAT_DIM], b0, BIAS_DIM * sizeof(float));

	memset(ekf->P, 0, sizeof(ekf->P));
	for (int i = 0; i < N; i++) {
		ekf->P[i * N + i] = (i < QUAT_DIM) ? quat_variance : bias_sigma * bias_sigma;
	}

	ekf->params = *params;
	ekf->acc_reject_run = 0;
	ekf->mag_reject_run = 0;
	ekf->mag_accept_run = 0;
	ekf->heading_converged = false;
	ekf->yaw_valid = false;
	ekf->state = EKF_STATE_RUNNING;

	return finish_step(ekf, normalize_and_project(ekf));
}

int ekf_align(const float accel_mean[3], const float mag_mean[3], bool mag_ok,
	      float declination_rad, float q_out[4])
{
	float roll, pitch, yaw = 0.0f;

	if (!ekf_all_finite(accel_mean, 3) || vec3_norm(accel_mean) < EKF_ACC_NORM_MIN ||
	    !isfinite(declination_rad)) {
		return -EINVAL;
	}

	tilt_from_accel(accel_mean, &roll, &pitch);

	if (mag_ok) {
		float sr = sinf(roll), cr = cosf(roll), sp = sinf(pitch), cp = cosf(pitch);
		float level_x, level_y;

		if (!ekf_all_finite(mag_mean, 3) || vec3_norm(mag_mean) < EKF_MAG_NORM_MIN) {
			return -EINVAL;
		}

		/* De-rotate roll and pitch: m_level = Ry(pitch) Rx(roll) m_b. */
		level_x = cp * mag_mean[0] + sp * (sr * mag_mean[1] + cr * mag_mean[2]);
		level_y = cr * mag_mean[1] - sr * mag_mean[2];
		yaw = ekf_wrap_pi(declination_rad + atan2f(-level_y, level_x));
	}

	quat_from_euler(roll, pitch, yaw, q_out);
	return 0;
}

void ekf_set_use_mag(struct ekf *ekf, bool use_mag)
{
	ekf->params.use_mag = use_mag;

	if (!use_mag) {
		ekf->heading_converged = false;
		ekf->yaw_valid = false;
		ekf->mag_reject_run = 0;
		ekf->mag_accept_run = 0;
	}
}

void ekf_get_stats(const struct ekf *ekf, struct ekf_stats *out)
{
	*out = ekf->stats;
}
