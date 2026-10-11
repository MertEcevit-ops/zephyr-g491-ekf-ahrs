#include "ekf_math.h"

#include <errno.h>
#include <math.h>

#include "ekf_config.h"

void quat_multiply(const float p[4], const float r[4], float out[4])
{
	out[0] = p[0] * r[0] - p[1] * r[1] - p[2] * r[2] - p[3] * r[3];
	out[1] = p[0] * r[1] + p[1] * r[0] + p[2] * r[3] - p[3] * r[2];
	out[2] = p[0] * r[2] - p[1] * r[3] + p[2] * r[0] + p[3] * r[1];
	out[3] = p[0] * r[3] + p[1] * r[2] - p[2] * r[1] + p[3] * r[0];
}

int quat_normalize(float q[4], float *norm_before)
{
	float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	/* The negated comparison also rejects NaN. */
	if (!(norm > EKF_QUAT_NORM_MIN)) {
		return -EDOM;
	}

	for (int i = 0; i < 4; i++) {
		q[i] /= norm;
	}

	*norm_before = norm;
	return 0;
}

void quat_from_rotation_vector(const float rotvec[3], float dq[4])
{
	float angle = vec3_norm(rotvec);
	float vector_scale;

	if (angle < EKF_QEXP_SMALL_ANGLE) {
		/* sin(a/2)/a would lose all precision here; use its Taylor series. */
		dq[0] = 1.0f - angle * angle / 8.0f;
		vector_scale = 0.5f - angle * angle / 48.0f;
	} else {
		dq[0] = cosf(0.5f * angle);
		vector_scale = sinf(0.5f * angle) / angle;
	}

	dq[1] = vector_scale * rotvec[0];
	dq[2] = vector_scale * rotvec[1];
	dq[3] = vector_scale * rotvec[2];
}

void quat_to_rotation_matrix(const float q[4], float rot[3][3])
{
	float q0q0 = q[0] * q[0], q1q1 = q[1] * q[1], q2q2 = q[2] * q[2], q3q3 = q[3] * q[3];
	float q0q1 = q[0] * q[1], q0q2 = q[0] * q[2], q0q3 = q[0] * q[3];
	float q1q2 = q[1] * q[2], q1q3 = q[1] * q[3], q2q3 = q[2] * q[3];

	rot[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
	rot[0][1] = 2.0f * (q1q2 - q0q3);
	rot[0][2] = 2.0f * (q1q3 + q0q2);
	rot[1][0] = 2.0f * (q1q2 + q0q3);
	rot[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
	rot[1][2] = 2.0f * (q2q3 - q0q1);
	rot[2][0] = 2.0f * (q1q3 - q0q2);
	rot[2][1] = 2.0f * (q2q3 + q0q1);
	rot[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

void quat_from_euler(float roll, float pitch, float yaw, float q[4])
{
	float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
	float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
	float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);

	q[0] = cr * cp * cy + sr * sp * sy;
	q[1] = sr * cp * cy - cr * sp * sy;
	q[2] = cr * sp * cy + sr * cp * sy;
	q[3] = cr * cp * sy - sr * sp * cy;
}

void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
	float sin_pitch = 2.0f * (q[0] * q[2] - q[1] * q[3]);

	/* Rounding can push the argument just outside asinf()'s domain. */
	sin_pitch = fminf(1.0f, fmaxf(-1.0f, sin_pitch));

	*roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	*pitch = asinf(sin_pitch);
	*yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

void quat_omega_matrix(const float w[3], float omega[4][4])
{
	const float rows[4][4] = {
		{ 0.0f, -w[0], -w[1], -w[2] },
		{ w[0], 0.0f, w[2], -w[1] },
		{ w[1], -w[2], 0.0f, w[0] },
		{ w[2], w[1], -w[0], 0.0f },
	};

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			omega[row][col] = rows[row][col];
		}
	}
}

void quat_xi_matrix(const float q[4], float xi[4][3])
{
	const float rows[4][3] = {
		{ -q[1], -q[2], -q[3] },
		{ q[0], -q[3], q[2] },
		{ q[3], q[0], -q[1] },
		{ -q[2], q[1], q[0] },
	};

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 3; col++) {
			xi[row][col] = rows[row][col];
		}
	}
}

void ekf_gravity_direction(const float q[4], float predicted[3], float jacobian[3][4])
{
	/* An accelerometer at rest reads -g, i.e. minus the third row of R(q). */
	const float rows[3][4] = {
		{ 2.0f * q[2], -2.0f * q[3], 2.0f * q[0], -2.0f * q[1] },
		{ -2.0f * q[1], -2.0f * q[0], -2.0f * q[3], -2.0f * q[2] },
		{ -2.0f * q[0], 2.0f * q[1], 2.0f * q[2], -2.0f * q[3] },
	};

	predicted[0] = -2.0f * (q[1] * q[3] - q[0] * q[2]);
	predicted[1] = -2.0f * (q[2] * q[3] + q[0] * q[1]);
	predicted[2] = -(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 4; col++) {
			jacobian[row][col] = rows[row][col];
		}
	}
}

void ekf_heading_jacobian(const float q[4], float jacobian[4], float *cos2_pitch)
{
	/* yaw = atan2(a, b) and a^2 + b^2 = cos^2(pitch) for a unit quaternion. */
	float a = 2.0f * (q[0] * q[3] + q[1] * q[2]);
	float b = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
	float denominator = a * a + b * b;

	*cos2_pitch = denominator;

	jacobian[0] = 2.0f * b * q[3] / denominator;
	jacobian[1] = 2.0f * b * q[2] / denominator;
	jacobian[2] = (2.0f * b * q[1] + 4.0f * a * q[2]) / denominator;
	jacobian[3] = (2.0f * b * q[0] + 4.0f * a * q[3]) / denominator;
}

float ekf_wrap_pi(float angle)
{
	float wrapped = fmodf(angle, 2.0f * EKF_PI);

	if (wrapped > EKF_PI) {
		wrapped -= 2.0f * EKF_PI;
	} else if (wrapped <= -EKF_PI) {
		wrapped += 2.0f * EKF_PI;
	}

	return wrapped;
}

float vec3_norm(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool ekf_all_finite(const float *values, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (!isfinite(values[i])) {
			return false;
		}
	}

	return true;
}
