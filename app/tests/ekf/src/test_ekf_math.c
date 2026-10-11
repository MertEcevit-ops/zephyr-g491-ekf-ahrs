/* TEST_PLAN.md section 3.2, T-MATH-01..10. */
#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include "ekf_math.h"
#include "sim.h"

static const float v1_q[4] = { 0.8616424f, 0.2996729f, -0.0574224f, 0.4055504f };

static void assert_vec_near(const float *actual, const float *expected, int count, float tol)
{
	for (int i = 0; i < count; i++) {
		zassert_within(actual[i], expected[i], tol, "element %d", i);
	}
}

static void random_unit_quat(uint32_t *seed, float q[4])
{
	float norm;

	for (int i = 0; i < 4; i++) {
		q[i] = sim_randn(seed);
	}
	zassert_ok(quat_normalize(q, &norm));
}

ZTEST(ekf_math, test_quat_multiply_identities)
{
	const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	const float i_unit[4] = { 0.0f, 1.0f, 0.0f, 0.0f };
	const float j_unit[4] = { 0.0f, 0.0f, 1.0f, 0.0f };
	const float k_unit[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	const float v1_conj[4] = { v1_q[0], -v1_q[1], -v1_q[2], -v1_q[3] };
	float out[4];

	quat_multiply(v1_q, identity, out);
	assert_vec_near(out, v1_q, 4, 1e-6f);

	quat_multiply(i_unit, j_unit, out);
	assert_vec_near(out, k_unit, 4, 1e-6f);

	quat_multiply(v1_q, v1_conj, out);
	assert_vec_near(out, identity, 4, 1e-6f);
}

ZTEST(ekf_math, test_quat_normalize_scales_and_rejects_zero)
{
	const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
	float q[4] = { 2.0f, 0.0f, 0.0f, 0.0f };
	float zero[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	float norm;

	zassert_ok(quat_normalize(q, &norm));
	assert_vec_near(q, identity, 4, 1e-6f);
	zassert_within(norm, 2.0f, 1e-6f);

	zassert_equal(quat_normalize(zero, &norm), -EDOM);
}

ZTEST(ekf_math, test_euler_to_quat_and_rotation_match_reference)
{
	const float v2_rot[3][3] = {
		{ 0.6644630f, -0.7332948f, 0.1441097f },
		{ 0.6644630f, 0.4914501f, -0.5629971f },
		{ 0.3420201f, 0.4698463f, 0.8137977f },
	};
	float q[4], rot[3][3];

	quat_from_euler(30.0f * SIM_DEG, -20.0f * SIM_DEG, 45.0f * SIM_DEG, q);
	assert_vec_near(q, v1_q, 4, 1e-5f);

	quat_to_rotation_matrix(q, rot);
	for (int row = 0; row < 3; row++) {
		assert_vec_near(rot[row], v2_rot[row], 3, 1e-5f);
	}
}

ZTEST(ekf_math, test_euler_round_trip_grid)
{
	float max_error = 0.0f;

	for (int roll = -170; roll <= 170; roll += 10) {
		for (int pitch = -80; pitch <= 80; pitch += 10) {
			for (int yaw = -170; yaw <= 170; yaw += 10) {
				const float in[3] = { (float)roll * SIM_DEG, (float)pitch * SIM_DEG,
						      (float)yaw * SIM_DEG };
				float q[4], out[3];

				quat_from_euler(in[0], in[1], in[2], q);
				quat_to_euler(q, &out[0], &out[1], &out[2]);
				for (int i = 0; i < 3; i++) {
					max_error = fmaxf(max_error, fabsf(out[i] - in[i]));
				}
			}
		}
	}

	zassert_true(max_error < 2e-4f, "max error %d urad", (int)(max_error * 1e6f));
}

ZTEST(ekf_math, test_rotation_matrix_matches_quaternion_sandwich)
{
	uint32_t seed = 0x12345678U;

	for (int trial = 0; trial < 100; trial++) {
		float q[4], rot[3][3], tmp[4], sandwich[4], rotated[3];
		float v[4] = { 0.0f, sim_randn(&seed), sim_randn(&seed), sim_randn(&seed) };

		random_unit_quat(&seed, q);
		const float q_conj[4] = { q[0], -q[1], -q[2], -q[3] };

		quat_to_rotation_matrix(q, rot);
		for (int row = 0; row < 3; row++) {
			rotated[row] = rot[row][0] * v[1] + rot[row][1] * v[2] + rot[row][2] * v[3];
		}

		quat_multiply(q, v, tmp);
		quat_multiply(tmp, q_conj, sandwich);
		assert_vec_near(rotated, &sandwich[1], 3, 1e-5f);
	}
}

ZTEST(ekf_math, test_rotation_vector_exponential)
{
	const float v7_rotvec[3] = { 0.1f * 0.01f, -0.2f * 0.01f, 0.3f * 0.01f };
	const float v7_dq[4] = { 0.9999983f, 0.0005000f, -0.0010000f, 0.0015000f };
	const float v7_product[4] = { 0.8608253f, 0.3004226f, -0.0585307f, 0.4065712f };
	const float tiny[3] = { 1e-9f, 0.0f, 0.0f };
	const float half_turn[3] = { EKF_PI, 0.0f, 0.0f };
	const float half_turn_q[4] = { 0.0f, 1.0f, 0.0f, 0.0f };
	const float below[3] = { 0.9999e-3f, 0.0f, 0.0f };
	const float above[3] = { 1.0001e-3f, 0.0f, 0.0f };
	float dq[4], product[4], dq_below[4], dq_above[4];

	quat_from_rotation_vector(v7_rotvec, dq);
	assert_vec_near(dq, v7_dq, 4, 1e-6f);
	quat_multiply(v1_q, dq, product);
	assert_vec_near(product, v7_product, 4, 1e-6f);

	quat_from_rotation_vector(tiny, dq);
	zassert_true(ekf_all_finite(dq, 4));
	zassert_within(dq[0], 1.0f, 1e-6f);

	quat_from_rotation_vector(half_turn, dq);
	assert_vec_near(dq, half_turn_q, 4, 1e-6f);

	/* The series and the closed form must meet at the switch-over angle. */
	quat_from_rotation_vector(below, dq_below);
	quat_from_rotation_vector(above, dq_above);
	zassert_within(dq_below[0], dq_above[0], 1e-7f);
	zassert_within(dq_below[1] / below[0], dq_above[1] / above[0], 1e-7f);
}

static float yaw_of(const float q[4])
{
	float roll, pitch, yaw;

	quat_to_euler(q, &roll, &pitch, &yaw);
	return yaw;
}

ZTEST(ekf_math, test_jacobians_match_reference_and_finite_differences)
{
	const float v6_gravity[3][4] = {
		{ -0.1148449f, -0.8111009f, 1.7232849f, -0.5993457f },
		{ -0.5993457f, -1.7232849f, -0.8111009f, 0.1148449f },
		{ -1.7232849f, 0.5993457f, -0.1148449f, -0.8111009f },
	};
	const float v6_heading[4] = { 0.6103431f, -0.0864193f, 0.2781614f, 2.5174362f };
	const float step = 1e-3f;
	uint32_t seed = 0x12345678U;
	float predicted[3], gravity_jac[3][4], heading_jac[4], cos2_pitch;

	ekf_gravity_direction(v1_q, predicted, gravity_jac);
	ekf_heading_jacobian(v1_q, heading_jac, &cos2_pitch);
	for (int row = 0; row < 3; row++) {
		assert_vec_near(gravity_jac[row], v6_gravity[row], 4, 1e-4f);
	}
	assert_vec_near(heading_jac, v6_heading, 4, 1e-4f);

	for (int trial = 0; trial < 50; trial++) {
		float q[4];

		random_unit_quat(&seed, q);
		ekf_gravity_direction(q, predicted, gravity_jac);
		ekf_heading_jacobian(q, heading_jac, &cos2_pitch);

		for (int col = 0; col < 4; col++) {
			float plus[4] = { q[0], q[1], q[2], q[3] };
			float minus[4] = { q[0], q[1], q[2], q[3] };
			float h_plus[3], h_minus[3], unused[3][4];

			plus[col] += step;
			minus[col] -= step;
			ekf_gravity_direction(plus, h_plus, unused);
			ekf_gravity_direction(minus, h_minus, unused);
			for (int row = 0; row < 3; row++) {
				zassert_within(gravity_jac[row][col],
					       (h_plus[row] - h_minus[row]) / (2.0f * step), 2e-3f);
			}

			/* Yaw is ill-conditioned near the poles; the filter gates there too. */
			if (cos2_pitch > 0.25f) {
				float slope = ekf_wrap_pi(yaw_of(plus) - yaw_of(minus)) / (2.0f * step);

				zassert_within(heading_jac[col], slope, 2e-3f * (1.0f + fabsf(slope)));
			}
		}
	}
}

ZTEST(ekf_math, test_omega_and_xi_identities)
{
	const float w[3] = { 0.3f, -0.7f, 1.1f };
	const float w_quat[4] = { 0.0f, w[0], w[1], w[2] };
	float omega[4][4], xi[4][3], product[4];

	quat_multiply(v1_q, w_quat, product);
	quat_omega_matrix(w, omega);
	quat_xi_matrix(v1_q, xi);

	for (int row = 0; row < 4; row++) {
		float via_omega = 0.0f, via_xi = 0.0f;

		for (int col = 0; col < 4; col++) {
			via_omega += omega[row][col] * v1_q[col];
		}
		for (int col = 0; col < 3; col++) {
			via_xi += xi[row][col] * w[col];
		}
		zassert_within(via_omega, product[row], 1e-6f);
		zassert_within(via_xi, product[row], 1e-6f);
	}

	for (int col = 0; col < 3; col++) {
		float dot = 0.0f;

		for (int row = 0; row < 4; row++) {
			dot += xi[row][col] * v1_q[row];
		}
		zassert_within(dot, 0.0f, 1e-6f);
	}
}

ZTEST(ekf_math, test_euler_at_gimbal_lock_stays_finite)
{
	for (int sign = -1; sign <= 1; sign += 2) {
		float q[4], euler[3];

		quat_from_euler(0.3f, (float)sign * 0.5f * EKF_PI, -0.4f, q);
		quat_to_euler(q, &euler[0], &euler[1], &euler[2]);

		zassert_true(ekf_all_finite(euler, 3));
		zassert_within(fabsf(euler[1]), 0.5f * EKF_PI, 1e-3f);
	}
}

ZTEST(ekf_math, test_wrap_pi_range)
{
	zassert_within(ekf_wrap_pi(EKF_PI + 0.1f), -EKF_PI + 0.1f, 1e-6f);
	zassert_within(ekf_wrap_pi(-EKF_PI), EKF_PI, 1e-6f);
	zassert_within(ekf_wrap_pi(0.5f), 0.5f, 1e-6f);
	zassert_within(ekf_wrap_pi(7.0f * EKF_PI + 0.2f), -EKF_PI + 0.2f, 1e-5f);
}

ZTEST_SUITE(ekf_math, NULL, NULL, NULL, NULL, NULL);
