/* TEST_PLAN.md section 3.4, T-HUB-01..04 and T-HUB-06. */
#include <errno.h>

#include <zephyr/ztest.h>

#include "ekf/ekf_config.h"
#include "sensor_cal.h"

#define CYCLES_PER_SEC 170000000U

/* A signed permutation must not round, so the tolerance is zero. */
static void assert_vec_equal(const float actual[3], float x, float y, float z)
{
	const float expected[3] = { x, y, z };

	for (int i = 0; i < 3; i++) {
		zassert_within(actual[i], expected[i], 0.0f, "axis %d", i);
	}
}

ZTEST(axis_map, test_ism_map_turns_sensor_frame_into_frd)
{
	const float at_rest[3] = { 0.0f, 0.0f, 9.80665f };
	const float ramp[3] = { 1.0f, 2.0f, 3.0f };
	float body[3];

	axis_map_apply(&ism_to_body, at_rest, body);
	assert_vec_equal(body, 0.0f, 0.0f, -9.80665f);

	axis_map_apply(&ism_to_body, ramp, body);
	assert_vec_equal(body, 1.0f, -2.0f, -3.0f);
}

ZTEST(axis_map, test_mmc_map_keeps_z_and_flips_y)
{
	const float ramp[3] = { 1.0f, 2.0f, 3.0f };
	float body[3];

	axis_map_apply(&mmc_to_body, ramp, body);
	assert_vec_equal(body, 1.0f, -2.0f, 3.0f);
}

ZTEST(axis_map, test_determinants_reflect_handedness)
{
	const struct axis_map cyclic = { .src = { 1, 2, 0 }, .sign = { 1, 1, 1 } };
	const struct axis_map swapped = { .src = { 1, 0, 2 }, .sign = { 1, 1, 1 } };

	zassert_equal(axis_map_determinant(&ism_to_body), 1);
	zassert_equal(axis_map_determinant(&mmc_to_body), -1);
	zassert_equal(axis_map_determinant(&cyclic), 1);
	zassert_equal(axis_map_determinant(&swapped), -1);
}

ZTEST(mag_cal, test_mag_cal_removes_hard_iron_then_scales)
{
	const float hard_iron[3] = { 0.1f, -0.05f, 0.02f };
	const float soft_iron[3][3] = {
		{ 1.1f, 0.0f, 0.0f },
		{ 0.0f, 0.9f, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
	};
	const float raw[3] = { 0.6f, 0.15f, 0.42f };
	float calibrated[3];

	mag_cal_apply(hard_iron, soft_iron, raw, calibrated);
	zassert_within(calibrated[0], 0.55f, 1e-6f);
	zassert_within(calibrated[1], 0.18f, 1e-6f);
	zassert_within(calibrated[2], 0.40f, 1e-6f);
}

ZTEST(mag_cal, test_mag_cal_applies_off_diagonal_terms)
{
	const float no_offset[3] = { 0.0f, 0.0f, 0.0f };
	const float soft_iron[3][3] = {
		{ 1.0f, 0.1f, 0.0f },
		{ 0.1f, 1.0f, -0.2f },
		{ 0.0f, -0.2f, 1.0f },
	};
	const float raw[3] = { 1.0f, 2.0f, 3.0f };
	float calibrated[3];

	mag_cal_apply(no_offset, soft_iron, raw, calibrated);
	zassert_within(calibrated[0], 1.2f, 1e-6f);
	zassert_within(calibrated[1], 1.5f, 1e-6f);
	zassert_within(calibrated[2], 2.6f, 1e-6f);
}

ZTEST(sensor_cal, test_accel_cal_removes_offset_then_scales)
{
	const float offset[3] = { 0.1f, -0.2f, 0.3f };
	const float scale[3] = { 1.01f, 0.99f, 1.0f };
	const float raw[3] = { 0.1f, 0.8f, 10.1066f };
	float calibrated[3];

	accel_cal_apply(offset, scale, raw, calibrated);
	zassert_within(calibrated[0], 0.0f, 1e-6f);
	zassert_within(calibrated[1], 0.99f, 1e-6f);
	zassert_within(calibrated[2], 9.8066f, 1e-5f);
}

ZTEST(sensor_cal, test_interval_survives_counter_wrap)
{
	float dt_s = 0.0f;

	zassert_ok(sensor_cal_interval_s(0x0018F0A0U, 0xFFFF0000U, CYCLES_PER_SEC, &dt_s));
	zassert_within(dt_s, 0.010f, 1e-9f);
}

ZTEST(sensor_cal, test_interval_outside_valid_range_is_rejected)
{
	const uint32_t nominal_cyc = 1634615U; /* 1/104 s at 170 MHz */
	const uint32_t too_short_cyc = (uint32_t)(EKF_DT_MIN * (float)CYCLES_PER_SEC);
	const uint32_t longest_cyc = (uint32_t)(EKF_DT_MAX * (float)CYCLES_PER_SEC);
	float dt_s = -1.0f;

	zassert_equal(sensor_cal_interval_s(1000U, 1000U, CYCLES_PER_SEC, &dt_s), -EINVAL);
	zassert_equal(sensor_cal_interval_s(too_short_cyc, 0U, CYCLES_PER_SEC, &dt_s), -EINVAL);
	zassert_equal(sensor_cal_interval_s(longest_cyc + 1000U, 0U, CYCLES_PER_SEC, &dt_s),
		      -EINVAL);
	zassert_true(dt_s < 0.0f, "a rejected interval must not be written");

	zassert_ok(sensor_cal_interval_s(longest_cyc, 0U, CYCLES_PER_SEC, &dt_s));
	zassert_ok(sensor_cal_interval_s(nominal_cyc, 0U, CYCLES_PER_SEC, &dt_s));
	zassert_within(dt_s, EKF_DT_NOMINAL, 1e-8f);
}

ZTEST_SUITE(axis_map, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(mag_cal, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(sensor_cal, NULL, NULL, NULL, NULL, NULL);
