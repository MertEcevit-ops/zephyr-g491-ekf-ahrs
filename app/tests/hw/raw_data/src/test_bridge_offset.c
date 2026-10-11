/* TEST_PLAN.md phase 2, T-RAW-00 and T-RAW-05: driver init and SET/RESET repeatability. */
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#include <app/drivers/sensor/mmc5983ma.h>

#include "hub_fixture.h"

#define REFRESH_COUNT        20
#define REFRESH_INTERVAL_MS  1000
#define STD_MAX_UG           1000
#define SPREAD_MAX_UG        4000

static const struct device *const mag_dev = DEVICE_DT_GET(DT_NODELABEL(mmc5983ma));

/* T-RAW-00 */
ZTEST(raw_bridge_offset, test_00_driver_initialised_with_a_measured_offset)
{
	int32_t offset_ug[3];

	zassert_true(device_is_ready(mag_dev));
	zassert_ok(sensor_hub_get_mag_offset_ug(offset_ug));
	printk("bridge offset after init [uG]: %d %d %d\n", offset_ug[0], offset_ug[1],
	       offset_ug[2]);

	/* The driver leaves all three at exactly zero when init hit a saturated field. */
	zassert_true(offset_ug[0] != 0 || offset_ug[1] != 0 || offset_ug[2] != 0,
		     "offset was not measured during init");
}

static uint32_t isqrt_u64(uint64_t value)
{
	uint64_t root = 0;

	for (uint64_t bit = 1ULL << 31; bit != 0; bit >>= 1) {
		if ((root + bit) * (root + bit) <= value) {
			root += bit;
		}
	}

	return (uint32_t)root;
}

/* T-RAW-05 */
ZTEST(raw_bridge_offset, test_05_refreshed_offset_is_repeatable)
{
	const struct sensor_value unused = { 0 };
	static int32_t offsets_ug[REFRESH_COUNT][3];

	for (int n = 0; n < REFRESH_COUNT; n++) {
		k_msleep(REFRESH_INTERVAL_MS);
		zassert_ok(sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ,
					   (enum sensor_attribute)MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH,
					   &unused));
		zassert_ok(sensor_hub_get_mag_offset_ug(offsets_ug[n]));
	}

	for (int axis = 0; axis < 3; axis++) {
		int32_t lowest = offsets_ug[0][axis], highest = offsets_ug[0][axis];
		int64_t sum = 0;
		uint64_t sq_dev_sum = 0;
		int32_t mean;
		uint32_t std_ug;

		for (int n = 0; n < REFRESH_COUNT; n++) {
			lowest = MIN(lowest, offsets_ug[n][axis]);
			highest = MAX(highest, offsets_ug[n][axis]);
			sum += offsets_ug[n][axis];
		}
		mean = (int32_t)(sum / REFRESH_COUNT);
		for (int n = 0; n < REFRESH_COUNT; n++) {
			int64_t deviation = offsets_ug[n][axis] - mean;

			sq_dev_sum += (uint64_t)(deviation * deviation);
		}
		std_ug = isqrt_u64(sq_dev_sum / (REFRESH_COUNT - 1));

		printk("axis %d (settle %d us): mean %d uG, std %u uG, spread %d uG\n", axis,
		       CONFIG_MMC5983MA_SR_SETTLE_US, mean, std_ug, highest - lowest);
		zassert_true(std_ug < STD_MAX_UG, "axis %d std %u uG", axis, std_ug);
		zassert_true(highest - lowest < SPREAD_MAX_UG, "axis %d spread %d uG", axis,
			     highest - lowest);
	}
}

ZTEST_SUITE(raw_bridge_offset, NULL, hub_fixture_setup, NULL, NULL, NULL);
