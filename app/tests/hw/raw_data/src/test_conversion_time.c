/*
 * TEST_PLAN.md phase 2, T-RAW-06: magnetometer conversion time, taken from the
 * INT pin (CONFIG_MMC5983MA_INT_DIAG). Drives the driver directly; the sensor
 * hub is not running in this scenario.
 */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#define MEASUREMENTS       200
#define CONVERSION_MAX_US  6000
#define INT_WAIT_LIMIT_US  20000

static const struct device *const mag_dev = DEVICE_DT_GET(DT_NODELABEL(mmc5983ma));
static const struct gpio_dt_spec int_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(mmc5983ma), int_gpios);

static uint32_t elapsed_us(uint32_t start_cyc)
{
	return (uint32_t)(((uint64_t)(k_cycle_get_32() - start_cyc) * USEC_PER_SEC) /
			  sys_clock_hw_cycles_per_sec());
}

ZTEST(raw_conversion_time, test_conversion_finishes_within_budget_and_int_clears)
{
	uint32_t slowest_us = 0, fastest_us = UINT32_MAX;

	zassert_true(device_is_ready(mag_dev));
	zassert_true(gpio_is_ready_dt(&int_gpio));

	/* Let the measurement left in flight by the driver's init finish. */
	k_msleep(20);

	for (int n = 0; n < MEASUREMENTS; n++) {
		uint32_t start_cyc, conversion_us;

		/*
		 * Reads the finished measurement, releases INT by writing
		 * Meas_M_Done and, as its last bus access, starts the next one.
		 */
		zassert_ok(sensor_sample_fetch(mag_dev), "measurement %d", n);
		start_cyc = k_cycle_get_32();
		zassert_equal(gpio_pin_get_dt(&int_gpio), 0, "INT still high after STATUS write");

		while (gpio_pin_get_dt(&int_gpio) == 0 && elapsed_us(start_cyc) < INT_WAIT_LIMIT_US) {
		}
		conversion_us = elapsed_us(start_cyc);

		slowest_us = MAX(slowest_us, conversion_us);
		fastest_us = MIN(fastest_us, conversion_us);
	}

	printk("conversion time (BW=%d): %u..%u us\n", CONFIG_MMC5983MA_BANDWIDTH, fastest_us,
	       slowest_us);
	zassert_true(slowest_us <= CONVERSION_MAX_US, "slowest conversion %u us", slowest_us);
}

ZTEST_SUITE(raw_conversion_time, NULL, NULL, NULL, NULL, NULL);
