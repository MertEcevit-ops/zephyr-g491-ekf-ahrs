/* TEST_PLAN.md phase 1, T-BUS-06 and T-BUS-08: the init order of HARDWARE_MAP.md section 4. */
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#include "bus.h"

#define CTRL4_I2C_DISABLE BIT(2)
/* Reserved bits 7, 4 and 0, plus DRDY_MASK (bit 3), which nothing in this project sets. */
#define CTRL4_MUST_BE_ZERO (BIT(7) | BIT(4) | BIT(3) | BIT(0))

static const struct spi_dt_spec ism = SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), BUS_OP_MODE3);

ZTEST(bus_id_drivers, test_ism330dhcx_driver_is_ready)
{
	zassert_true(device_is_ready(DEVICE_DT_GET(DT_NODELABEL(ism330dhcx))));
}

ZTEST(bus_id_drivers, test_ism330dhcx_i2c_block_is_disabled_after_driver_init)
{
	uint8_t ctrl4 = 0;

	zassert_ok(bus_read(&ism, ISM330_REG_CTRL4_C, &ctrl4, 1));
	zassert_true((ctrl4 & CTRL4_I2C_DISABLE) != 0, "CTRL4_C = 0x%02x", ctrl4);
	zassert_equal(ctrl4 & CTRL4_MUST_BE_ZERO, 0, "CTRL4_C = 0x%02x", ctrl4);
}

/*
 * Evidence only. The output data rate comes from the chip's own oscillator, so
 * the "104 Hz" sample interval differs from board to board by this trim value.
 */
ZTEST(bus_id_drivers, test_ism330dhcx_reports_its_data_rate_trim)
{
	const int nominal_period_us = 64 * 1000000 / 6667; /* 104 Hz setting: 6667 Hz / 64 */
	uint8_t raw = 0;
	int trim_centi_percent;

	zassert_ok(bus_read(&ism, ISM330_REG_INTERNAL_FREQ_FINE, &raw, 1));
	trim_centi_percent = (int8_t)raw * 15;

	printk("INTERNAL_FREQ_FINE %d: data rate %+d.%02d %% off nominal, period about %d us\n",
	       (int8_t)raw, trim_centi_percent / 100, abs(trim_centi_percent) % 100,
	       nominal_period_us * 10000 / (10000 + trim_centi_percent));
}

ZTEST_SUITE(bus_id_drivers, NULL, NULL, NULL, NULL, NULL);
