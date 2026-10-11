/* TEST_PLAN.md phase 1, T-BUS-06 and T-BUS-08: the init order of HARDWARE_MAP.md section 4. */
#include <zephyr/device.h>
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

ZTEST_SUITE(bus_id_drivers, NULL, NULL, NULL, NULL, NULL);
