/*
 * TEST_PLAN.md phase 1, T-BUS-01..05, 07 and 09: the bus, both chip selects and
 * both chips, without any sensor driver in the way. Tests run in name order;
 * the soak comes last.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#include "bus.h"
#include "mmc5983ma_decode.h"
#include "mmc5983ma_reg.h"

#define ID_READS         1000
#define MODE_PROBE_READS 100
#define SOAK_CYCLES      100000
#define MEAS_TIMEOUT_MS  20
#define ONE_GAUSS_COUNTS 16384

static const struct spi_dt_spec ism = SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), BUS_OP_MODE3);
static const struct spi_dt_spec mmc = SPI_DT_SPEC_GET(DT_NODELABEL(mmc5983ma), BUS_OP_MODE3);
static const struct spi_dt_spec mmc_mode0 =
	SPI_DT_SPEC_GET(DT_NODELABEL(mmc5983ma), BUS_OP_MODE0);

static int count_matching_reads(const struct spi_dt_spec *bus, uint8_t reg, uint8_t expected,
				int reads)
{
	int matches = 0;

	for (int i = 0; i < reads; i++) {
		uint8_t value = 0;

		if (bus_read(bus, reg, &value, 1) == 0 && value == expected) {
			matches++;
		}
	}

	return matches;
}

ZTEST(bus_id_raw, test_01_both_spi_devices_are_ready)
{
	zassert_true(spi_is_ready_dt(&ism));
	zassert_true(spi_is_ready_dt(&mmc));
}

ZTEST(bus_id_raw, test_02_ism330dhcx_who_am_i_is_stable)
{
	int matches = count_matching_reads(&ism, ISM330_REG_WHO_AM_I, ISM330_WHO_AM_I, ID_READS);

	zassert_equal(matches, ID_READS, "%d of %d reads returned 0x6B", matches, ID_READS);
}

ZTEST(bus_id_raw, test_03_mmc5983ma_product_id_is_stable)
{
	int matches = count_matching_reads(&mmc, MMC5983MA_REG_PRODUCT_ID, MMC5983MA_PRODUCT_ID,
					   ID_READS);

	zassert_equal(matches, ID_READS, "%d of %d reads returned 0x30", matches, ID_READS);
}

ZTEST(bus_id_raw, test_04_mmc5983ma_software_reset_reloads_otp)
{
	uint8_t status = 0, product_id = 0;

	zassert_ok(bus_write(&mmc, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST));
	k_msleep(MMC5983MA_RESET_TIME_MS);

	zassert_ok(bus_read(&mmc, MMC5983MA_REG_STATUS, &status, 1));
	zassert_true((status & MMC5983MA_STATUS_OTP_READ_DONE) != 0, "status 0x%02x", status);

	zassert_ok(bus_read(&mmc, MMC5983MA_REG_PRODUCT_ID, &product_id, 1));
	zassert_equal(product_id, MMC5983MA_PRODUCT_ID, "product id 0x%02x", product_id);
}

ZTEST(bus_id_raw, test_05_mmc5983ma_single_measurement_is_plausible)
{
	uint8_t raw[MMC5983MA_MEASUREMENT_LEN];
	uint8_t status = 0;
	int32_t counts[3];
	int64_t deadline;

	zassert_ok(bus_write(&mmc, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_M));

	deadline = k_uptime_get() + MEAS_TIMEOUT_MS;
	do {
		k_usleep(MMC5983MA_POLL_INTERVAL_US);
		zassert_ok(bus_read(&mmc, MMC5983MA_REG_STATUS, &status, 1));
	} while ((status & MMC5983MA_STATUS_MEAS_M_DONE) == 0 && k_uptime_get() <= deadline);
	zassert_true((status & MMC5983MA_STATUS_MEAS_M_DONE) != 0, "no Meas_M_Done in %d ms",
		     MEAS_TIMEOUT_MS);

	zassert_ok(bus_read(&mmc, MMC5983MA_REG_XOUT0, raw, sizeof(raw)));
	zassert_false(mmc5983ma_raw_saturated(raw));

	mmc5983ma_decode_18bit(raw, counts);
	printk("field counts: %d %d %d\n", counts[0], counts[1], counts[2]);
	for (int i = 0; i < 3; i++) {
		zassert_true(counts[i] > -ONE_GAUSS_COUNTS && counts[i] < ONE_GAUSS_COUNTS,
			     "axis %d reads %d counts", i, counts[i]);
	}
}

/*
 * T-BUS-09. The datasheet implies mode 3, SparkFun's library reports that mode
 * 0 "is what worked". Mode 3 must work here; the mode 0 result is evidence only.
 */
ZTEST(bus_id_raw, test_09_mmc5983ma_answers_in_spi_mode_3)
{
	int mode3 = count_matching_reads(&mmc, MMC5983MA_REG_PRODUCT_ID, MMC5983MA_PRODUCT_ID,
					 MODE_PROBE_READS);
	int mode0 = count_matching_reads(&mmc_mode0, MMC5983MA_REG_PRODUCT_ID,
					 MMC5983MA_PRODUCT_ID, MODE_PROBE_READS);
	/* Leaves the bus in mode 3 again for whatever runs next. */
	int mode3_again = count_matching_reads(&mmc, MMC5983MA_REG_PRODUCT_ID,
					       MMC5983MA_PRODUCT_ID, MODE_PROBE_READS);

	printk("PRODUCT_ID matches: mode 3 %d/%d, mode 0 %d/%d\n", mode3, MODE_PROBE_READS, mode0,
	       MODE_PROBE_READS);
	zassert_equal(mode3, MODE_PROBE_READS);
	zassert_equal(mode3_again, MODE_PROBE_READS);
}

/*
 * T-BUS-07. Interleaves both chips the way the application does. It is the
 * experimental proof of the first-byte rule (HARDWARE_MAP.md section 4): no
 * transaction may make the other chip's I2C block answer.
 */
ZTEST(bus_id_raw, test_99_interleaved_traffic_soak)
{
	uint8_t burst[MMC5983MA_MEASUREMENT_LEN];
	uint8_t id = 0;
	uint32_t id_mismatches = 0, bus_errors = 0;
	int64_t start_ms = k_uptime_get();
	int64_t elapsed_ms;

	for (int cycle = 0; cycle < SOAK_CYCLES; cycle++) {
		bus_errors += (bus_read(&ism, ISM330_REG_WHO_AM_I, &id, 1) != 0);
		id_mismatches += (id != ISM330_WHO_AM_I);

		bus_errors += (bus_read(&mmc, MMC5983MA_REG_PRODUCT_ID, &id, 1) != 0);
		id_mismatches += (id != MMC5983MA_PRODUCT_ID);

		bus_errors += (bus_read(&ism, ISM330_REG_WHO_AM_I, burst, 4) != 0);
		id_mismatches += (burst[0] != ISM330_WHO_AM_I);

		bus_errors += (bus_read(&mmc, MMC5983MA_REG_XOUT0, burst, sizeof(burst)) != 0);
		bus_errors += (bus_write(&mmc, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_M) != 0);
	}

	elapsed_ms = k_uptime_get() - start_ms;
	printk("soak: %d cycles in %d ms, %d transactions/s\n", SOAK_CYCLES, (int)elapsed_ms,
	       (int)((int64_t)SOAK_CYCLES * 5 * MSEC_PER_SEC / MAX(elapsed_ms, 1)));

	zassert_equal(bus_errors, 0, "%u SPI errors", bus_errors);
	zassert_equal(id_mismatches, 0, "%u identity mismatches", id_mismatches);
}

ZTEST_SUITE(bus_id_raw, NULL, NULL, NULL, NULL, NULL);
