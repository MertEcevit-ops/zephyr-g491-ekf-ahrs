/* TEST_PLAN.md T-HUB-05: MMC5983MA_SPEC.md section 6.5 vectors. */
#include <zephyr/ztest.h>

#include "mmc5983ma_decode.h"

ZTEST(mmc_decode, test_reference_burst_decodes_to_signed_counts_and_microgauss)
{
	const uint8_t raw[7] = { 0x90, 0x00, 0x7F, 0xFF, 0x78, 0x00, 0x30 };
	int32_t counts[3];

	mmc5983ma_decode_18bit(raw, counts);

	zassert_equal(counts[0], 16384);
	zassert_equal(counts[1], -1);
	zassert_equal(counts[2], -8192);
	zassert_false(mmc5983ma_raw_saturated(raw));

	zassert_equal(mmc5983ma_counts_to_ug(counts[0]), 1000000);
	zassert_equal(mmc5983ma_counts_to_ug(counts[1]), -61);
	zassert_equal(mmc5983ma_counts_to_ug(counts[2]), -500000);
}

ZTEST(mmc_decode, test_low_bits_come_from_the_shared_register)
{
	const uint8_t raw[7] = { 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0xE4 };
	int32_t counts[3];

	mmc5983ma_decode_18bit(raw, counts);

	zassert_equal(counts[0], 3);
	zassert_equal(counts[1], 2);
	zassert_equal(counts[2], 1);
}

ZTEST(mmc_decode, test_rail_values_are_reported_as_saturation)
{
	const uint8_t x_at_zero[7] = { 0x00, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00 };
	const uint8_t z_at_full_scale[7] = { 0x80, 0x00, 0x80, 0x00, 0xFF, 0xFF, 0x0C };
	const uint8_t mid_scale[7] = { 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00 };

	zassert_true(mmc5983ma_raw_saturated(x_at_zero));
	zassert_true(mmc5983ma_raw_saturated(z_at_full_scale));
	zassert_false(mmc5983ma_raw_saturated(mid_scale));
}

ZTEST_SUITE(mmc_decode, NULL, NULL, NULL, NULL, NULL);
