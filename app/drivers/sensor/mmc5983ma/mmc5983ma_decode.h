/*
 * Pure conversions of the MMC5983MA output format (MMC5983MA_SPEC.md section 6).
 * No bus or kernel dependency, so the unit tests include this file directly.
 */
#ifndef MMC5983MA_DECODE_H_
#define MMC5983MA_DECODE_H_

#include <stdbool.h>
#include <stdint.h>

#define MMC5983MA_ZERO_FIELD_COUNTS 131072
#define MMC5983MA_FULL_SCALE_COUNTS 262143U

/* Unsigned 18-bit outputs of the three axes from the 0x00..0x06 burst. */
static inline void mmc5983ma_unpack_18bit(const uint8_t raw[7], uint32_t unsigned_counts[3])
{
	unsigned_counts[0] = ((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) |
			     ((raw[6] >> 6) & 0x3U);
	unsigned_counts[1] = ((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) |
			     ((raw[6] >> 4) & 0x3U);
	unsigned_counts[2] = ((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) |
			     ((raw[6] >> 2) & 0x3U);
}

/** Signed counts (16384 per gauss) of the three axes, zero field removed. */
static inline void mmc5983ma_decode_18bit(const uint8_t raw[7], int32_t counts[3])
{
	uint32_t unsigned_counts[3];

	mmc5983ma_unpack_18bit(raw, unsigned_counts);
	for (int i = 0; i < 3; i++) {
		counts[i] = (int32_t)unsigned_counts[i] - MMC5983MA_ZERO_FIELD_COUNTS;
	}
}

/** True when any axis sits on a rail, i.e. the field magnitude reached 8 G. */
static inline bool mmc5983ma_raw_saturated(const uint8_t raw[7])
{
	uint32_t unsigned_counts[3];

	mmc5983ma_unpack_18bit(raw, unsigned_counts);
	for (int i = 0; i < 3; i++) {
		if (unsigned_counts[i] == 0U || unsigned_counts[i] == MMC5983MA_FULL_SCALE_COUNTS) {
			return true;
		}
	}

	return false;
}

/** Counts to micro-gauss; 1 count is exactly 15625/256 uG. */
static inline int64_t mmc5983ma_counts_to_ug(int32_t counts)
{
	return ((int64_t)counts * 15625) / 256;
}

#endif /* MMC5983MA_DECODE_H_ */
