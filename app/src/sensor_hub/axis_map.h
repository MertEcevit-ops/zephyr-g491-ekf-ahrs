/*
 * Sensor-frame to body-frame (FRD) axis maps, MATH_SPEC.md section 2.6.
 *
 * The maps are signed permutations, so applying them costs no multiplication
 * and adds no rounding.
 */
#ifndef AXIS_MAP_H_
#define AXIS_MAP_H_

#include <stdint.h>

struct axis_map {
	uint8_t src[3]; /* body axis i takes sensor axis src[i] */
	int8_t sign[3];
};

static const struct axis_map ism_to_body = { .src = { 0, 1, 2 }, .sign = { 1, -1, -1 } };

/*
 * Determinant -1 on purpose: the MMC5983MA datasheet frame is left-handed
 * (MATH_SPEC.md section 2.3). Do not "fix" it into a rotation.
 */
static const struct axis_map mmc_to_body = { .src = { 0, 1, 2 }, .sign = { 1, -1, 1 } };

#endif /* AXIS_MAP_H_ */
