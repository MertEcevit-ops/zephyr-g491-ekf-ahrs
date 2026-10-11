#include "sensor_cal.h"

#include <errno.h>

#include "ekf/ekf_config.h"

void axis_map_apply(const struct axis_map *map, const float in[3], float out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = (map->sign[i] < 0) ? -in[map->src[i]] : in[map->src[i]];
	}
}

int axis_map_determinant(const struct axis_map *map)
{
	const uint8_t *src = map->src;
	/* Parity of the permutation: the number of out-of-order pairs. */
	int inversions = (src[0] > src[1]) + (src[0] > src[2]) + (src[1] > src[2]);
	int determinant = map->sign[0] * map->sign[1] * map->sign[2];

	return (inversions % 2 == 0) ? determinant : -determinant;
}

void accel_cal_apply(const float offset[3], const float scale[3], const float in[3], float out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = scale[i] * (in[i] - offset[i]);
	}
}

void mag_cal_apply(const float hard_iron[3], const float soft_iron[3][3], const float in[3],
		   float out[3])
{
	const float centered[3] = { in[0] - hard_iron[0], in[1] - hard_iron[1],
				    in[2] - hard_iron[2] };

	for (int row = 0; row < 3; row++) {
		out[row] = soft_iron[row][0] * centered[0] + soft_iron[row][1] * centered[1] +
			   soft_iron[row][2] * centered[2];
	}
}

int sensor_cal_interval_s(uint32_t timestamp_cyc, uint32_t reference_cyc, uint32_t cycles_per_sec,
			  float *dt_s)
{
	uint32_t elapsed_cyc = timestamp_cyc - reference_cyc;
	float interval_s = (float)elapsed_cyc / (float)cycles_per_sec;

	if (interval_s <= EKF_DT_MIN || interval_s > EKF_DT_MAX) {
		return -EINVAL;
	}

	*dt_s = interval_s;
	return 0;
}
