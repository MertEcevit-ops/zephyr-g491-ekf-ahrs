/*
 * Pure sensor-path arithmetic: axis maps, calibration models and the sample
 * interval. No device or kernel access, so it is unit tested on the host.
 */
#ifndef SENSOR_CAL_H_
#define SENSOR_CAL_H_

#include <stdint.h>

#include "axis_map.h"

/** out[i] = sign[i] * in[src[i]]. out must not alias in. */
void axis_map_apply(const struct axis_map *map, const float in[3], float out[3]);

/** Determinant of the map: +1 for a rotation, -1 when it changes handedness. */
int axis_map_determinant(const struct axis_map *map);

/** out = scale * (in - offset), per axis; m/s^2 in the sensor frame. */
void accel_cal_apply(const float offset[3], const float scale[3], const float in[3], float out[3]);

/** out = soft_iron * (in - hard_iron); gauss in the sensor frame. */
void mag_cal_apply(const float hard_iron[3], const float soft_iron[3][3], const float in[3],
		   float out[3]);

/**
 * Time between two cycle-counter timestamps in seconds.
 *
 * The unsigned difference stays correct across one wrap of the 32-bit counter.
 *
 * @retval -EINVAL the interval is outside (EKF_DT_MIN, EKF_DT_MAX]; dt_s is not written
 */
int sensor_cal_interval_s(uint32_t timestamp_cyc, uint32_t reference_cyc, uint32_t cycles_per_sec,
			  float *dt_s);

#endif /* SENSOR_CAL_H_ */
