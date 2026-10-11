/*
 * Accelerometer six-face calibration, sensor frame (MATH_SPEC.md section 3.3):
 * f_cal = scale * (f - offset).
 *
 * Identity values: the six-face procedure (TEST_PLAN.md T-RAW-12) has not been
 * run for this board yet.
 */
#ifndef ACCEL_CAL_H_
#define ACCEL_CAL_H_

static const float ACCEL_CAL_OFFSET[3] = { 0.0f, 0.0f, 0.0f }; /* m/s^2 */
static const float ACCEL_CAL_SCALE[3] = { 1.0f, 1.0f, 1.0f };

#endif /* ACCEL_CAL_H_ */
