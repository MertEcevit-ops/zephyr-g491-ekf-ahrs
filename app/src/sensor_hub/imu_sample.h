/* Contract between the sensor hub and the fusion thread (MATH_SPEC.md section 4). */
#ifndef IMU_SAMPLE_H_
#define IMU_SAMPLE_H_

#include <stdint.h>

#define IMU_SAMPLE_MAG_VALID (1U << 0)

struct imu_sample {
	uint32_t timestamp_cyc; /* k_cycle_get_32() taken in the DRDY trigger handler */
	float dt_s;             /* timestamp delta to the previous accepted sample */
	float gyro[3];          /* rad/s, body frame */
	float accel[3];         /* m/s^2, body frame, calibrated */
	float mag[3];           /* gauss, body frame, hard/soft-iron corrected */
	uint32_t flags;
};

#endif /* IMU_SAMPLE_H_ */
