/*
 * Sensor path: DRDY-clocked acquisition of both chips, calibration, axis
 * mapping and time stamping. Produces struct imu_sample for the fusion thread.
 */
#ifndef SENSOR_HUB_H_
#define SENSOR_HUB_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#include "imu_sample.h"

/* Event counters; they wrap on overflow. */
struct sensor_hub_stats {
	uint32_t frame_drops;   /* DRDY frames lost because the sensor thread fell behind */
	uint32_t queue_drops;   /* samples lost because the fusion thread fell behind */
	uint32_t drdy_timeouts; /* no DRDY within 50 ms */
	uint32_t bus_errors;
	uint32_t mag_not_ready; /* magnetometer conversion still running at the tick */
	uint32_t mag_saturated; /* magnetometer axis on a rail (field >= 8 G) */
	uint32_t dt_fault;      /* sample interval outside (EKF_DT_MIN, EKF_DT_MAX] */
};

struct sensor_hub_snapshot {
	uint32_t seq;             /* samples handed to the fusion queue so far */
	struct imu_sample sample; /* latest sample built, body frame, calibrated */
	float accel_a[3];         /* m/s^2, ISM330DHCX frame, uncalibrated */
	float gyro_a[3];          /* rad/s, ISM330DHCX frame */
	float mag_m[3];           /* gauss, MMC5983MA frame, uncalibrated; last valid reading */
	struct sensor_hub_stats stats;
};

/**
 * Check that both sensor devices initialised.
 *
 * @retval -ENODEV a sensor driver failed its init
 */
int sensor_hub_init(void);

/**
 * Route the ISM330DHCX data-ready interrupt to the hub. Call it last, once the
 * consumer threads run.
 */
int sensor_hub_enable_trigger(void);

/** Sensor thread entry; needs K_FP_REGS. */
void sensor_hub_run(void *p1, void *p2, void *p3);

/**
 * Take the next sample from the fusion queue.
 *
 * @retval -EAGAIN no sample arrived within the timeout
 */
int sensor_hub_get_sample(struct imu_sample *sample, k_timeout_t timeout);

/** Copy the latest readings and counters; callable from any thread. */
void sensor_hub_get_snapshot(struct sensor_hub_snapshot *out);

/**
 * Ask the sensor thread to re-measure the magnetometer bridge offset. It does
 * so at the next tick if the board has been at rest, and refuses otherwise.
 */
void sensor_hub_request_mag_refresh(void);

/** Current magnetometer bridge offset in micro-gauss, sensor frame. */
int sensor_hub_get_mag_offset_ug(int32_t offset_ug[3]);

#endif /* SENSOR_HUB_H_ */
