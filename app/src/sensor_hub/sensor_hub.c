#include "sensor_hub.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <app/drivers/sensor/mmc5983ma.h>

#include "accel_cal.h"
#include "axis_map.h"
#include "mag_cal.h"
#include "sensor_cal.h"

LOG_MODULE_REGISTER(sensor_hub, CONFIG_APP_LOG_LEVEL);

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= 1000000,
	     "dt timestamping needs a >= 1 MHz cycle counter; keep CONFIG_PM=n");
BUILD_ASSERT(CONFIG_MMC5983MA_BANDWIDTH >= 1,
	     "BW=00 needs up to 10 ms per measurement and cannot keep up with 104 Hz");

#define DRDY_TIMEOUT_MS  50
#define ISM_FRAME_DEPTH  2
/* 4 x 9.6 ms stays below EKF_DT_MAX, so a full queue never turns into a dt fault. */
#define IMU_SAMPLE_DEPTH 4

/* Bridge offset refresh precondition (MMC5983MA_SPEC.md 9.9): 0.5 s below 0.05 rad/s. */
#define STILL_RATE_SQ_MAX (0.05f * 0.05f)
#define STILL_SAMPLES_MIN 52U

/* Readings of one DRDY tick, as the ISM330DHCX driver reports them. */
struct ism_frame {
	uint32_t timestamp_cyc;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
};

K_MSGQ_DEFINE(ism_frame_q, sizeof(struct ism_frame), ISM_FRAME_DEPTH, 4);
K_MSGQ_DEFINE(imu_sample_q, sizeof(struct imu_sample), IMU_SAMPLE_DEPTH, 4);

static const struct device *const imu_dev = DEVICE_DT_GET(DT_NODELABEL(ism330dhcx));
static const struct device *const mag_dev = DEVICE_DT_GET(DT_NODELABEL(mmc5983ma));

/* Written by the ISM330DHCX driver thread, read by the sensor thread. */
static atomic_t handler_frame_drops;
static atomic_t handler_bus_errors;
static atomic_t mag_refresh_requested;

/* Owned by the sensor thread. */
static struct {
	struct sensor_hub_snapshot latest;
	uint32_t reference_cyc; /* timestamp of the last sample the fusion queue took */
	bool has_reference;
	bool drdy_lost;
	uint32_t still_samples;
	int64_t last_refresh_ms;
} hub;

static struct k_spinlock snapshot_lock;
static struct sensor_hub_snapshot published;

/*
 * Runs in the ISM330DHCX driver thread (cooperative). The driver calls it
 * again until both data-ready flags are clear, and only reading the outputs
 * clears them, so accelerometer and gyroscope must both be fetched here.
 */
static void on_imu_data_ready(const struct device *dev, const struct sensor_trigger *trigger)
{
	/* First statement: the delay up to here is the same every tick and cancels in dt. */
	struct ism_frame frame = { .timestamp_cyc = k_cycle_get_32() };

	ARG_UNUSED(trigger);

	if (sensor_sample_fetch(dev) != 0 ||
	    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, frame.accel) != 0 ||
	    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, frame.gyro) != 0) {
		atomic_inc(&handler_bus_errors);
		return;
	}

	if (k_msgq_put(&ism_frame_q, &frame, K_NO_WAIT) != 0) {
		atomic_inc(&handler_frame_drops);
	}
}

static void sensor_values_to_float(const struct sensor_value in[3], float out[3])
{
	for (int i = 0; i < 3; i++) {
		out[i] = (float)in[i].val1 + (float)in[i].val2 * 1.0e-6f;
	}
}

/* Reads the pipelined magnetometer sample; false when this tick carries no new reading. */
static bool read_mag(float mag_m[3])
{
	struct sensor_hub_stats *stats = &hub.latest.stats;
	struct sensor_value field[3];
	int rc = sensor_sample_fetch(mag_dev);

	if (rc == 0) {
		rc = sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, field);
	}

	switch (rc) {
	case 0:
		sensor_values_to_float(field, mag_m);
		return true;
	case -EBUSY:
		stats->mag_not_ready++;
		return false;
	case -ERANGE:
		stats->mag_saturated++;
		return false;
	default:
		stats->bus_errors++;
		return false;
	}
}

static void track_stillness(const float gyro[3])
{
	float rate_sq = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];

	hub.still_samples = (rate_sq < STILL_RATE_SQ_MAX) ? hub.still_samples + 1U : 0U;
}

/*
 * PROJECT_SPEC 4.4. dt_s always refers to the previous sample that reached the
 * fusion queue, so a dropped sample leaves no gap in the gyro integration.
 */
static int stamp_interval(struct imu_sample *sample)
{
	int rc;

	if (!hub.has_reference) {
		hub.reference_cyc = sample->timestamp_cyc;
		hub.has_reference = true;
		return -EAGAIN;
	}

	rc = sensor_cal_interval_s(sample->timestamp_cyc, hub.reference_cyc,
				   sys_clock_hw_cycles_per_sec(), &sample->dt_s);
	if (rc != 0) {
		hub.latest.stats.dt_fault++;
		hub.reference_cyc = sample->timestamp_cyc;
	}

	return rc;
}

static void offer_to_fusion(const struct imu_sample *sample)
{
	if (k_msgq_put(&imu_sample_q, sample, K_NO_WAIT) != 0) {
		hub.latest.stats.queue_drops++;
		return;
	}

	hub.reference_cyc = sample->timestamp_cyc;
	hub.latest.seq++;
}

static void publish_snapshot(void)
{
	struct sensor_hub_stats *stats = &hub.latest.stats;
	k_spinlock_key_t key;

	stats->frame_drops = (uint32_t)atomic_get(&handler_frame_drops);

	key = k_spin_lock(&snapshot_lock);
	published = hub.latest;
	published.stats.bus_errors += (uint32_t)atomic_get(&handler_bus_errors);
	k_spin_unlock(&snapshot_lock, key);
}

static void process_frame(const struct ism_frame *frame)
{
	struct sensor_hub_snapshot *latest = &hub.latest;
	struct imu_sample *sample = &latest->sample;
	float accel_cal[3], mag_cal[3];
	bool mag_valid = read_mag(latest->mag_m);

	sensor_values_to_float(frame->accel, latest->accel_a);
	sensor_values_to_float(frame->gyro, latest->gyro_a);

	/* Calibration lives in the sensor frames, so a new axis map never invalidates it. */
	accel_cal_apply(ACCEL_CAL_OFFSET, ACCEL_CAL_SCALE, latest->accel_a, accel_cal);
	mag_cal_apply(MAG_CAL_V, MAG_CAL_W, latest->mag_m, mag_cal);

	axis_map_apply(&ism_to_body, accel_cal, sample->accel);
	axis_map_apply(&ism_to_body, latest->gyro_a, sample->gyro);
	axis_map_apply(&mmc_to_body, mag_cal, sample->mag);

	sample->timestamp_cyc = frame->timestamp_cyc;
	sample->dt_s = 0.0f;
	sample->flags = mag_valid ? IMU_SAMPLE_MAG_VALID : 0U;

	track_stillness(sample->gyro);

	if (stamp_interval(sample) == 0) {
		offer_to_fusion(sample);
	}

	publish_snapshot();
}

static void refresh_mag_offset(void)
{
	const struct sensor_value unused = { 0 };
	int rc = sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ,
				 (enum sensor_attribute)MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH, &unused);

	hub.last_refresh_ms = k_uptime_get();

	if (rc == 0) {
		LOG_INF("magnetometer bridge offset refreshed");
	} else if (rc == -ERANGE) {
		hub.latest.stats.mag_saturated++;
		LOG_WRN("bridge offset refresh skipped: field saturated");
	} else {
		hub.latest.stats.bus_errors++;
		LOG_ERR("bridge offset refresh failed (%d)", rc);
	}
}

/*
 * The refresh blocks for about 11 ms and costs one DRDY tick, which is only
 * harmless while the board does not rotate.
 */
static void service_mag_refresh(void)
{
	bool requested = atomic_cas(&mag_refresh_requested, 1, 0);
	bool still = hub.still_samples >= STILL_SAMPLES_MIN;
	bool due = (k_uptime_get() - hub.last_refresh_ms) >=
		   (int64_t)CONFIG_APP_MAG_OFFSET_REFRESH_S * MSEC_PER_SEC;

	if (requested && !still) {
		LOG_WRN("bridge offset refresh refused: board is moving");
		return;
	}

	if (still && (requested || due)) {
		refresh_mag_offset();
	}
}

static void note_drdy_timeout(void)
{
	hub.latest.stats.drdy_timeouts++;

	if (!hub.drdy_lost) {
		hub.drdy_lost = true;
		LOG_ERR("no ISM330DHCX data-ready within %d ms", DRDY_TIMEOUT_MS);
	}

	publish_snapshot();
}

void sensor_hub_run(void *p1, void *p2, void *p3)
{
	struct ism_frame frame;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* The driver measured the offset during its own init. */
	hub.last_refresh_ms = k_uptime_get();

	while (true) {
		if (k_msgq_get(&ism_frame_q, &frame, K_MSEC(DRDY_TIMEOUT_MS)) != 0) {
			note_drdy_timeout();
			continue;
		}

		hub.drdy_lost = false;
		process_frame(&frame);
		service_mag_refresh();
	}
}

int sensor_hub_init(void)
{
	if (!device_is_ready(imu_dev)) {
		LOG_ERR("%s is not ready", imu_dev->name);
		return -ENODEV;
	}
	if (!device_is_ready(mag_dev)) {
		LOG_ERR("%s is not ready", mag_dev->name);
		return -ENODEV;
	}

	return 0;
}

int sensor_hub_enable_trigger(void)
{
	/*
	 * Accelerometer only: both sensors share the output data rate, and a
	 * second trigger would run the handler twice per tick.
	 */
	static const struct sensor_trigger data_ready = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	return sensor_trigger_set(imu_dev, &data_ready, on_imu_data_ready);
}

int sensor_hub_get_sample(struct imu_sample *sample, k_timeout_t timeout)
{
	return (k_msgq_get(&imu_sample_q, sample, timeout) == 0) ? 0 : -EAGAIN;
}

void sensor_hub_get_snapshot(struct sensor_hub_snapshot *out)
{
	k_spinlock_key_t key = k_spin_lock(&snapshot_lock);

	*out = published;
	k_spin_unlock(&snapshot_lock, key);
}

void sensor_hub_request_mag_refresh(void)
{
	atomic_set(&mag_refresh_requested, 1);
}

int sensor_hub_get_mag_offset_ug(int32_t offset_ug[3])
{
	struct sensor_value offset[3];
	int rc = sensor_attr_get(mag_dev, SENSOR_CHAN_MAGN_XYZ,
				 (enum sensor_attribute)MMC5983MA_ATTR_BRIDGE_OFFSET, offset);

	if (rc != 0) {
		return rc;
	}

	for (int i = 0; i < 3; i++) {
		offset_ug[i] = (int32_t)sensor_value_to_micro(&offset[i]);
	}

	return 0;
}
