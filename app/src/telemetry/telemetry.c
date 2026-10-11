#include "telemetry.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/sys/atomic.h>

#include "ekf/ekf_math.h"
#include "fusion/fusion.h"
#include "sensor_hub/sensor_hub.h"

LOG_MODULE_REGISTER(telemetry, CONFIG_APP_LOG_LEVEL);

#define RATE_HZ_MIN 1
#define RATE_HZ_MAX 50

/* Floats never reach a variadic call (they would be promoted to double). */
#define SCALE_MILLI   1.0e3f
#define SCALE_MICRO   1.0e6f
#define RAD_TO_MDEG   (180000.0f / EKF_PI)

enum telemetry_mode {
	TELEMETRY_OFF = 0,
	TELEMETRY_RAW,    /* R: body frame, calibrated */
	TELEMETRY_SENSOR, /* S: sensor frames, uncalibrated */
	TELEMETRY_FUSION, /* T: filter output */
};

static atomic_t stream_mode = ATOMIC_INIT(TELEMETRY_FUSION);
static atomic_t stream_rate_hz = ATOMIC_INIT(CONFIG_APP_TELEMETRY_RATE_HZ);

static int32_t scaled(float value, float scale)
{
	float product = value * scale;

	/* A faulted filter may hold NaN; lroundf() on it is undefined. */
	return isfinite(product) ? (int32_t)lroundf(product) : 0;
}

static void print_raw_record(const struct shell *sh)
{
	struct sensor_hub_snapshot hub;
	const struct imu_sample *s = &hub.sample;

	sensor_hub_get_snapshot(&hub);
	shell_fprintf(sh, SHELL_NORMAL, "R,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n", hub.seq,
		      scaled(s->dt_s, SCALE_MICRO), scaled(s->accel[0], SCALE_MILLI),
		      scaled(s->accel[1], SCALE_MILLI), scaled(s->accel[2], SCALE_MILLI),
		      scaled(s->gyro[0], SCALE_MICRO), scaled(s->gyro[1], SCALE_MICRO),
		      scaled(s->gyro[2], SCALE_MICRO), scaled(s->mag[0], SCALE_MICRO),
		      scaled(s->mag[1], SCALE_MICRO), scaled(s->mag[2], SCALE_MICRO),
		      s->flags & IMU_SAMPLE_MAG_VALID);
}

static void print_sensor_record(const struct shell *sh)
{
	struct sensor_hub_snapshot hub;

	sensor_hub_get_snapshot(&hub);
	shell_fprintf(sh, SHELL_NORMAL, "S,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n", hub.seq,
		      scaled(hub.accel_a[0], SCALE_MILLI), scaled(hub.accel_a[1], SCALE_MILLI),
		      scaled(hub.accel_a[2], SCALE_MILLI), scaled(hub.gyro_a[0], SCALE_MICRO),
		      scaled(hub.gyro_a[1], SCALE_MICRO), scaled(hub.gyro_a[2], SCALE_MICRO),
		      scaled(hub.mag_m[0], SCALE_MICRO), scaled(hub.mag_m[1], SCALE_MICRO),
		      scaled(hub.mag_m[2], SCALE_MICRO), hub.sample.flags & IMU_SAMPLE_MAG_VALID);
}

/* q and -q are the same attitude; only this output copy is made canonical (q0 >= 0). */
static void canonical_quaternion(const float q[4], float out[4])
{
	float sign = (q[0] < 0.0f) ? -1.0f : 1.0f;

	for (int i = 0; i < 4; i++) {
		out[i] = sign * q[i];
	}
}

static void print_fusion_record(const struct shell *sh)
{
	struct fusion_snapshot f;
	struct sensor_hub_snapshot hub;
	float q[4], roll, pitch, yaw;

	fusion_get_snapshot(&f);
	sensor_hub_get_snapshot(&hub);
	canonical_quaternion(f.q, q);
	quat_to_euler(q, &roll, &pitch, &yaw);

	shell_fprintf(sh, SHELL_NORMAL,
		      "T,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
		      "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
		      f.seq, f.timestamp_ms, (int)f.state, scaled(roll, RAD_TO_MDEG),
		      scaled(pitch, RAD_TO_MDEG), scaled(yaw, RAD_TO_MDEG),
		      scaled(q[0], SCALE_MICRO), scaled(q[1], SCALE_MICRO),
		      scaled(q[2], SCALE_MICRO), scaled(q[3], SCALE_MICRO),
		      scaled(f.gyro_bias[0], SCALE_MICRO), scaled(f.gyro_bias[1], SCALE_MICRO),
		      scaled(f.gyro_bias[2], SCALE_MICRO), f.ekf.acc_soft, f.ekf.acc_gated,
		      f.ekf.acc_rejected, f.ekf.mag_gated, f.ekf.mag_rejected, f.ekf.yaw_resets,
		      f.ekf.tilt_resets, f.ekf.gyro_sat, f.ekf.numeric_faults, hub.stats.dt_fault,
		      hub.stats.frame_drops + hub.stats.queue_drops, f.ekf_cyc, f.latency_cyc,
		      f.flags);
}

void telemetry_run(void *p1, void *p2, void *p3)
{
	const struct shell *sh = shell_backend_uart_get_ptr();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_msleep(MSEC_PER_SEC / (int32_t)atomic_get(&stream_rate_hz));

		switch ((enum telemetry_mode)atomic_get(&stream_mode)) {
		case TELEMETRY_RAW:
			print_raw_record(sh);
			break;
		case TELEMETRY_SENSOR:
			print_sensor_record(sh);
			break;
		case TELEMETRY_FUSION:
			print_fusion_record(sh);
			break;
		case TELEMETRY_OFF:
		default:
			break;
		}
	}
}

static int cmd_telemetry_mode(const struct shell *sh, size_t argc, char **argv)
{
	static const struct {
		const char *name;
		enum telemetry_mode mode;
	} modes[] = {
		{ "raw", TELEMETRY_RAW },
		{ "sensor", TELEMETRY_SENSOR },
		{ "fusion", TELEMETRY_FUSION },
		{ "off", TELEMETRY_OFF },
	};

	ARG_UNUSED(argc);

	for (size_t i = 0; i < ARRAY_SIZE(modes); i++) {
		if (strcmp(argv[1], modes[i].name) == 0) {
			atomic_set(&stream_mode, modes[i].mode);
			return 0;
		}
	}

	shell_error(sh, "unknown mode '%s'", argv[1]);
	return -EINVAL;
}

static int cmd_telemetry_rate(const struct shell *sh, size_t argc, char **argv)
{
	int parse_error = 0;
	long rate_hz = shell_strtol(argv[1], 10, &parse_error);

	ARG_UNUSED(argc);

	if (parse_error != 0 || rate_hz < RATE_HZ_MIN || rate_hz > RATE_HZ_MAX) {
		shell_error(sh, "rate must be %d..%d Hz", RATE_HZ_MIN, RATE_HZ_MAX);
		return -EINVAL;
	}

	atomic_set(&stream_rate_hz, (atomic_val_t)rate_hz);
	return 0;
}

static int cmd_mag_offset(const struct shell *sh, size_t argc, char **argv)
{
	int32_t offset_ug[3];
	int rc = sensor_hub_get_mag_offset_ug(offset_ug);

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (rc != 0) {
		shell_error(sh, "bridge offset not available (%d)", rc);
		return rc;
	}

	shell_print(sh, "bridge offset [uG, sensor frame]: %d %d %d", offset_ug[0], offset_ug[1],
		    offset_ug[2]);
	return 0;
}

static int cmd_mag_refresh(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	sensor_hub_request_mag_refresh();
	shell_print(sh, "refresh requested; the log reports whether the board was at rest");
	return 0;
}

static int cmd_ekf_align(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fusion_request_align();
	return 0;
}

static int cmd_ekf_mag(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (strcmp(argv[1], "on") == 0) {
		fusion_set_use_mag(true);
	} else if (strcmp(argv[1], "off") == 0) {
		fusion_set_use_mag(false);
	} else {
		shell_error(sh, "expected on or off");
		return -EINVAL;
	}

	return 0;
}

static int cmd_ekf_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct fusion_snapshot f;
	struct sensor_hub_snapshot hub;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fusion_get_snapshot(&f);
	sensor_hub_get_snapshot(&hub);

	shell_print(sh, "state %s, heading_converged %u, yaw_valid %u",
		    (f.state == FUSION_RUNNING) ? "RUNNING" : "ALIGNING",
		    (f.flags & FUSION_FLAG_HEADING_CONVERGED) ? 1U : 0U,
		    (f.flags & FUSION_FLAG_YAW_VALID) ? 1U : 0U);
	shell_print(sh, "gyro bias [urad/s]: %d %d %d", scaled(f.gyro_bias[0], SCALE_MICRO),
		    scaled(f.gyro_bias[1], SCALE_MICRO), scaled(f.gyro_bias[2], SCALE_MICRO));
	shell_print(sh, "ekf: acc_soft %u acc_gated %u acc_rejected %u mag_gated %u mag_rejected %u",
		    f.ekf.acc_soft, f.ekf.acc_gated, f.ekf.acc_rejected, f.ekf.mag_gated,
		    f.ekf.mag_rejected);
	shell_print(sh, "ekf: yaw_resets %u tilt_resets %u gyro_sat %u numeric_faults %u",
		    f.ekf.yaw_resets, f.ekf.tilt_resets, f.ekf.gyro_sat, f.ekf.numeric_faults);
	shell_print(sh, "fusion: ekf_input_faults %u ekf_numeric_faults %u align_restarts %u",
		    f.fusion.ekf_input_faults, f.fusion.ekf_numeric_faults,
		    f.fusion.align_restarts);
	shell_print(sh, "sensors: frame_drops %u queue_drops %u drdy_timeouts %u bus_errors %u",
		    hub.stats.frame_drops, hub.stats.queue_drops, hub.stats.drdy_timeouts,
		    hub.stats.bus_errors);
	shell_print(sh, "sensors: mag_not_ready %u mag_saturated %u dt_fault %u",
		    hub.stats.mag_not_ready, hub.stats.mag_saturated, hub.stats.dt_fault);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(telemetry_cmds,
	SHELL_CMD_ARG(mode, NULL, "<raw|sensor|fusion|off>", cmd_telemetry_mode, 2, 0),
	SHELL_CMD_ARG(rate, NULL, "<1..50> records per second", cmd_telemetry_rate, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(telemetry, &telemetry_cmds, "Telemetry stream", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(mag_cmds,
	SHELL_CMD(offset, NULL, "Print the bridge offset", cmd_mag_offset),
	SHELL_CMD(refresh, NULL, "Re-measure the bridge offset if at rest", cmd_mag_refresh),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(mag, &mag_cmds, "Magnetometer", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(ekf_cmds,
	SHELL_CMD(align, NULL, "Restart the alignment", cmd_ekf_align),
	SHELL_CMD(stats, NULL, "Print state and health counters", cmd_ekf_stats),
	SHELL_CMD_ARG(mag, NULL, "<on|off> heading correction", cmd_ekf_mag, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(ekf, &ekf_cmds, "Attitude filter", NULL);
