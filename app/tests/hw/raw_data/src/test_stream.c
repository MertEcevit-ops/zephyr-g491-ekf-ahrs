/*
 * TEST_PLAN.md phase 2, T-RAW-01..04, 07 and 08: static statistics and timing
 * of the sample stream. Thresholds are for uncalibrated readings.
 */
#include <math.h>
#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/ztest.h>

#include "ekf/ekf_config.h"
#include "hub_fixture.h"

#define WINDOW_SAMPLES (10U * SAMPLES_PER_S)

/* A tick with a periodic SET sleeps for the 1 ms settle time inside the driver. */
#define SET_TICK_MIN_US       900
#define SET_TICK_MAX_US       1500
#define TICK_P99_MAX_US       300
#define TICK_HISTOGRAM_BIN_US 10
#define TICK_HISTOGRAM_BINS   (SET_TICK_MIN_US / TICK_HISTOGRAM_BIN_US)

#define ISM330_SPI_OPERATION                                                                       \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
#define ISM330_READ_INTERNAL_FREQ_FINE (BIT(7) | 0x63)
/* The 104 Hz setting divides the chip's 6667 Hz base rate by 64. */
#define ISM330_NOMINAL_PERIOD_US       (64.0e6f / 6667.0f)
#define ISM330_FREQ_FINE_STEP          0.0015f

static const struct spi_dt_spec ism330_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), ISM330_SPI_OPERATION);

/* Running mean and variance (Welford); sums of squares would cancel in single precision. */
struct running_stats {
	uint32_t count;
	float mean;
	float sq_dev_sum;
};

static void stats_add(struct running_stats *stats, float value)
{
	float delta = value - stats->mean;

	stats->count++;
	stats->mean += delta / (float)stats->count;
	stats->sq_dev_sum += delta * (value - stats->mean);
}

static float stats_std(const struct running_stats *stats)
{
	return (stats->count > 1U) ? sqrtf(stats->sq_dev_sum / (float)(stats->count - 1U)) : 0.0f;
}

struct window_stats {
	struct running_stats accel_a[3]; /* m/s^2, sensor frame */
	struct running_stats accel_b[3]; /* m/s^2, body frame */
	struct running_stats gyro_b[3];  /* rad/s */
	struct running_stats mag_m[3];   /* gauss, sensor frame */
	struct running_stats dt_us;
	uint32_t tick_histogram[TICK_HISTOGRAM_BINS];
	uint32_t set_ticks;
	uint32_t slowest_tick_us;
	struct sensor_hub_stats delta;   /* hub counters accumulated during the window */
};

static struct window_stats window;

static uint32_t cycles_to_us(uint32_t cycles)
{
	return (uint32_t)(((uint64_t)cycles * USEC_PER_SEC) / sys_clock_hw_cycles_per_sec());
}

/*
 * The test thread is cooperative and blocked on the queue, so it runs the
 * moment the sensor thread enqueues: the cycle count taken here bounds the
 * sensor thread's tick time (DRDY timestamp to k_msgq_put) from above.
 */
static void record_tick_time(uint32_t tick_us)
{
	window.slowest_tick_us = MAX(window.slowest_tick_us, tick_us);

	if (tick_us >= SET_TICK_MIN_US) {
		window.set_ticks++;
	} else {
		window.tick_histogram[tick_us / TICK_HISTOGRAM_BIN_US]++;
	}
}

static void collect_window(uint32_t samples)
{
	struct sensor_hub_stats baseline;
	struct sensor_hub_snapshot snapshot;
	struct imu_sample sample;

	window = (struct window_stats){ 0 };
	hub_fixture_resync(&baseline);

	for (uint32_t n = 0; n < samples; n++) {
		zassert_ok(sensor_hub_get_sample(&sample, K_MSEC(200)), "stream stopped at %u", n);
		record_tick_time(cycles_to_us(k_cycle_get_32() - sample.timestamp_cyc));

		sensor_hub_get_snapshot(&snapshot);
		stats_add(&window.dt_us, sample.dt_s * 1.0e6f);
		for (int i = 0; i < 3; i++) {
			stats_add(&window.accel_a[i], snapshot.accel_a[i]);
			stats_add(&window.accel_b[i], sample.accel[i]);
			stats_add(&window.gyro_b[i], sample.gyro[i]);
			stats_add(&window.mag_m[i], snapshot.mag_m[i]);
		}
	}

	sensor_hub_get_snapshot(&snapshot);
	window.delta.frame_drops = snapshot.stats.frame_drops - baseline.frame_drops;
	window.delta.queue_drops = snapshot.stats.queue_drops - baseline.queue_drops;
	window.delta.drdy_timeouts = snapshot.stats.drdy_timeouts - baseline.drdy_timeouts;
	window.delta.bus_errors = snapshot.stats.bus_errors - baseline.bus_errors;
	window.delta.mag_not_ready = snapshot.stats.mag_not_ready - baseline.mag_not_ready;
	window.delta.mag_saturated = snapshot.stats.mag_saturated - baseline.mag_saturated;
	window.delta.dt_fault = snapshot.stats.dt_fault - baseline.dt_fault;
}

static float mean_norm(const struct running_stats axes[3])
{
	return sqrtf(axes[0].mean * axes[0].mean + axes[1].mean * axes[1].mean +
		     axes[2].mean * axes[2].mean);
}

static int milli(float value)
{
	return (int)(value * 1000.0f);
}

static void print_axes(const char *label, const struct running_stats axes[3], float scale)
{
	printk("%s mean %d %d %d, std %d %d %d\n", label, (int)(axes[0].mean * scale),
	       (int)(axes[1].mean * scale), (int)(axes[2].mean * scale),
	       (int)(stats_std(&axes[0]) * scale), (int)(stats_std(&axes[1]) * scale),
	       (int)(stats_std(&axes[2]) * scale));
}

static void assert_no_stream_faults(void)
{
	zassert_equal(window.delta.dt_fault, 0, "dt_fault %u", window.delta.dt_fault);
	zassert_equal(window.delta.queue_drops, 0, "queue_drops %u", window.delta.queue_drops);
	zassert_equal(window.delta.frame_drops, 0, "frame_drops %u", window.delta.frame_drops);
	zassert_equal(window.delta.bus_errors, 0, "bus_errors %u", window.delta.bus_errors);
	zassert_equal(window.delta.mag_not_ready, 0, "mag -EBUSY %u", window.delta.mag_not_ready);
}

/* T-RAW-01 and T-RAW-02 */
ZTEST(raw_stream, test_accelerometer_reads_gravity_with_frd_sign)
{
	float norm;

	collect_window(WINDOW_SAMPLES);
	print_axes("accel A [mm/s^2]", window.accel_a, 1.0e3f);
	print_axes("accel b [mm/s^2]", window.accel_b, 1.0e3f);

	norm = mean_norm(window.accel_a);
	zassert_true(fabsf(window.accel_a[0].mean) < 0.8f);
	zassert_true(fabsf(window.accel_a[1].mean) < 0.8f);
	zassert_true(window.accel_a[2].mean >= 9.0f && window.accel_a[2].mean <= 10.6f);
	zassert_true(norm >= 9.3f && norm <= 10.3f, "norm %d mm/s^2", milli(norm));
	for (int i = 0; i < 3; i++) {
		zassert_true(stats_std(&window.accel_a[i]) < 0.02f, "axis %d", i);
	}

	/* Flat and component side up, the body z axis (down) reads -g. */
	zassert_true(window.accel_b[2].mean < -9.0f);
}

/* T-RAW-03 */
ZTEST(raw_stream, test_gyroscope_is_quiet_at_rest)
{
	collect_window(WINDOW_SAMPLES);
	print_axes("gyro b [urad/s]", window.gyro_b, 1.0e6f);

	for (int i = 0; i < 3; i++) {
		zassert_true(fabsf(window.gyro_b[i].mean) < 0.06f, "axis %d mean", i);
		zassert_true(stats_std(&window.gyro_b[i]) < 0.005f, "axis %d std", i);
	}
}

/* T-RAW-04 */
ZTEST(raw_stream, test_magnetometer_reads_a_plausible_earth_field)
{
	float norm;

	collect_window(WINDOW_SAMPLES);
	print_axes("mag M [uG]", window.mag_m, 1.0e6f);

	norm = mean_norm(window.mag_m);
	zassert_true(norm >= 0.2f && norm <= 0.8f, "norm %d mG", milli(norm));
	for (int i = 0; i < 3; i++) {
		zassert_true(stats_std(&window.mag_m[i]) < 2.0e-3f, "axis %d std", i);
	}
	zassert_equal(window.delta.mag_not_ready, 0, "-EBUSY %u", window.delta.mag_not_ready);
	zassert_equal(window.delta.mag_saturated, 0, "-ERANGE %u", window.delta.mag_saturated);
}

static uint32_t tick_percentile_us(uint32_t percent)
{
	uint32_t regular_ticks = 0, seen = 0, threshold;

	for (int bin = 0; bin < TICK_HISTOGRAM_BINS; bin++) {
		regular_ticks += window.tick_histogram[bin];
	}
	threshold = (regular_ticks * percent + 99U) / 100U;

	for (int bin = 0; bin < TICK_HISTOGRAM_BINS; bin++) {
		seen += window.tick_histogram[bin];
		if (seen >= threshold) {
			return (uint32_t)(bin + 1) * TICK_HISTOGRAM_BIN_US;
		}
	}

	return SET_TICK_MIN_US;
}

/*
 * The sensor clocks itself from an internal oscillator, so its real output
 * data rate differs from board to board. INTERNAL_FREQ_FINE holds this chip's
 * deviation from nominal in steps of 0.15 %.
 */
static float expected_sample_interval_us(void)
{
	uint8_t cmd = ISM330_READ_INTERNAL_FREQ_FINE;
	uint8_t freq_fine = 0;
	const struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 },
		{ .buf = &freq_fine, .len = 1 },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	zassert_ok(spi_transceive_dt(&ism330_spi, &tx, &rx));

	return ISM330_NOMINAL_PERIOD_US / (1.0f + ISM330_FREQ_FINE_STEP * (float)(int8_t)freq_fine);
}

/* T-RAW-07 */
ZTEST(raw_stream, test_sample_interval_and_tick_time_meet_budget)
{
	const float expected_us = expected_sample_interval_us();
	/* One SET every CONFIG_MMC5983MA_PERIODIC_SET_INTERVAL measurements, plus the edges. */
	const uint32_t set_ticks_max =
		WINDOW_SAMPLES / CONFIG_MMC5983MA_PERIODIC_SET_INTERVAL + 2U;
	uint32_t p99_us;

	collect_window(WINDOW_SAMPLES);
	p99_us = tick_percentile_us(99);
	printk("dt mean %d us (chip trim predicts %d us), std %d ns; tick p99 <= %u us, "
	       "slowest %u us, SET ticks %u\n",
	       (int)window.dt_us.mean, (int)expected_us, (int)(stats_std(&window.dt_us) * 1.0e3f),
	       p99_us, window.slowest_tick_us, window.set_ticks);

	zassert_within(window.dt_us.mean, expected_us, 0.02f * expected_us);
	zassert_true(stats_std(&window.dt_us) < 50.0f);
	assert_no_stream_faults();

	zassert_true(p99_us < TICK_P99_MAX_US, "tick p99 %u us", p99_us);
	zassert_true(window.slowest_tick_us < SET_TICK_MAX_US, "slowest tick %u us",
		     window.slowest_tick_us);
	zassert_true(window.set_ticks <= set_ticks_max, "%u ticks above %d us", window.set_ticks,
		     SET_TICK_MIN_US);
}

/* T-RAW-08; named so that it runs last. */
ZTEST(raw_stream, test_zz_long_run_has_no_faults)
{
	collect_window((uint32_t)CONFIG_RAW_DATA_LONG_RUN_S * SAMPLES_PER_S);
	printk("long run: %u samples, slowest tick %u us\n", window.dt_us.count,
	       window.slowest_tick_us);

	assert_no_stream_faults();
	zassert_equal(window.delta.drdy_timeouts, 0);
}

ZTEST_SUITE(raw_stream, NULL, hub_fixture_setup, NULL, NULL, NULL);
