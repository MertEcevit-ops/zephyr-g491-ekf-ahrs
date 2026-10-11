#include "hub_fixture.h"

#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define SENSOR_STACK_SIZE 1024
#define SENSOR_PRIORITY   K_PRIO_PREEMPT(2)
#define RESYNC_SAMPLES    4

/* Same stack size, priority and options as the application's sensor thread. */
K_THREAD_DEFINE(sensor_thread, SENSOR_STACK_SIZE, sensor_hub_run, NULL, NULL, NULL,
		SENSOR_PRIORITY, K_FP_REGS, SYS_FOREVER_MS);

void *hub_fixture_setup(void)
{
	static bool started;

	if (!started) {
		zassert_ok(sensor_hub_init());
		k_thread_start(sensor_thread);
		zassert_ok(sensor_hub_enable_trigger());
		started = true;
	}

	return NULL;
}

void hub_fixture_resync(struct sensor_hub_stats *baseline)
{
	struct sensor_hub_snapshot snapshot;
	struct imu_sample sample;

	while (sensor_hub_get_sample(&sample, K_NO_WAIT) == 0) {
	}
	for (int i = 0; i < RESYNC_SAMPLES; i++) {
		zassert_ok(sensor_hub_get_sample(&sample, K_MSEC(200)), "no samples from the hub");
	}

	sensor_hub_get_snapshot(&snapshot);
	*baseline = snapshot.stats;
}
