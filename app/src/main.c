/* Start-up: readiness checks, the three application threads, then the DRDY trigger. */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fusion/fusion.h"
#include "sensor_hub/sensor_hub.h"
#include "telemetry/telemetry.h"

LOG_MODULE_REGISTER(app_main, CONFIG_APP_LOG_LEVEL);

#define SENSOR_STACK_SIZE    1024
#define FUSION_STACK_SIZE    2048
#define TELEMETRY_STACK_SIZE 2048

#define SENSOR_PRIORITY    K_PRIO_PREEMPT(2)
#define FUSION_PRIORITY    K_PRIO_PREEMPT(3)
#define TELEMETRY_PRIORITY K_PRIO_PREEMPT(5)

/* Created stopped; main() starts them once the sensors are known to be alive. */
K_THREAD_DEFINE(sensor_thread, SENSOR_STACK_SIZE, sensor_hub_run, NULL, NULL, NULL,
		SENSOR_PRIORITY, K_FP_REGS, SYS_FOREVER_MS);
K_THREAD_DEFINE(fusion_thread, FUSION_STACK_SIZE, fusion_run, NULL, NULL, NULL, FUSION_PRIORITY,
		K_FP_REGS, SYS_FOREVER_MS);
K_THREAD_DEFINE(telemetry_thread, TELEMETRY_STACK_SIZE, telemetry_run, NULL, NULL, NULL,
		TELEMETRY_PRIORITY, K_FP_REGS, SYS_FOREVER_MS);

int main(void)
{
	int rc = sensor_hub_init();

	if (rc != 0) {
		LOG_ERR("sensors not ready (%d), nothing started", rc);
		return rc;
	}

	/* Consumers first, so that the first sample already finds its reader. */
	k_thread_start(telemetry_thread);
	k_thread_start(fusion_thread);
	k_thread_start(sensor_thread);

	rc = sensor_hub_enable_trigger();
	if (rc != 0) {
		LOG_ERR("cannot enable the data-ready trigger (%d)", rc);
	}

	return rc;
}
