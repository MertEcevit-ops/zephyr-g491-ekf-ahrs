/* Runs the application's sensor hub under the hardware tests. */
#ifndef HUB_FIXTURE_H_
#define HUB_FIXTURE_H_

#include "sensor_hub/sensor_hub.h"

#define SAMPLES_PER_S 104U

/** Suite setup: checks the sensors, starts the sensor thread and the DRDY trigger once. */
void *hub_fixture_setup(void);

/**
 * Empty the fusion queue and wait until samples flow with a regular interval
 * again. The test thread does not read the queue between tests, which costs
 * queue drops and one dt fault; counters read after this call exclude them.
 */
void hub_fixture_resync(struct sensor_hub_stats *baseline);

#endif /* HUB_FIXTURE_H_ */
