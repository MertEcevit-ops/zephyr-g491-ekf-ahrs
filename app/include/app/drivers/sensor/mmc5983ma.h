/* Application-visible extensions of the MMC5983MA sensor driver. */
#ifndef APP_DRIVERS_SENSOR_MMC5983MA_H_
#define APP_DRIVERS_SENSOR_MMC5983MA_H_

#include <zephyr/drivers/sensor.h>

enum mmc5983ma_attribute {
	/**
	 * attr_set on SENSOR_CHAN_MAGN_XYZ: run the blocking SET/RESET bridge
	 * offset sequence (about 11 ms). The value argument is ignored. Call it
	 * only while the board is at rest.
	 */
	MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH = SENSOR_ATTR_PRIV_START,
	/**
	 * attr_get on SENSOR_CHAN_MAGN_X/Y/Z (one value) or SENSOR_CHAN_MAGN_XYZ
	 * (three values): current bridge offset in gauss, sensor frame.
	 */
	MMC5983MA_ATTR_BRIDGE_OFFSET,
};

#endif /* APP_DRIVERS_SENSOR_MMC5983MA_H_ */
