# Application symbols, shared with the hardware test applications in tests/hw.

module = APP
module-str = app
source "subsys/logging/Kconfig.template.log_config"

config APP_ISM330_SPI_ONLY_INIT_PRIORITY
	int "ISM330DHCX I2C_disable hook priority"
	default 91
	help
	  POST_KERNEL priority of the hook that sets CTRL4_C.I2C_disable. It must
	  run after the ISM330DHCX driver (CONFIG_SENSOR_INIT_PRIORITY) and before
	  the MMC5983MA driver (CONFIG_MMC5983MA_INIT_PRIORITY).

config APP_EKF_USE_MAG
	bool "Correct heading with the magnetometer"
	default y
	help
	  When disabled, heading comes from the gyroscope alone.

config APP_MAG_DECLINATION_MDEG
	int "Magnetic declination D [millidegree, east positive]"
	default 0
	range -180000 180000
	help
	  0 makes yaw relative to magnetic north.

config APP_MAG_INCLINATION_MDEG
	int "Magnetic inclination I [millidegree, down positive]"
	default 56000
	range -90000 90000
	help
	  The default is the approximate value for Istanbul. Take the value for
	  the place of use from the NOAA WMM calculator.

config APP_TELEMETRY_RATE_HZ
	int "Telemetry stream rate at boot [Hz]"
	default 20
	range 1 50

config APP_MAG_OFFSET_REFRESH_S
	int "Seconds between magnetometer bridge offset refreshes"
	default 60
	range 1 86400
	help
	  The refresh blocks the sensor thread for about 11 ms and only runs
	  while the board is at rest.
