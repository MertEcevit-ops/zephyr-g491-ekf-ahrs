/* MMC5983MA driver internals. The application-facing part is <app/drivers/sensor/mmc5983ma.h>. */
#ifndef MMC5983MA_H_
#define MMC5983MA_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/* Mode 3, like the ISM330DHCX on the same bus; SCK idles high across both chip selects. */
#define MMC5983MA_SPI_OPERATION                                                                    \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

struct mmc5983ma_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec int_gpio; /* .port == NULL when absent */
};

struct mmc5983ma_data {
	struct k_mutex lock;
	uint8_t ctrl0;           /* shadow of the persistent bits only; strobes never land here */
	uint8_t ctrl1;
	int32_t offset[3];       /* bridge offset, counts, sensor frame */
	int32_t field_ug[3];     /* last field, micro-gauss, sensor frame */
	uint16_t meas_since_set;
	bool meas_in_flight;     /* a TM_M has been issued and its result not read yet */
	bool force_set;
};

#endif /* MMC5983MA_H_ */
