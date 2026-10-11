/* Raw register access to both chips through the devicetree nodes the drivers use. */
#ifndef BUS_H_
#define BUS_H_

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#define BUS_OP_MODE3                                                                               \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
#define BUS_OP_MODE0 (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define BUS_READ BIT(7)

#define ISM330_REG_WHO_AM_I 0x0F
#define ISM330_REG_CTRL4_C  0x13
#define ISM330_WHO_AM_I     0x6B

static inline int bus_read(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *dst, size_t len)
{
	uint8_t cmd = BUS_READ | reg;
	const struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 },
		{ .buf = dst, .len = len },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	return spi_transceive_dt(bus, &tx, &rx) ? -EIO : 0;
}

static inline int bus_write(const struct spi_dt_spec *bus, uint8_t reg, uint8_t val)
{
	uint8_t frame[2] = { reg, val };
	const struct spi_buf tx_buf = { .buf = frame, .len = sizeof(frame) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(bus, &tx) ? -EIO : 0;
}

#endif /* BUS_H_ */
