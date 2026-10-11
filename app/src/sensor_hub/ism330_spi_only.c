/*
 * Sets CTRL4_C.I2C_disable on the ISM330DHCX.
 *
 * With CS high the chip keeps listening on SCK/MOSI as an I2C slave, so the
 * MMC5983MA's SPI traffic on the shared lines could address it. ST's start-up
 * procedure for SPI use writes this bit; the upstream Zephyr driver does not
 * (HARDWARE_MAP.md section 4). The hook runs between the two sensor drivers.
 */
#include <errno.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define ISM330_SPI_OPERATION                                                                       \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define ISM330_SPI_READ          BIT(7)
#define ISM330_REG_CTRL4_C       0x13
#define ISM330_CTRL4_I2C_DISABLE BIT(2)
/* Reserved bits of CTRL4_C (not_used_01..03 in ST's ism330dhcx_reg.h); they stay 0. */
#define ISM330_CTRL4_RESERVED    (BIT(7) | BIT(4) | BIT(0))

BUILD_ASSERT(CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY > CONFIG_SENSOR_INIT_PRIORITY,
	     "I2C_disable must be written after the ISM330DHCX driver resets the chip");
#ifdef CONFIG_MMC5983MA
BUILD_ASSERT(CONFIG_MMC5983MA_INIT_PRIORITY > CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY,
	     "MMC5983MA traffic must start after the ISM330DHCX I2C block is disabled");
#endif

static const struct spi_dt_spec ism330_spi =
	SPI_DT_SPEC_GET(DT_NODELABEL(ism330dhcx), ISM330_SPI_OPERATION);

static int read_ctrl4(uint8_t *value)
{
	uint8_t cmd = ISM330_SPI_READ | ISM330_REG_CTRL4_C;
	const struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 },
		{ .buf = value, .len = 1 },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	return spi_transceive_dt(&ism330_spi, &tx, &rx) ? -EIO : 0;
}

static int write_ctrl4(uint8_t value)
{
	uint8_t frame[2] = { ISM330_REG_CTRL4_C, value };
	const struct spi_buf tx_buf = { .buf = frame, .len = sizeof(frame) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(&ism330_spi, &tx) ? -EIO : 0;
}

static int ism330_disable_i2c(void)
{
	uint8_t ctrl4;
	int rc;

	if (!spi_is_ready_dt(&ism330_spi)) {
		return -ENODEV;
	}

	rc = read_ctrl4(&ctrl4);
	if (rc != 0) {
		return rc;
	}

	ctrl4 |= ISM330_CTRL4_I2C_DISABLE;
	ctrl4 &= (uint8_t)~ISM330_CTRL4_RESERVED;

	return write_ctrl4(ctrl4);
}

SYS_INIT(ism330_disable_i2c, POST_KERNEL, CONFIG_APP_ISM330_SPI_ONLY_INIT_PRIORITY);
