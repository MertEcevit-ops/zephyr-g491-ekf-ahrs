/*
 * MEMSIC MMC5983MA magnetometer, SPI only (MMC5983MA_SPEC.md).
 *
 * One-shot measurements run as a pipeline: every sample_fetch() reads the
 * measurement started by the previous call and starts the next one, so the
 * caller never waits for a conversion. Thread context only.
 */
#define DT_DRV_COMPAT memsic_mmc5983ma

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <app/drivers/sensor/mmc5983ma.h>

#include "mmc5983ma.h"
#include "mmc5983ma_decode.h"
#include "mmc5983ma_reg.h"

LOG_MODULE_REGISTER(mmc5983ma, CONFIG_SENSOR_LOG_LEVEL);

#define UG_PER_GAUSS 1000000

/* The datasheet gives no bridge offset range; 1 G is merely worth a warning. */
#define OFFSET_WARN_COUNTS 16384

#define MEASUREMENT_POLLS_MAX                                                                      \
	(CONFIG_MMC5983MA_MEAS_TIMEOUT_MS * USEC_PER_MSEC / MMC5983MA_POLL_INTERVAL_US)

static int read_registers(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *dst, size_t len)
{
	uint8_t cmd = MMC5983MA_SPI_READ | (reg & MMC5983MA_SPI_ADDR_MASK);
	const struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 }, /* clocked in while the command byte goes out */
		{ .buf = dst, .len = len },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	if (len > MMC5983MA_MEASUREMENT_LEN) {
		return -EINVAL;
	}

	return spi_transceive_dt(bus, &tx, &rx) ? -EIO : 0;
}

static int write_register(const struct spi_dt_spec *bus, uint8_t reg, uint8_t val)
{
	uint8_t frame[2] = { reg & MMC5983MA_SPI_ADDR_MASK, val };
	const struct spi_buf tx_buf = { .buf = frame, .len = sizeof(frame) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(bus, &tx) ? -EIO : 0;
}

/*
 * CTRL0 cannot be read back. Writing a strobe without the shadow would clear
 * INT_meas_done_en; storing a strobe in the shadow would repeat it forever.
 */
static int strobe_ctrl0(const struct device *dev, uint8_t strobe)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;

	return write_register(&cfg->bus, MMC5983MA_REG_CTRL0, data->ctrl0 | strobe);
}

static int start_measurement(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	int rc = strobe_ctrl0(dev, MMC5983MA_CTRL0_TM_M);

	if (rc == 0) {
		data->meas_in_flight = true;
	}

	return rc;
}

static int measurement_done(const struct device *dev, bool *done)
{
	const struct mmc5983ma_config *cfg = dev->config;
	uint8_t status;
	int rc = read_registers(&cfg->bus, MMC5983MA_REG_STATUS, &status, 1);

	if (rc == 0) {
		*done = (status & MMC5983MA_STATUS_MEAS_M_DONE) != 0U;
	}

	return rc;
}

static int wait_measurement_done(const struct device *dev)
{
	bool done = false;

	for (uint32_t poll = 0; poll < MEASUREMENT_POLLS_MAX; poll++) {
		int rc;

		k_usleep(MMC5983MA_POLL_INTERVAL_US);
		rc = measurement_done(dev, &done);
		if (rc != 0) {
			return rc;
		}
		if (done) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int read_measurement(const struct device *dev, uint8_t raw[MMC5983MA_MEASUREMENT_LEN])
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	int rc = read_registers(&cfg->bus, MMC5983MA_REG_XOUT0, raw, MMC5983MA_MEASUREMENT_LEN);

	if (rc != 0) {
		return rc;
	}
	data->meas_in_flight = false;

	if (IS_ENABLED(CONFIG_MMC5983MA_INT_DIAG)) {
		/* Writing 1 to Meas_M_Done releases the INT pin. */
		rc = write_register(&cfg->bus, MMC5983MA_REG_STATUS, MMC5983MA_STATUS_MEAS_M_DONE);
	}

	return rc;
}

/* SET or RESET coil pulse followed by the settling time the datasheet leaves open. */
static int pulse_coil(const struct device *dev, uint8_t strobe)
{
	int rc = strobe_ctrl0(dev, strobe);

	if (rc == 0) {
		k_usleep(CONFIG_MMC5983MA_SR_SETTLE_US);
	}

	return rc;
}

static int measure_blocking(const struct device *dev, int32_t counts[3])
{
	uint8_t raw[MMC5983MA_MEASUREMENT_LEN];
	int rc = start_measurement(dev);

	if (rc == 0) {
		rc = wait_measurement_done(dev);
	}
	if (rc == 0) {
		rc = read_measurement(dev, raw);
	}
	if (rc != 0) {
		return rc;
	}
	if (mmc5983ma_raw_saturated(raw)) {
		return -ERANGE;
	}

	mmc5983ma_decode_18bit(raw, counts);
	return 0;
}

/*
 * The datasheet axis polarity holds after SET, and normal samples assume it.
 * Leaves a measurement in flight so that the pipeline continues seamlessly.
 */
static int magnetise_and_restart_pipeline(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	int rc = pulse_coil(dev, MMC5983MA_CTRL0_SET);

	if (rc != 0) {
		return rc;
	}

	data->meas_since_set = 0;
	data->force_set = false;

	return start_measurement(dev);
}

static void store_bridge_offset(const struct device *dev, const int32_t after_set[3],
				const int32_t after_reset[3])
{
	struct mmc5983ma_data *data = dev->data;

	/* SET reads +H + offset, RESET reads -H + offset: the mean is the offset. */
	for (int i = 0; i < 3; i++) {
		data->offset[i] = (after_set[i] + after_reset[i]) / 2;
		if (data->offset[i] > OFFSET_WARN_COUNTS || data->offset[i] < -OFFSET_WARN_COUNTS) {
			LOG_WRN("axis %d bridge offset is %d counts", i, data->offset[i]);
		}
	}
}

/* MMC5983MA_SPEC 8.3. Blocks for about 11 ms. */
static int refresh_bridge_offset(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	int32_t after_set[3], after_reset[3];
	int rc, restart_rc;

	if (data->meas_in_flight) {
		bool done;

		rc = measurement_done(dev, &done);
		if (rc == 0 && !done) {
			rc = wait_measurement_done(dev);
		}
		if (rc != 0) {
			return rc;
		}
		data->meas_in_flight = false;
	}

	rc = pulse_coil(dev, MMC5983MA_CTRL0_SET);
	if (rc == 0) {
		rc = measure_blocking(dev, after_set);
	}
	if (rc == 0) {
		rc = pulse_coil(dev, MMC5983MA_CTRL0_RESET);
	}
	if (rc == 0) {
		rc = measure_blocking(dev, after_reset);
	}
	if (rc == 0) {
		store_bridge_offset(dev, after_set, after_reset);
	} else if (rc != -ERANGE) {
		return rc;
	}

	/* Also after a saturated pass: the sensor may sit in the RESET state. */
	restart_rc = magnetise_and_restart_pipeline(dev);

	return (restart_rc != 0) ? restart_rc : rc;
}

static int fetch_pipelined(const struct device *dev)
{
	struct mmc5983ma_data *data = dev->data;
	uint8_t raw[MMC5983MA_MEASUREMENT_LEN];
	int32_t counts[3];
	bool done;
	int result = 0;
	int rc;

	if (!data->meas_in_flight) {
		rc = start_measurement(dev);
		/* Nothing has been measured for this call; stale data is never returned. */
		return (rc == 0) ? -EBUSY : rc;
	}

	rc = measurement_done(dev, &done);
	if (rc != 0) {
		return rc;
	}
	if (!done) {
		return -EBUSY;
	}

	rc = read_measurement(dev, raw);
	if (rc != 0) {
		return rc;
	}

	if (mmc5983ma_raw_saturated(raw)) {
		/* A field above 8 G can upset the film magnetisation; restore it. */
		data->force_set = true;
		result = -ERANGE;
	} else {
		mmc5983ma_decode_18bit(raw, counts);
		for (int i = 0; i < 3; i++) {
			data->field_ug[i] =
				(int32_t)mmc5983ma_counts_to_ug(counts[i] - data->offset[i]);
		}
	}

	data->meas_since_set++;
	if (data->meas_since_set >= CONFIG_MMC5983MA_PERIODIC_SET_INTERVAL || data->force_set) {
		rc = magnetise_and_restart_pipeline(dev);
	} else {
		rc = start_measurement(dev);
	}

	return (rc != 0) ? rc : result;
}

static int mmc5983ma_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mmc5983ma_data *data = dev->data;
	int rc;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_MAGN_XYZ) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	rc = fetch_pipelined(dev);
	k_mutex_unlock(&data->lock);

	return rc;
}

static void ug_to_sensor_value(int64_t micro_gauss, struct sensor_value *val)
{
	/* Both parts carry the sign, as struct sensor_value requires. */
	val->val1 = (int32_t)(micro_gauss / UG_PER_GAUSS);
	val->val2 = (int32_t)(micro_gauss % UG_PER_GAUSS);
}

/* Index of the first axis and the axis count addressed by a magnetometer channel. */
static int magn_channel_axes(enum sensor_channel chan, int *first, int *count)
{
	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
		*first = chan - SENSOR_CHAN_MAGN_X;
		*count = 1;
		return 0;
	case SENSOR_CHAN_MAGN_XYZ:
		*first = 0;
		*count = 3;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int mmc5983ma_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;
	int first, count;

	if (magn_channel_axes(chan, &first, &count) != 0) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < count; i++) {
		ug_to_sensor_value(data->field_ug[first + i], &val[i]);
	}
	k_mutex_unlock(&data->lock);

	return 0;
}

static int mmc5983ma_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;
	int rc;

	ARG_UNUSED(val);

	if (chan != SENSOR_CHAN_MAGN_XYZ || (int)attr != MMC5983MA_ATTR_BRIDGE_OFFSET_REFRESH) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	rc = refresh_bridge_offset(dev);
	k_mutex_unlock(&data->lock);

	return rc;
}

static int mmc5983ma_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	struct mmc5983ma_data *data = dev->data;
	int first, count;

	if ((int)attr != MMC5983MA_ATTR_BRIDGE_OFFSET ||
	    magn_channel_axes(chan, &first, &count) != 0) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < count; i++) {
		ug_to_sensor_value(mmc5983ma_counts_to_ug(data->offset[first + i]), &val[i]);
	}
	k_mutex_unlock(&data->lock);

	return 0;
}

static DEVICE_API(sensor, mmc5983ma_api) = {
	.attr_set = mmc5983ma_attr_set,
	.attr_get = mmc5983ma_attr_get,
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
};

/* Software reset, then identity and OTP checks. Clears every write-only register. */
static int reset_and_identify(const struct device *dev)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	uint8_t product_id, status;
	int rc;

	rc = write_register(&cfg->bus, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
	if (rc != 0) {
		return rc;
	}
	k_msleep(MMC5983MA_RESET_TIME_MS);
	data->ctrl0 = 0;
	data->ctrl1 = 0;

	rc = read_registers(&cfg->bus, MMC5983MA_REG_PRODUCT_ID, &product_id, 1);
	if (rc != 0) {
		return rc;
	}
	if (product_id != MMC5983MA_PRODUCT_ID) {
		LOG_ERR("unexpected product id 0x%02x", product_id);
		return -ENODEV;
	}

	rc = read_registers(&cfg->bus, MMC5983MA_REG_STATUS, &status, 1);
	if (rc != 0) {
		return rc;
	}
	if ((status & MMC5983MA_STATUS_OTP_READ_DONE) == 0U) {
		LOG_ERR("factory OTP data not loaded (status 0x%02x)", status);
		return -EIO;
	}

	return 0;
}

static int enable_int_diagnostics(const struct device *dev)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;

	if (cfg->int_gpio.port == NULL || !gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("INT diagnostics enabled without a usable int-gpios");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT) != 0) {
		return -EIO;
	}

	data->ctrl0 |= MMC5983MA_CTRL0_INT_MEAS_DONE_EN;
	return 0;
}

static int mmc5983ma_init(const struct device *dev)
{
	const struct mmc5983ma_config *cfg = dev->config;
	struct mmc5983ma_data *data = dev->data;
	int rc;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}
	k_mutex_init(&data->lock);

	rc = reset_and_identify(dev);
	if (rc != 0) {
		return rc;
	}

	data->ctrl1 = FIELD_PREP(MMC5983MA_CTRL1_BW_MASK, CONFIG_MMC5983MA_BANDWIDTH);
	rc = write_register(&cfg->bus, MMC5983MA_REG_CTRL1, data->ctrl1);
	if (rc != 0) {
		return rc;
	}

	if (IS_ENABLED(CONFIG_MMC5983MA_INT_DIAG)) {
		rc = enable_int_diagnostics(dev);
		if (rc != 0) {
			return rc;
		}
	}

	rc = refresh_bridge_offset(dev);
	if (rc == -ERANGE) {
		/* A magnet near the board at power-up; the first refresh at rest fixes it. */
		LOG_WRN("field saturated during init, bridge offset left at zero");
		memset(data->offset, 0, sizeof(data->offset));
		rc = 0;
	}

	return rc;
}

#define MMC5983MA_DEFINE(inst)                                                                     \
	static struct mmc5983ma_data mmc5983ma_data_##inst;                                        \
	static const struct mmc5983ma_config mmc5983ma_config_##inst = {                           \
		.bus = SPI_DT_SPEC_INST_GET(inst, MMC5983MA_SPI_OPERATION),                        \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),                      \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mmc5983ma_init, NULL, &mmc5983ma_data_##inst,           \
				     &mmc5983ma_config_##inst, POST_KERNEL,                        \
				     CONFIG_MMC5983MA_INIT_PRIORITY, &mmc5983ma_api);

DT_INST_FOREACH_STATUS_OKAY(MMC5983MA_DEFINE)
