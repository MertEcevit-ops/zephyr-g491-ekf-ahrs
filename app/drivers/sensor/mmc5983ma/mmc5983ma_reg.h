/* MMC5983MA register map and bit fields, MMC5983MA_SPEC.md sections 2 to 4. */
#ifndef MMC5983MA_REG_H_
#define MMC5983MA_REG_H_

#include <zephyr/sys/util.h>

/* Command byte: bit 7 selects read, bit 6 is don't-care and sent as 0. */
#define MMC5983MA_SPI_READ      BIT(7)
#define MMC5983MA_SPI_ADDR_MASK GENMASK(5, 0)

#define MMC5983MA_REG_XOUT0      0x00
#define MMC5983MA_REG_XYZOUT2    0x06
#define MMC5983MA_REG_TOUT       0x07
#define MMC5983MA_REG_STATUS     0x08
#define MMC5983MA_REG_CTRL0      0x09
#define MMC5983MA_REG_CTRL1      0x0A
#define MMC5983MA_REG_CTRL2      0x0B
#define MMC5983MA_REG_CTRL3      0x0C
#define MMC5983MA_REG_PRODUCT_ID 0x2F

/* Xout0..XYZout2, the only range that is read as a burst. */
#define MMC5983MA_MEASUREMENT_LEN 7

#define MMC5983MA_PRODUCT_ID 0x30

#define MMC5983MA_STATUS_MEAS_M_DONE   BIT(0)
#define MMC5983MA_STATUS_MEAS_T_DONE   BIT(1)
#define MMC5983MA_STATUS_OTP_READ_DONE BIT(4)

#define MMC5983MA_CTRL0_TM_M             BIT(0)
#define MMC5983MA_CTRL0_TM_T             BIT(1)
#define MMC5983MA_CTRL0_INT_MEAS_DONE_EN BIT(2)
#define MMC5983MA_CTRL0_SET              BIT(3)
#define MMC5983MA_CTRL0_RESET            BIT(4)
#define MMC5983MA_CTRL0_AUTO_SR_EN       BIT(5)
#define MMC5983MA_CTRL0_OTP_READ         BIT(6)

#define MMC5983MA_CTRL1_BW_MASK    GENMASK(1, 0)
#define MMC5983MA_CTRL1_X_INHIBIT  BIT(2)
#define MMC5983MA_CTRL1_YZ_INHIBIT GENMASK(4, 3)
#define MMC5983MA_CTRL1_SW_RST     BIT(7)

#define MMC5983MA_CTRL2_CM_FREQ_MASK GENMASK(2, 0)
#define MMC5983MA_CTRL2_CMM_EN       BIT(3)
#define MMC5983MA_CTRL2_PRD_SET_MASK GENMASK(6, 4)
#define MMC5983MA_CTRL2_EN_PRD_SET   BIT(7)

#define MMC5983MA_CTRL3_ST_ENP BIT(1)
#define MMC5983MA_CTRL3_ST_ENM BIT(2)
#define MMC5983MA_CTRL3_SPI_3W BIT(6)

/* Power-on and software-reset time. */
#define MMC5983MA_RESET_TIME_MS 10
/* Interval between STATUS reads while a blocking measurement runs. */
#define MMC5983MA_POLL_INTERVAL_US 500

#endif /* MMC5983MA_REG_H_ */
