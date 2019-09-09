#ifndef BMI088_RP_SPI
#define BMI088_RP_SPI

#ifdef __cplusplus
extern "C"
{
#endif

/* header files */
#include "bmi08x_defs.h"
#if BMI08X_FEATURE_BMI088 == 1



int8_t user_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t user_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

void user_delay_milli_sec(uint32_t msek);

extern struct bmi08x_dev dev;

#endif
#ifdef __cplusplus
}
#endif

#endif /* BMI088_RP_SPI */