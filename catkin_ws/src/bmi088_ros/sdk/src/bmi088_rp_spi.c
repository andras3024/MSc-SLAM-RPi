#include "bmi08x.h"
#include "bmi088.h"
#include "bmi088_rp_spi.h"
#include "spi.h"
#include <unistd.h>

#ifdef __cplusplus
extern "C"
{
#endif



#if BMI08X_FEATURE_BMI088 == 1

int8_t user_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    int8_t iError = 0;
    uint8_t buf[len*2+1];
    uint8_t index = 0;

    for(int i=0;i<len;i++) {
        index = i * 2;
        buf[index] = (reg_addr++);
        buf[index + 1] = *(data + i);
    }

    iError = spi_write_read(cs_pin, buf, len*2);
    return iError;
}

int8_t user_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    int32_t iError = 0;
    uint8_t buf[len+1];
	memset( buf, 0, (len+1)*sizeof(uint8_t) );

    buf[0] = reg_addr;

    iError = spi_write_read(cs_pin, buf, len+1);
    for(int i = 0;i < len; i++) {
        *(data + i) = buf[i+1];
    }

    return iError;
}

void user_delay_milli_sec(uint32_t msek) {
    usleep(msek*1000);
}

struct bmi08x_dev dev= {
        .accel_id = 0,
        .gyro_id = 1,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_ms = user_delay_milli_sec
};

#ifdef __cplusplus
extern "C"
{
#endif



#endif