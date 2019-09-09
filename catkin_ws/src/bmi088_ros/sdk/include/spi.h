#ifndef RP_SPI
#define RP_SPI

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Needed for SPI port
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C"
{
#endif
int spi_open_port (int spi_device);
int spi_close_port (int spi_device);
int spi_write_read (int spi_device, unsigned char *data, int length);
#ifdef __cplusplus
}
#endif

#endif