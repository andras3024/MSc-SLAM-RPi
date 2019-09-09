#include "spi.h"

int spi_cs0_fd;				//file descriptor for the SPI device
int spi_cs1_fd;				//file descriptor for the SPI device
unsigned char spi_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;

// spi_device 0=CS0, 1=CS1
int spi_open_port (int spi_device) {
    int *spi_cs_fd;

    //----- SET SPI MODE -----
    //SPI_MODE_0 (0,0) 	CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
    //SPI_MODE_1 (0,1) 	CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_2 (1,0) 	CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
    //SPI_MODE_3 (1,1) 	CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
    spi_mode = SPI_MODE_0;
    //----- SET BITS PER WORD -----
    spi_bitsPerWord = 8;
    //----- SET SPI BUS SPEED -----
    spi_speed = 1000000;		//1000000 = 1MHz (1uS per bit) 

    if (spi_device) spi_cs_fd = &spi_cs1_fd;
    else spi_cs_fd = &spi_cs0_fd;

    if (spi_device)*spi_cs_fd = open("/dev/spidev0.1", O_RDWR);
    else *spi_cs_fd = open("/dev/spidev0.0", O_RDWR);

    if (*spi_cs_fd < 0) {
        perror("Error - Could not open SPI device");
        exit(1);
    }

    // Set SPI write mode
    if(ioctl(*spi_cs_fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }
    // Set SPI read mode
    if(ioctl(*spi_cs_fd, SPI_IOC_RD_MODE, &spi_mode) < 0) {
      perror("Could not set SPIMode (RD)...ioctl fail");
      exit(1);
    }
    // Set SPI write bits-per-word
    if(ioctl(*spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord) < 0) {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      exit(1);
    }
    // Set SPI read bits-per-word
    if(ioctl(*spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord) < 0) {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      exit(1);
    }
    // Set SPI write max speed (hz)
    if(ioctl(*spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
      perror("Could not set SPI speed (WR)...ioctl fail");
      exit(1);
    }
    // Set SPI read max speed (hz)
    if(ioctl(*spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed) < 0) {
      perror("Could not set SPI speed (RD)...ioctl fail");
      exit(1);
    }

    return(0);
}

//********** SPI CLOSE PORT **********
int spi_close_port (int spi_device) {
    int *spi_cs_fd;

    if (spi_device) spi_cs_fd = &spi_cs1_fd;
    else spi_cs_fd = &spi_cs0_fd;

    if(close(*spi_cs_fd) < 0) {
    	perror("Error - Could not close SPI device");
    	exit(1);
    }

    return(0);
}

// data	Bytes to write.  Contents is overwritten with bytes read.
int spi_write_read (int spi_device, unsigned char *data, int length) {
	struct spi_ioc_transfer spi[length];
    int *spi_cs_fd;
	int i;

    if (spi_device) spi_cs_fd = &spi_cs1_fd;
    else spi_cs_fd = &spi_cs0_fd;

	//one spi transfer for each byte
	for (i = 0 ; i < length ; i++) {
		//printf("->\t%i\t%x\n",i, data[i]);
		memset(&spi[i], 0, sizeof (spi[i]));
		spi[i].tx_buf        = (unsigned long)(data + i); // transmit from "data"
		spi[i].rx_buf        = (unsigned long)(data + i); // receive into "data"
		spi[i].len           = sizeof(*(data + i));
		spi[i].delay_usecs   = 0;
		spi[i].speed_hz      = spi_speed;
		spi[i].bits_per_word = spi_bitsPerWord;
		spi[i].cs_change = 0;
	}

	if(ioctl(*spi_cs_fd, SPI_IOC_MESSAGE(length), &spi) < 0) {
		perror("Error - Problem transmitting spi data..ioctl");
		exit(-2);
	}

	return(0);
}

