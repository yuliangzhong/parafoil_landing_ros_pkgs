#include "bmi08x.h"
#include "bmi088.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

using namespace std;

uint8_t mode = 0;
uint32_t speed = 100000;
uint8_t bits = 8;

typedef int8_t (*bmi_bus_read_function)(uint8_t, uint8_t *, uint32_t, void *);
typedef int8_t (*bmi_bus_write_function)(uint8_t, const uint8_t *, uint32_t, void *);

int8_t spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len, uint8_t* fd) 
{
	struct spi_ioc_transfer spi[len];

	data[0] = reg_addr;
	memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
	spi[0].tx_buf = (unsigned long long)((&reg_addr));
	// spi[0].rx_buf = (unsigned long long)(data);
	spi[0].len = 1;
	spi[0].speed_hz = speed;
	spi[0].bits_per_word = bits;

	for (int i=1; i<len; ++i)
	{
		memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
		spi[i].rx_buf = (unsigned long long)(data+i);
		spi[i].len = 1;
		spi[i].speed_hz = speed;
		spi[i].bits_per_word = bits;
	}
	int8_t rslt = ioctl(*fd, SPI_IOC_MESSAGE(len), spi);

	if(rslt < 0)
	{
		perror("Error transfering data over SPI bus\n");
		close(*fd);
		return -1;
	}

	return 0;
}

int8_t spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, uint8_t* fd) 
{
    struct spi_ioc_transfer spi[len+1];

	memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
			spi[0].tx_buf = (unsigned long long)(&reg_addr);
			spi[0].len = 1;
			spi[0].speed_hz = speed;
			spi[0].bits_per_word = bits;

	for (int i=1; i<len+1; ++i)
	{
		memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
		spi[i].tx_buf = (unsigned long long)(data+i-1);
		// spi[i].rx_buf = (unsigned long long)(data+i-1);
		spi[i].len = 1;
		spi[i].speed_hz = speed;
		spi[i].bits_per_word = bits;
	}
	int8_t rslt = ioctl(*fd, SPI_IOC_MESSAGE(len+1), spi);

	if(rslt < 0)
	{
		perror("Error transfering data over SPI bus\n");
		close(*fd);
		return -1;
	}

	return 0;
}


static void delay_us(uint32_t us, void *intf_ptr) { usleep(us); }


int main()
{

	uint8_t acc_dev_addr = 0;
	acc_dev_addr = open("/dev/spidev0.0", O_RDWR);
	uint8_t gyro_dev_addr = 0;
	gyro_dev_addr = open("/dev/spidev0.1", O_RDWR);

	if(acc_dev_addr < 0 || gyro_dev_addr < 0)
	{
		perror("Error opening SPI bus");
		return -1;
	}
	
  	int fds[2] = {acc_dev_addr, gyro_dev_addr};
  	for (int i = 0; i < 2; i++) {
    int fd = fds[i];
    if(ioctl(fd, SPI_IOC_WR_MODE, &mode)<0)
    {
      perror("bad mode\n");
      close(fd);
      return -1;
    }

    if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)
    {
      perror("err set speed\n");
      close(fd);
      return -1;
    }

    if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)<0)
    {
      perror("err set bits per word\n");
      close(fd);
      return -1;
    }
  }

	printf("acc addr: %d, gyro addr: %d\n", acc_dev_addr, gyro_dev_addr);

    BMI088 device{
        .bus_read = (bmi_bus_read_function) &spi_read,
        .bus_write = (bmi_bus_write_function) &spi_write,
        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr
    };

    bool rslt = bmi088_init(&device);
    
	InertialMeasurement meas;
    for (int i=0;i<600;++i)
    {
        bmi088_sample(&device, &meas);
		printf("------------------------------\n");
        usleep(100000);
    }

    return 0;
}

// reference:
// https://www.youtube.com/watch?v=8hYCLlt7UdA
