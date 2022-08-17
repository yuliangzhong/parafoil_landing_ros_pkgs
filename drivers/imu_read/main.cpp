#include "bmi08x.h"
#include "bmi088.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
// #include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
// #include <linux/ioctl.h>
// #include <sys/stat.h>
// #include <linux/types.h>
#include <linux/spi/spidev.h>

// #include <spidev_lib++.h>

using namespace std;

uint8_t mode = 0;
uint32_t speed = 100000;
uint8_t bits = 8;

typedef int8_t (*bmi_bus_read_function)(uint8_t, uint8_t *, uint32_t, void *);
typedef int8_t (*bmi_bus_write_function)(uint8_t, const uint8_t *, uint32_t, void *);

// dev->read(reg_addr, temp_buff, (len + dev->dummy_byte), dev->intf_ptr_accel);
int8_t spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len, uint8_t* fd) 
{
	struct spi_ioc_transfer spi[len];

	data[0] = reg_addr;
  	// printf("reg addr: [%d]\n", reg_addr);
	memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
	spi[0].tx_buf = (unsigned long long)((&reg_addr));
	spi[0].rx_buf = (unsigned long long)(data);
	spi[0].len = 1;
	spi[0].speed_hz = speed;
	spi[0].bits_per_word = bits;

	for (int i=1; i<len; ++i)
	{
		memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
		// spi[i].tx_buf = (unsigned long long)((&reg_addr)+i);
		spi[i].rx_buf = (unsigned long long)(data+i);
		spi[i].len = 1;
		spi[i].speed_hz = speed;
		spi[i].bits_per_word = bits;
	}
	int8_t rslt = ioctl(*fd, SPI_IOC_MESSAGE(len), spi);

	if(rslt < 0)
	{
		perror("Error transfering data over SPI bus");
		close(*fd);
		return -1;
	}
	return 0;
}

// dev->write(reg_addr, reg_data, len, dev->intf_ptr_accel);
int8_t spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, uint8_t* fd) 
{
	printf("in spi write \n");
	// len = len + 1;
    struct spi_ioc_transfer spi[len+1];
	// spi[0] is address

	memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
			spi[0].tx_buf = (unsigned long long)(&reg_addr);
			// spi[0].rx_buf = (unsigned long long)(&reg_addr);
			spi[0].len = 1;
			spi[0].speed_hz = speed;
			spi[0].bits_per_word = bits;
	
	// // spi[1] is data
	// memset(&spi[1], 0, sizeof(struct spi_ioc_transfer));
	// 		spi[1].tx_buf = (unsigned long long)(data);
	// 		spi[1].rx_buf = (unsigned long long)(data);
	// 		spi[1].len = 1;
	// 		spi[1].speed_hz = speed;
	// 		spi[1].bits_per_word = bits;

	for (int i=1; i<len+1; ++i)
	{
		memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
		spi[i].tx_buf = (unsigned long long)(data+i-1);
		spi[i].rx_buf = (unsigned long long)(data+i-1);
		spi[i].len = 1;
		spi[i].speed_hz = speed;
		spi[i].bits_per_word = bits;
	}
	int8_t rslt = ioctl(*fd, SPI_IOC_MESSAGE(len+1), spi);

	// for (int i=0; i<len; ++i)
	// {
	// 	printf("%d  ",data[i]);
	// }
	// printf("\n");

	if(rslt < 0)
	{
		perror("Error transfering data over SPI bus");
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

	// uint8_t reg_addr = 0x7D;
	// uint8_t data = 4;
	// uint32_t len = 1;
	// spi_write(reg_addr, &data, len, &acc_dev_addr);

    BMI088 device{
        .bus_read = (bmi_bus_read_function) &spi_read,
        .bus_write = (bmi_bus_write_function) &spi_write,
        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr
    };

    bool rslt = bmi088_init(&device);

	// close(acc_dev_addr);
	// close(gyro_dev_addr);
	printf("end!!\n");
    
	InertialMeasurement meas;
    for (int i=0;i<10;++i)
    {
        bmi088_sample(&device, &meas);
        sleep(1);
    }

    return 0;
    // spi_config_t spi_config;
    // spi_config.mode = 0;
    // spi_config.speed = 10000;
    // spi_config.delay = 0;
    // spi_config.bits_per_word = 8;

    // SPI acc_spi = SPI("/dev/spidev0.0", &spi_config);  
    // SPI gyro_spi = SPI("/dev/spidev0.1", &spi_config);  
    // acc_spi.begin();
    // gyro_spi.begin();

//     bmi08x_dev dev;
//     dev.intf_ptr_accel = &acc_dev_addr;
//     dev.intf_ptr_gyro = &gyro_dev_addr;
//     dev.read = (bmi_bus_read_function) &spi_read;
//     dev.write = (bmi_bus_write_function) &spi_write;
//     dev.delay_us = &delay_us;

//     // Set up the rest of the required fields
//     dev.variant = BMI088_VARIANT;
//     dev.intf = BMI08X_SPI_INTF;

    

//     int8_t rslt=0;
//     printf("rslt: %d \n", rslt);
//     rslt = bmi08a_init(&dev);
//     printf("final rslt: %d \n", rslt);

//     printf("rslt: %d \n", rslt);
//     rslt = bmi08g_init(&dev);
//     printf("final rslt: %d \n", rslt);

//     bmi08a_get_power_mode(&dev);
//     bmi08g_get_power_mode(&dev);
//     bmi08a_get_meas_conf(&dev);
//     bmi08g_get_meas_conf(&dev);
    
//     dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
//     dev.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;

//     dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
//     dev.gyro_cfg.odr = BMI08X_GYRO_BW_47_ODR_400_HZ;

//     // This is a wonky casting between the Bosch API, which usex
//     // 3G … 24G as 0x00 ... 0x03 and 2000 dps ... 125 dps as 0x04 … 0x00
//     dev.accel_cfg.range = 0x00; // (uint8_t)accel_fsr;
//     dev.gyro_cfg.range = 0x03;  // 4 - (uint8_t)gyro_fsr;

//     bmi08a_set_power_mode(&dev);
//     bmi08g_set_power_mode(&dev);
//     bmi08a_set_meas_conf(&dev);
//     bmi08g_set_meas_conf(&dev);

//     
}

// reference:
// https://www.youtube.com/watch?v=8hYCLlt7UdA
