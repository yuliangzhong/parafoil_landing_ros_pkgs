#include <iostream>

#include "bmi08x.h"
#include "bmi088.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <spidev_lib++.h>

using namespace std;

typedef int8_t (*bmi_bus_read_function)(uint8_t, uint8_t *, uint32_t, void *);
typedef int8_t (*bmi_bus_write_function)(uint8_t, const uint8_t *, uint32_t, void *);

// dev->read(reg_addr, temp_buff, (len + dev->dummy_byte), dev->intf_ptr_accel);
int8_t spi_read(uint8_t reg_addr, uint8_t *data, uint32_t len, SPI *handle) 
{
    uint8_t tx_buffer[len];
    uint8_t rx_buffer[len];
    if (handle->begin())
    {
        // memset(tx_buffer,0,len);
        // memset(rx_buffer,0,len);
        // sprintf((char*)tx_buffer,"hello world");
        // printf("sending %s, to spidev2.0 in full duplex \n ",(char*)tx_buffer);
        // handle->xfer(tx_buffer,strlen((char*)tx_buffer),rx_buffer,strlen((char*)tx_buffer));
        handle->xfer(&reg_addr, 1, data, len);

        printf("data=%s\n",(char *)data);
        return 0;
    }
    return -1;
}

int8_t spi_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, SPI *handle) 
{
    return 0;
}

// int8_t spi_master_bus_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
//                            struct SPIDeviceHandle *handle) {
//     // Set up new transaction, chip select pin to be pulled low is passed as the
//     // user field and then read in the .pre and .post callbacks
//     spi_transaction_t trans = {
//         .addr = reg_addr,
//         .rx_buffer = data,
//         .tx_buffer = NULL,
//         .length = len * 8,
//         .user = (void *)(handle->cs),
//     };

//     esp_err_t ret = spi_device_transmit(handle->drive_dev_handle, &trans);
//     ESP_LOGD(TAG, "Read (%u B) @ %#02x: %#02x (%d)", len, reg_addr, *data, ret);

//     return ret == ESP_OK ? 0 : -1;
// }
// int8_t spi_master_bus_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
//                             struct SPIDeviceHandle *handle) 




// int  main( void)
// {  
//     spi_config_t spi_config;
//     uint8_t tx_buffer[32];
//     uint8_t rx_buffer[32];

//     SPI *mySPI = NULL;
//     spi_config.mode=0;
//     spi_config.speed=10000;
//     spi_config.delay=0;
//     spi_config.bits_per_word=8;  
    
//     mySPI = new SPI("/dev/spidev0.0", &spi_config);  
    
//     if (mySPI->begin())
//     {
//         // memset(tx_buffer,1,32);
//         // memset(rx_buffer,1,32);
//         tx_buffer = "hello";
//         // sprintf((char*)tx_buffer,"hello world");
//         printf("sending %s, to spidev2.0 in full duplex \n ", (char*)tx_buffer);
//         // mySPI->xfer(tx_buffer,strlen((char*)tx_buffer),rx_buffer,strlen((char*)tx_buffer));
//         // printf("rx_buffer=%s\n",(char *)rx_buffer);
//         //mySPI->end();
//     }
//     delete mySPI;

//     return 1;
// }

static void delay_us(uint32_t us, void *intf_ptr) { usleep(us); }

// uint8_t acc_dev_addr = 0;
// uint8_t gyro_dev_addr = 0;
  

int main()
{
    spi_config_t spi_config;
    spi_config.mode = 0;
    spi_config.speed = 10000;
    spi_config.delay = 0;
    spi_config.bits_per_word = 8;

    SPI acc_spi = SPI("/dev/spidev0.0", &spi_config);  
    SPI gyro_spi = SPI("/dev/spidev0.1", &spi_config);  
    acc_spi.begin();
    gyro_spi.begin();

    bmi08x_dev dev;
    dev.intf_ptr_accel = &acc_spi;
    dev.intf_ptr_gyro = &gyro_spi;
    dev.read = (bmi_bus_read_function) &spi_read;
    dev.write = (bmi_bus_write_function) &spi_write;
    dev.delay_us = &delay_us;

    // Set up the rest of the required fields
    dev.variant = BMI088_VARIANT;
    dev.intf = BMI08X_SPI_INTF;

    

    int8_t rslt=0;
    printf("rslt: %d \n", rslt);
    rslt = bmi08a_init(&dev);
    printf("final rslt: %d \n", rslt);

    printf("rslt: %d \n", rslt);
    rslt = bmi08g_init(&dev);
    printf("final rslt: %d \n", rslt);

    bmi08a_get_power_mode(&dev);
    bmi08g_get_power_mode(&dev);
    bmi08a_get_meas_conf(&dev);
    bmi08g_get_meas_conf(&dev);
    
    dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    dev.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;

    dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
    dev.gyro_cfg.odr = BMI08X_GYRO_BW_47_ODR_400_HZ;

    // This is a wonky casting between the Bosch API, which usex
    // 3G … 24G as 0x00 ... 0x03 and 2000 dps ... 125 dps as 0x04 … 0x00
    dev.accel_cfg.range = 0x00; // (uint8_t)accel_fsr;
    dev.gyro_cfg.range = 0x03;  // 4 - (uint8_t)gyro_fsr;

    bmi08a_set_power_mode(&dev);
    bmi08g_set_power_mode(&dev);
    bmi08a_set_meas_conf(&dev);
    bmi08g_set_meas_conf(&dev);

    InertialMeasurement meas;

    BMI088 device{
        .bus_read = (bmi_bus_read_function) &spi_read,
        .bus_write = (bmi_bus_write_function) &spi_write,
        .intf_ptr_accel = &acc_spi,
        .intf_ptr_gyro = &gyro_spi
    };

    bmi088_init(&device);

    for (int i=0;i<10;++i)
    {
        bmi088_sample(&device, &meas);
        sleep(1);
    }
    
    return 0;
}