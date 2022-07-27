#include <iostream>

#include "bmi088.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <spidev_lib++.h>

using namespace std;

int8_t read(uint8_t reg_addr, uint8_t *data, uint32_t len, struct SPIDeviceHandle *handle) 
{
    return 0;
}
int8_t write(uint8_t reg_addr, const uint8_t *data, uint32_t len, struct SPIDeviceHandle *handle) 
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




int  main( void)
{  
    spi_config_t spi_config;
    uint8_t tx_buffer[32];
    uint8_t rx_buffer[32];

    SPI *mySPI = NULL;
    spi_config.mode=0;
    spi_config.speed=1000000;
    spi_config.delay=0;
    spi_config.bits_per_word=8;  
    
    mySPI = new SPI("/dev/spidev1.0", &spi_config);  
    
    if (mySPI->begin())
    {
        memset(tx_buffer,0,32);
        memset(rx_buffer,0,32);
        sprintf((char*)tx_buffer,"hello world");
        printf("sending %s, to spidev2.0 in full duplex \n ",(char*)tx_buffer);
        mySPI->xfer(tx_buffer,strlen((char*)tx_buffer),rx_buffer,strlen((char*)tx_buffer));
        printf("rx_buffer=%s\n",(char *)rx_buffer);
        cout<<rx_buffer<<endl;
        //mySPI->end();
        delete mySPI;
    }
    return 1;
}


// int main()
// {
//     spi_config_t spi_config;
//     uint8_t tx_buffer[32];
//     uint8_t rx_buffer[32];

//     InertialMeasurement meas;
//     uint8_t acc_dev_addr = 0;
//     uint8_t gyro_dev_addr = 0;

//     BMI088 device{
//         .bus_read = read,
//         .bus_write = write,
//         .intf_ptr_accel = &acc_dev_addr,
//         .intf_ptr_gyro = &gyro_dev_addr
//     };

//     bool rslt = bmi088_init(&device);

//     for (int i=0;i<10;++i)
//     {
//         bmi088_sample(&device, &meas);
//         sleep(1);
//     }
    
//     return 0;
// }