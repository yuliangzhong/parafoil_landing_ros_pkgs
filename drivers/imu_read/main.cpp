#include <iostream>

#include "bmi088.h"
#include <unistd.h>

using namespace std;

int8_t read(uint8_t a, uint8_t *p, uint32_t b, void *q)
{
    return 0;
}
int8_t write(uint8_t a, const uint8_t *p, uint32_t b, void *q)
{
    return 0;
}


int main()
{
  
    InertialMeasurement meas;
    uint8_t acc_dev_addr = 0;
    uint8_t gyro_dev_addr = 0;

    BMI088 device{
        .bus_read = read,
        .bus_write = write,
        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr
    };

    bool rslt = bmi088_init(&device);

    for (int i=0;i<10;++i)
    {
        bmi088_sample(&device, &meas);
        sleep(1);
    }
    
    return 0;
}