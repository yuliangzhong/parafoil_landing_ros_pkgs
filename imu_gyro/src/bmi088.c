/*******************************************************************************
 * @file    bmi088.c
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Driver for the BMI088 Inertial Measurement Unit
 ******************************************************************************/

#include "bmi088.h"

#include <stdbool.h>

#include <stdio.h>
#include <unistd.h>

/* Private function declaration --------------------------------------------- */

/**
 * @brief Does a busy wait for some microseconds.
 * @param us Number of microseconds to wait.
 * @param intf_ptr pass whatever you want
 */
static void delay_us(uint32_t us, void *intf_ptr);

/* Local variable declaration ----------------------------------------------- */

static const char *TAG = "bmi088-driver";

/* Public function implementation ------------------------------------------- */

bool bmi088_init(struct BMI088 *bmi) {

    // Copy the interface pointers to the internal bmi08x_dev struct
    bmi->_dev.intf_ptr_accel = bmi->intf_ptr_accel;
    bmi->_dev.intf_ptr_gyro = bmi->intf_ptr_gyro;
    bmi->_dev.read = bmi->bus_read;
    bmi->_dev.write = bmi->bus_write;
    bmi->_dev.delay_us = &delay_us;

    // Set up the rest of the required fields
    bmi->_dev.variant = BMI088_VARIANT;
    bmi->_dev.intf = BMI08X_SPI_INTF;

    // Initiate communication
    int8_t rslt;

    if ((rslt = bmi08a_init(&bmi->_dev)) != BMI08X_OK) {
        printf("Failed to initialize accelerometer! error code %d\n", rslt);
        return false;
    }    
    
    if ((rslt = bmi08g_init(&bmi->_dev)) != BMI08X_OK) {
        printf("Failed to initialize gyroscope!\n");
        return false;
    }    

    bmi->_dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
    bmi->_dev.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
    bmi->_dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
    bmi->_dev.gyro_cfg.odr = BMI08X_GYRO_BW_47_ODR_400_HZ;

    // This is a wonky casting between the Bosch API, which usex
    // 3G … 24G as 0x00 ... 0x03 and 2000 dps ... 125 dps as 0x04 … 0x00
    bmi->_dev.accel_cfg.range = 0x00; // (uint8_t)accel_fsr;
    bmi->_dev.gyro_cfg.range = 0x03;  // 4 - (uint8_t)gyro_fsr;

    bmi08a_set_power_mode(&bmi->_dev);
    bmi08g_set_power_mode(&bmi->_dev);
    bmi08a_set_meas_conf(&bmi->_dev);
    bmi08g_set_meas_conf(&bmi->_dev);

    return true;
}

bool bmi088_sample(struct BMI088 *bmi088, InertialMeasurement *meas) {
    int8_t result = BMI08X_OK;
    struct bmi08x_sensor_data data;

    struct LinearAcceleration accel_meas = {};
    struct AngularVelocity gyro_meas = {};

    // Sample accelerometer
    result = bmi08a_get_data(&data, &bmi088->_dev);

    if (result == BMI08X_OK) {
        accel_meas.x = data.x / 10920.0f;
        accel_meas.y = data.y / 10920.0f;
        accel_meas.z = data.z / 10920.0f;
    } else {
        printf("Accelerometer sampling error (%d)", result);
        return false;
    }

    printf("Accelerations: a_x = %+.10f [g], a_y = %+.10f [g], a_z = %+.10f [g]\n", accel_meas.x, accel_meas.y, accel_meas.z);

    // Sample gyroscope
    result = bmi08g_get_data(&data, &bmi088->_dev);
    if (result == BMI08X_OK) {
        gyro_meas.x = data.x / 131.072f;
        gyro_meas.y = data.y / 131.072f;
        gyro_meas.z = data.z / 131.072f;
    } else {
        printf("Gyroscope sampling error (%d)", result);
        return false;
    }

    printf("Angular velocities: Ω_x = %+.2f [deg/s], Ω_y = %+.2f [deg/s], Ω_z = %+.2f [deg/s]\n", gyro_meas.x, gyro_meas.y, gyro_meas.z);

    meas->linear_accel = accel_meas;
    meas->angular_vel = gyro_meas;
    return true;
}

/* Private function implementation ------------------------------------------ */

static void delay_us(uint32_t us, void *intf_ptr) { usleep(us); }
