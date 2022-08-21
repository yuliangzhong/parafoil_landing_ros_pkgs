/*******************************************************************************
 * @file    bmi088.h
 * @author  Lukas Vogel
 * @brief   Driver for the BMI088 Inertial Measurement Unit
 ******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bmi08x.h"
#include "control-types.h"

/**
 * An acceleration measurement from the built-in IMU has 16-bit resolution in
 * each direction. Depending on the full-scale range, accelerations up to ±24g
 * can be measured.
 *
 * The entries for the three dimensions have the unit LSB/g – to calculate the
 * acceleration in units of g, it is necessary to divide by the following
 * divisors:
 *
 * - ± 3g: 10920 LSB/g
 * - ± 6g:  5460 LSB/g
 * - ±12g:  2730 LSB/g
 * - ±24g:  1365 LSB/g
 */
enum BMI088AccelFullscaleRange {
    FSR_3G = 0,
    FSR_6G,
    FSR_12G,
    FSR_24G,
};

/**
 * @brief Data type for angular velocity measurements from an IMU.
 * An angular velocity measurement from the built-in IMU has 16-bit resolution
 * in each direction. Depending on the full-scale range, angular velocities up
 * to ±2000 deg/s can be measured.
 *
 * The entries for the three dimensions have the unit LSB/g – to calculate the
 * angular velocities in units of deg/s, it is necessary to divide by the
 * following divisors:
 *
 * - ± 125 deg/s: 262.144 LSB*s/deg
 * - ± 250 deg/s: 131.072 LSB*s/deg
 * - ± 500 deg/s:  65.536 LSB*s/deg
 * - ±1000 deg/s:  32.768 LSB*s/deg
 * - ±2000 deg/s:  16.384 LSB*s/deg
 *
 */
enum BMI088GyroFullscaleRange {
    FSR_125DPS = 0,
    FSR_250DPS,
    FSR_500DPS,
    FSR_1000DPS,
    FSR_2000DPS
};

typedef int8_t (*bmi_bus_read_function)(uint8_t, uint8_t *, uint32_t, void *);
typedef int8_t (*bmi_bus_write_function)(uint8_t, const uint8_t *, uint32_t,
                                         void *);

/**
 * @brief A BMI088 inertial measurement unit with an accelerometer and a
 * gyroscope.
 *
 * The BMI088 driver needs two function pointers for bus transactions: one for
 * writing data to a register over SPI and one to read data. In addition, the
 * intf_ptr field can be set before calling bmi088_setup(). The value of this
 * paramter will be passed to the bus read function as well.
 */
struct BMI088 {
    bmi_bus_read_function bus_read;
    bmi_bus_write_function bus_write;

    /**
     * Parameter that will be passed to @c bus_read and @c bus_write functions.
     * Set before calling bmi088_setup()!
     *
     * If the bus transaction is with the accelerometer part, then the
     * appropriate chip select pin will need to be pulled low.
     */
    void *intf_ptr_accel;

    /**
     * @brief Parameter that will be passed to @c bus_read and @c bus_write
     * functions if the bus transaction is with the gyroscope part.
     *
     * If the bus transaction is with the gyroscope part, then the appropriate
     * chip select pin will need to be pulled low.
     */
    void *intf_ptr_gyro;

    /** Internal struct, do not modify. */
    struct bmi08x_dev _dev;
};

/* Public function declarations --------------------------------------------- */

/**
 * @brief Set up the BMI088 sensor according to the parameters passed in @p bmi.
 *
 * @param bmi Struct holding device and bus information
 * @returns true if the BMI088 could be set up, false if an error occurred.
 */
bool bmi088_init(struct BMI088 *bmi);

/**
 * @brief Configure (or re-configure) the BMI088 to use a range setting.
 *
 * The accelerometer and gyroscope ranges can be varied.
 *
 * @param bmi The BMI088 struct
 * @param fsr_accel The acceleration full scale range to be set
 * @param fsr_gyro The gyroscope full scale range to be set
 * @returns true if the new ranges have been set, false otherwise
 */
bool bmi088_configure(struct BMI088 *bmi,
                      enum BMI088AccelFullscaleRange fsr_accel,
                      enum BMI088GyroFullscaleRange fsr_gyro);

/**
 * @brief Sample the BMI088 IMU.
 * @param bmi sensor to sample
 * @param meas The measurement obtained from the BMI088
 * @returns true if the sampling was successful, false otherwise
 */
bool bmi088_sample(struct BMI088 *bmi, InertialMeasurement *meas);

#ifdef __cplusplus
}
#endif
