/*******************************************************************************
 * @file    control-types.hpp
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Defines types for common quantities used by sensors, controllers and
 *          estimators.
 ******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Quantity types ----------------------------------------------------------- */

/** A linear acceleration of a body [m/s^2]. */
struct LinearAcceleration {
    float x;
    float y;
    float z;
};

/** An angular velocity of a body [rad/s] */
struct AngularVelocity {
    float x;
    float y;
    float z;
};

/**
 * @brief Steering angle of a car [rad].
 *
 * By convention, the steering angle is defined by a rotation in the positive
 * direction around the z-axis, which points up.
 * This results in:
 *
 * - steering angle > 0 <=> car steers to left
 * - steering angle < 0 <=> car steers to right
 */
typedef float SteeringAngle;

/**
 * @brief Throttle torque applied to the steering wheels.
 *
 * This throttle is dimensionless and restricted to an interval of [-1, 1].
 * Positive values indicate *forward* acceleration of the vehicle, and vice
 * versa for negative values.
 */
typedef float Throttle;

/* Measurement types -------------------------------------------------------- */

typedef struct InertialMeasurement {
    struct LinearAcceleration linear_accel;
    struct AngularVelocity angular_vel;
} InertialMeasurement;

/** A raw ADC measurement. */
typedef uint16_t ADCMeasurement;

#ifdef __cplusplus
}
#endif
