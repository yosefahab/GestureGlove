// Driver for the MPU6050 IMU over the Pico's default I2C pins.
#pragma once

#include <stdint.h>

// The accelerometer's sensitivity per LSB = 4096.0 for +-8g
#define MPU_ACC_SENSITIVITY 4096.0
// The gyroscope's sensitivity per LSB = 32.8 for +-1000 deg/s
#define MPU_GYRO_SENSITIVITY 32.8

// A reading in physical units: g for acceleration, deg/s for rotation.
// Values are scaled here rather than by the caller so the sensitivity
// constants stay next to the range configuration that determines them.
typedef struct
{
    float acceleration[3];
    float gyro[3];
} mpu_sample_t;

// Per-axis zero offsets, in the same units as mpu_sample_t.
typedef struct
{
    float acceleration[3];
    float gyro[3];
} mpu_bias_t;

// Reset the device and configure ranges, filtering and sample rate.
void mpu_setup(void);

// Average a short burst of samples to estimate the resting bias.
// Must be called after mpu_setup and while the device is held still.
mpu_bias_t mpu_calibrate(void);

// Read one acceleration and gyro sample.
mpu_sample_t mpu_read(void);
