#include "mpu6050.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// mpu6050 address
static const uint8_t mpuAddr = 0x68;

void mpu_setup(void)
{
    // reset mpu
    uint8_t buffer[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);
    sleep_ms(100);

    // disable all interrupts
    buffer[0] = 0x38;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // disable fifo
    buffer[0] = 0x23;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // turn on internal clock source(8 MHz) and disable temperature sensor
    buffer[0] = 0x6B;
    buffer[1] = 0x08;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // disable I2C master
    buffer[0] = 0x24;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // reset FIFO and DMP
    buffer[0] = 0x6A;
    buffer[1] = 0x0C;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    //  Configure gyro and accelerometer
    // sample rate = gyroscope output rate/(1 + SMPLRT_DIV)

    // set DLPF_CFG (Digital low-pass filter) to 188 Hz
    buffer[0] = 0x1A;
    buffer[1] = 0x01;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // since SMPLRT_DIV = 0 && gyro output rate = 1kHz -> sample rate = 1 Khz
    buffer[0] = 0x19;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // set accelerometer full-scale to +-10g, least sensitivity
    buffer[0] = 0x1C;
    buffer[1] = 0x10;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // Set gyro full-scale to +-1000 degrees/sec, least sensitivity
    buffer[0] = 0x1B;
    buffer[1] = 0x10;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, false);

    sleep_ms(100);
}

mpu_sample_t mpu_read(void)
{
    uint8_t buffer[6];
    int16_t acceleration[3], gyro[3];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t regAddr = 0x3B;
    i2c_write_blocking(i2c_default, mpuAddr, &regAddr, 1, true);
    i2c_read_blocking(i2c_default, mpuAddr, buffer, 6, true);

    for (int i = 0; i < 3; i++)
    {
        // ACCEL_XOUT[i*2] | ACCEL_XOUT[i*2+1]
        acceleration[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register auto increments on each read
    regAddr = 0x43;
    i2c_write_blocking(i2c_default, mpuAddr, &regAddr, 1, true);
    i2c_read_blocking(i2c_default, mpuAddr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    mpu_sample_t sample;
    for (int i = 0; i < 3; i++)
    {
        sample.acceleration[i] = acceleration[i] / MPU_ACC_SENSITIVITY;
        sample.gyro[i] = gyro[i] / MPU_GYRO_SENSITIVITY;
    }
    return sample;
}

mpu_bias_t mpu_calibrate(void)
{
    uint8_t regAddr[] = {0x6A, 0x40};

    // configure FIFO to capture accelerometer and gyro data for bias calculation

    // enable user control FIFO
    i2c_write_blocking(i2c_default, mpuAddr, regAddr, 2, true);
    // enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes)
    regAddr[0] = 0x23;
    regAddr[1] = 0x78;
    i2c_write_blocking(i2c_default, mpuAddr, regAddr, 2, true);

    // accumulate 80 samples in 80 milliseconds = 960 bytes
    sleep_ms(80);

    // disable FIFO
    regAddr[0] = 0x6A;
    regAddr[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, regAddr, 2, true);

    uint16_t fifo_count, packet_count;
    uint8_t data[12];

    // read FIFO sample count
    regAddr[0] = 0x72;
    i2c_write_blocking(i2c_default, mpuAddr, regAddr, 1, true);
    i2c_read_blocking(i2c_default, mpuAddr, data, 2, false);
    fifo_count = (data[0] << 8) | data[1];
    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count / 12;

    float accError[3] = {0}, gyroError[3] = {0};

    for (int i = 0; i < packet_count; i++)
    {
        // read data for averaging
        int16_t accel_temp[3], gyro_temp[3];
        regAddr[0] = 0x74;
        i2c_write_blocking(i2c_default, mpuAddr, regAddr, 1, true);
        i2c_read_blocking(i2c_default, mpuAddr, data, 12, false);

        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = ((data[0] << 8) | data[1]);
        accel_temp[1] = ((data[2] << 8) | data[3]);
        accel_temp[2] = ((data[4] << 8) | data[5]);
        gyro_temp[0] = ((data[6] << 8) | data[7]);
        gyro_temp[1] = ((data[8] << 8) | data[9]);
        gyro_temp[2] = ((data[10] << 8) | data[11]);

        for (int axis = 0; axis < 3; axis++)
        {
            accError[axis] += accel_temp[axis];
            gyroError[axis] += gyro_temp[axis];
        }
    }

    // Normalize sums to get average count biases
    for (int axis = 0; axis < 3; axis++)
    {
        accError[axis] /= packet_count;
        gyroError[axis] /= packet_count;
    }

    // Remove gravity from the z-axis accelerometer bias calculation
    if (accError[2] > 0L)
    {
        accError[2] -= MPU_ACC_SENSITIVITY;
    }
    else
    {
        accError[2] += MPU_ACC_SENSITIVITY;
    }

    mpu_bias_t bias;
    for (int axis = 0; axis < 3; axis++)
    {
        bias.acceleration[axis] = accError[axis] / MPU_ACC_SENSITIVITY;
        bias.gyro[axis] = gyroError[axis] / MPU_GYRO_SENSITIVITY;
    }
    return bias;
}
