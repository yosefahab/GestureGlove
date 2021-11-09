#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

//  MPU SCL -> PICO GPIO 5
//  MPU SDA -> PICO GPIO 4

// pin definitions
#define ledPin 25

volatile bool mpuReady = false;
int16_t acceleration[3];
int16_t gyro[3];

// mpu6050 address
uint8_t mpuAddr = 0x68;

// The accelerometer's sensitivity per LSBi = 16384.0 for +-2g
#define accScale 16384.0
// The gyroscope’s sensitivity per LSBi = 131.0 for +-250 deg
#define gyroScale 131.0

#ifdef i2c_default
static void mpu_reset()
{
    // Two byte reset. First byte register, second byte data
    uint8_t buffer[] = {0x6B, 0x00};

    // reset mpu
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, false);
}

static void read_mpu_data()
{
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t regAddr = 0x3B;
    i2c_write_blocking(i2c_default, mpuAddr, &regAddr, 1, true);
    i2c_read_blocking(i2c_default, mpuAddr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        // ACCEL_XOUT[i*2] | ACCEL_XOUT[i*2+1]
        acceleration[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    regAddr = 0x43;
    i2c_write_blocking(i2c_default, mpuAddr, &regAddr, 1, true);
    i2c_read_blocking(i2c_default, mpuAddr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}
#endif

const float AccErrorX = -0.426656, AccErrorY = -0.015939, AccErrorZ = 0.985365;
const float gyroErrorX = 7.276831, gyroErrorY = 2.669962, gyroErrorZ = -2.213282;
// void calculate_IMU_error()
// {
//     float AccX, AccY, AccZ;
//     float GyroX, GyroY, GyroZ;
//     int c = 0;
//     while (c++ < 200)
//     {
//         read_mpu_data();
//         AccX = acceleration[0] / accScale;
//         AccY = acceleration[1] / accScale;
//         AccZ = acceleration[2] / accScale;

//         // TODO: find better way to calculate average error
//         AccErrorX = AccErrorX + AccX; // ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / M_PI));
//         AccErrorY = AccErrorY + AccY; // ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / M_PI));
//         AccErrorZ = AccErrorZ + AccZ;
//     }
//     //Divide the sum by 200 to get the error value
//     AccErrorX = AccErrorX / 200;
//     AccErrorY = AccErrorY / 200;
//     AccErrorZ = AccErrorZ / 200;
//     c = 0;
//     // Read gyro values 200 times
//     while (c++ < 200)
//     {
//         read_mpu_data();
//         GyroX = gyro[0] / gyroScale;
//         GyroY = gyro[1] / gyroScale;
//         GyroZ = gyro[2] / gyroScale;

//         gyroErrorX = gyroErrorX + GyroX;
//         gyroErrorY = gyroErrorY + GyroY;
//         gyroErrorZ = gyroErrorZ + GyroZ;
//     }
//     //Divide the sum by 200 to get the error value
//     gyroErrorX = gyroErrorX / 200;
//     gyroErrorY = gyroErrorY / 200;
//     gyroErrorZ = gyroErrorZ / 200;
//     // Print the error values on the Serial Monitor
// }
bool timer_callback(struct repeating_timer *timer)
{
    gpio_put(ledPin, !gpio_get(ledPin));
    read_mpu_data();
    mpuReady = true;
}

int main()
{

    // Initialize chosen serial port (usb in this case)
    stdio_init_all();
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // setup LED pin
    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    // Initialize I2C HW block, and set baud rate
    i2c_init(i2c_default, 62500);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    mpu_reset();
    // calculate_IMU_error();

    struct repeating_timer outputTimer;
    // here a timer is set to sample the mpu every 16ms (62.5 khz)
    add_repeating_timer_ms(16, timer_callback, NULL, &outputTimer);
    while (true)
    {
        if (mpuReady)
        {
            // printf("%f\t%f\t%f\n%f\t%f\t%f\n", AccErrorX, AccErrorY, AccErrorZ, gyroErrorX, gyroErrorY, gyroErrorZ); // -0.426656,-0.015939,0.985365,7.276831,2.669962,-2.213282
            printf("%f\t%f\t%f\t%f\t%f\t%f\n", ((float)acceleration[0] / accScale) - AccErrorX, ((float)acceleration[1] / accScale) - AccErrorY, ((float)acceleration[2] / accScale) - AccErrorZ,
                   ((float)gyro[0] / gyroScale) - gyroErrorX, ((float)gyro[1] / gyroScale) - gyroErrorY, ((float)gyro[2] / gyroScale) - gyroErrorZ);

            // printf("Acc.X = %f\tAcc.Y = %f\tAcc.Z = %f\n", (float)acceleration[0] / accScale, (float)acceleration[1] / accScale, (float)acceleration[2] / accScale);
            // printf("Gyro.X = %f\tGyro.Y = %f\tGyro.Z = %f\n", (float)gyro[0] / gyroScale, (float)gyro[1] / gyroScale, (float)gyro[2] / gyroScale);
            mpuReady = false;
        }
    }

    return 0;
}