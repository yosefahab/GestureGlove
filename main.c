#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "math.h"

//  MPU SCL -> PICO GPIO 5
//  MPU SDA -> PICO GPIO 4
//  MPU INT -> PICO GPIO 3

// pin definitions
#define ledPin 25
#define interruptPin 3

volatile bool mpuReady = false;
int16_t acceleration[3];
// int16_t gyro[3];

// mpu6050 address
uint8_t mpuAddr = 0x68;

#define mpuScale 16384.0
// The accelerometers’sensitivity per LSBi = 16384.0 for +-2g

#ifdef i2c_default
static void mpu_reset()
{
    // Two byte reset. First byte register, second byte data
    uint8_t buffer[] = {0x6B, 0x00};

    // reset mpu
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // interrupt setup (active high interrupt)
    buffer[0] = 0x37;
    buffer[1] = 0b010110;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, sizeof(buffer), true);
    // enable interrupts
    buffer[0] = 0x38;
    buffer[1] = 0x01;
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
    // regAddr = 0x43;
    // i2c_write_blocking(i2c_default, mpuAddr, &regAddr, 1, true);
    // i2c_read_blocking(i2c_default, mpuAddr, buffer, 6, false);

    // for (int i = 0; i < 3; i++)
    // {
    //     gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    // }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    // regAddr = 0x41;
    // i2c_write_blocking(i2c_default, mpuAddr, &regAddr, 1, true);
    // i2c_read_blocking(i2c_default, mpuAddr, buffer, 2, false);
}
#endif
void interrupt_callback(uint gpio, uint32_t events)
{
    mpuReady = true;
}

float AccErrorX, AccErrorY, AccErrorZ;
void calculate_IMU_error()
{
    float AccX, AccY, AccZ;
    // float GyroX, GyroY, GyroZ;
    int c = 0;
    while (c++ < 200)
    {
        read_mpu_data();
        AccX = acceleration[0] / mpuScale;
        AccY = acceleration[1] / mpuScale;
        AccZ = acceleration[2] / mpuScale;

        // TODO: find better way to calculate average error
        AccErrorX = AccErrorX + AccX; // ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / M_PI));
        AccErrorY = AccErrorY + AccY; // ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / M_PI));
        AccErrorZ = AccErrorZ + AccZ;
    }
    //Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    AccErrorZ = AccErrorZ / 200;
    // c = 0;
    // // Read gyro values 200 times
    // while (c < 200)
    // {
    //     i2c_write_blocking(MPU);
    //     Wire.write(0x43);
    //     Wire.endTransmission(false);
    //     Wire.requestFrom(MPU, 6, true);
    //     GyroX = Wire.read() << 8 | Wire.read();
    //     GyroY = Wire.read() << 8 | Wire.read();
    //     GyroZ = Wire.read() << 8 | Wire.read();
    //     // Sum all readings
    //     GyroErrorX = GyroErrorX + (GyroX / 131.0);
    //     GyroErrorY = GyroErrorY + (GyroY / 131.0);
    //     GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    //     c++;
    // }
    // //Divide the sum by 200 to get the error value
    // GyroErrorX = GyroErrorX / 200;
    // GyroErrorY = GyroErrorY / 200;
    // GyroErrorZ = GyroErrorZ / 200;
    // Print the error values on the Serial Monitor
}

int main()
{
    // by default pico runs at 125 mhz

    // Initialize chosen serial port (usb in this case)
    stdio_init_all();
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // setup LED & interrupt pins
    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    gpio_init(interruptPin);
    gpio_set_dir(interruptPin, GPIO_IN);
    gpio_pull_up(interruptPin);

    gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_RISE, 1, interrupt_callback);

    // Initialize I2C HW block, and set baud rate
    i2c_init(i2c_default, 115200);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    mpu_reset();
    calculate_IMU_error();

    while (true)
    {
        printf("AccErrorX: %f\tAccErrorY: %f\tAccErrorZ: %f\n", AccErrorX, AccErrorY, AccErrorZ);
        if (mpuReady)
        {
            mpuReady = false;
            gpio_put(ledPin, !gpio_get(ledPin));

            read_mpu_data();
            // printf("%f,%f,%f,%d,%d,%d\n", acceleration[0] / mpuScale, acceleration[1] / mpuScale, acceleration[2] / mpuScale, gyro[0], gyro[1], gyro[2]);
            printf("Acc.X = %f\tAcc.Y = %f\tAcc.Z = %f\n", (float)acceleration[0] / mpuScale, (float)acceleration[1] / mpuScale, (float)acceleration[2] / mpuScale);
            // printf("Gyro. X = %d\tY = %d\tZ = %d\n", gyro[0], gyro[1], gyro[2]);
        }
        sleep_ms(100);
    }

    return 0;
}