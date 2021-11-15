#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ei_run_classifier.h"
#include "./ei_classifier_porting.cpp"
//  MPU SCL -> PICO GPIO 5
//  MPU SDA -> PICO GPIO 4

#define ledPin 25
// The accelerometer's sensitivity per LSBi = 16384.0 for +-2g
#define accSensitivity 16384.0
// The gyroscope’s sensitivity per LSBi = 131.0 for +-250 deg
#define gyroSensitivity 131.0

volatile bool mpuReady = false;
int16_t acceleration[3], gyro[3];

// mpu6050 address
uint8_t mpuAddr = 0x68;

#ifdef i2c_default
static void setup_mpu()
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

    // set accelerometer full-scale to 2g, max sensitivity
    buffer[0] = 0x1C;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, true);

    // Set gyro full-scale to 250 degrees/sec, max sensitivity
    buffer[0] = 0x1B;
    buffer[1] = 0x00;
    i2c_write_blocking(i2c_default, mpuAddr, buffer, 2, false);

    sleep_ms(100);
}

static void read_mpu_data()
{
    uint8_t buffer[6];

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
}
#endif

float AccErrorX, AccErrorY, AccErrorZ;
float gyroErrorX, gyroErrorY, gyroErrorZ;
void calibrate_mpu()
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

        AccErrorX += accel_temp[0];
        AccErrorY += accel_temp[1];
        AccErrorZ += accel_temp[2];
        gyroErrorX += gyro_temp[0];
        gyroErrorY += gyro_temp[1];
        gyroErrorZ += gyro_temp[2];
    }
    // Normalize sums to get average count biases
    AccErrorX /= packet_count;
    AccErrorY /= packet_count;
    AccErrorZ /= packet_count;
    gyroErrorX /= packet_count;
    gyroErrorY /= packet_count;
    gyroErrorZ /= packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (AccErrorZ > 0L)
    {
        AccErrorZ -= accSensitivity;
    }
    else
    {
        AccErrorZ += accSensitivity;
    }

    AccErrorX /= accSensitivity;
    AccErrorY /= accSensitivity;
    AccErrorZ /= accSensitivity;
    gyroErrorX /= gyroSensitivity;
    gyroErrorY /= gyroSensitivity;
    gyroErrorZ /= gyroSensitivity;
}

bool timer_callback(struct repeating_timer *timer)
{
    gpio_put(ledPin, !gpio_get(ledPin));
    mpuReady = true;
    return true;
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

    setup_mpu();
    calibrate_mpu();

    struct repeating_timer outputTimer;
    // here a timer is set to sample the mpu every 16ms (62.5 khz)
    add_repeating_timer_ms(16, timer_callback, NULL, &outputTimer);
    while (true)
    {
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

        for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3)
        {
            // Determine the next tick (and then sleep later)
            uint64_t next_tick = ei_read_timer_us() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
            read_mpu_data();
            buffer[i] = acceleration[0] / accSensitivity;
            buffer[i + 1] = acceleration[1] / accSensitivity;
            buffer[i + 2] = acceleration[2] / accSensitivity;

            sleep_us(next_tick - ei_read_timer_us());
        }
        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        ei_impulse_result_t result = {0};
        err = run_classifier(&signal, &result, true);
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
                  result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        }

        ei_printf("    anomaly score: %.3f\n", result.anomaly);
        // if (mpuReady)
        // {
        //     read_mpu_data();
        //     printf("errors = %f\t%f\t%f\t%f\t%f\t%f\n", AccErrorX, AccErrorY, AccErrorZ, gyroErrorX, gyroErrorY, gyroErrorZ);
        //     printf("reads  = %f\t%f\t%f\t%f\t%f\t%f\n\n",
        //            ((float)acceleration[0] / accSensitivity) - AccErrorX, ((float)acceleration[1] / accSensitivity) - AccErrorY, ((float)acceleration[2] / accSensitivity) - AccErrorZ,
        //            ((float)gyro[0] / gyroSensitivity) - gyroErrorX, ((float)gyro[1] / gyroSensitivity) - gyroErrorY, ((float)gyro[2] / gyroSensitivity) - gyroErrorZ);

        //     // printf("Acc.X = %f\tAcc.Y = %f\tAcc.Z = %f\n", (float)acceleration[0] / accSensitivity, (float)acceleration[1] / accSensitivity, (float)acceleration[2] / accSensitivity);
        //     // printf("Gyro.X = %f\tGyro.Y = %f\tGyro.Z = %f\n\n", (float)gyro[0] / gyroSensitivity, (float)gyro[1] / gyroSensitivity, (float)gyro[2] / gyroSensitivity);
        //     mpuReady = false;
        // }
    }

    return 0;
}
