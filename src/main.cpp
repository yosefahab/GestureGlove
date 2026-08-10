#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "./ei_classifier_porting.cpp"
#include "ei_run_classifier.h"
// generated from protocol.json, shared with the host receiver
#include "protocol.h"
#include "mpu6050.h"
//  MPU SCL -> PICO GPIO 5
//  MPU SDA -> PICO GPIO 4
//  HC-05 RX -> PICO GPIO 0 (UART TX)
//  HC-05 TX -> PICO GPIO 1 (UART RX)
#define ledPin 25
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_ID uart0

// standard-mode I2C, 100 kHz
#define I2C_RATE 100000

// Fill one inference window with consecutive readings, paced to the rate the
// model was trained at.
static void sample_window(float *buffer)
{
    for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
         i += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)
    {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = ei_read_timer_ms() + EI_CLASSIFIER_INTERVAL_MS;

        mpu_sample_t sample = mpu_read();
        buffer[i] = sample.acceleration[0];
        buffer[i + 1] = sample.acceleration[1];
        buffer[i + 2] = sample.acceleration[2];
        buffer[i + 3] = sample.gyro[0];
        buffer[i + 4] = sample.gyro[1];
        buffer[i + 5] = sample.gyro[2];

        sleep_ms(next_tick - ei_read_timer_ms());
    }
}

// Return the label of the highest scoring class in a classifier result.
static const char *best_label(const ei_impulse_result_t &result)
{
    int mxIdx = 0;
    float mx = -1;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
        if (result.classification[ix].value >= mx)
        {
            mx = result.classification[ix].value;
            mxIdx = ix;
        }
    }
    return result.classification[mxIdx].label;
}

static void init_peripherals(void)
{
    // Initialize chosen serial port (usb in this case)
    stdio_init_all();
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // setup LED pin
    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);

    // Initialize I2C HW block, and set baud rate
    i2c_init(i2c_default, I2C_RATE);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Initialize UART
    uart_init(UART_ID, GLOVE_BAUD_RATE);
    // 0 is TX 1 is RX
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

int main()
{
    init_peripherals();

    mpu_setup();
    // The bias is measured but deliberately not subtracted: the model was
    // trained on uncalibrated samples, so removing the offset now would shift
    // the input distribution away from what it learned.
    mpu_calibrate();

    while (true)
    {
        gpio_put(ledPin, !gpio_get(ledPin));

        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
        sample_window(buffer);

        signal_t signal;
        numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        ei_impulse_result_t result = {0};
        run_classifier(&signal, &result, false);

        // print class with highest probability
        printf("%s\n", best_label(result));
    }

    return 0;
}
